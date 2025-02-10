#include "mbed.h"
#include <cstdint>
#include <cstdio>

/**
 * This program reads five analog inputs (CV, slider, center, left, right).
 * The pots (slider, center, left, right) are filtered using an EWMA,
 * while the CV input is used unfiltered.
 * A "plateau" logic determines the final volume output, written to a DAC.
 *
 * A Timer is used for:
 *  - Measuring the iteration rate (refresh count per second).
 *  - Triggering a console print every 1 second (bare-metal, no threads).
 *
 * Compiled on a Cortex-M4F with FPU and -O3 (plus LTO if desired),
 * this can achieve high iteration rates.
 */

// ======================== Constants =========================

// Range scale (0..65535) as float
static const float UI16_MAX_F = 65535.0f;

// Reference voltage for debug output
static const float VREF = 3.3f;

// Width of the central "plateau" (0..1 fraction)
static const float CENTER_WIDTH = 0.2f;

// half-plateau in float: 0.2 * 65535 / 2
static const float center_width_float = CENTER_WIDTH * UI16_MAX_F * 0.5f;

// The slider region excluding the plateau = 65535 - 2*(plateau half)
static const float slider_length_minus_center_float =
    UI16_MAX_F - (2.0f * center_width_float);

// ======================== EWMA (Exponential Weighted Moving Average) =========================
// Only used for pots, not for CV.
inline float ewma_filter_float(float in, float out_prev, float alpha, bool &initDone)
{
    if (!initDone) {
        initDone = true;
        return in;
    }
    // output = out_prev + alpha*(in - out_prev)
    return out_prev + alpha * (in - out_prev);
}

// ======================== Global variables =========================

// --- CV: unfiltered ---
float cv_raw_f = 0.0f; // [0..65535] float

// --- Pots: raw + filtered ---
float slider_raw_f = 0.0f;
float center_raw_f = 0.0f;
float left_raw_f   = 0.0f;
float right_raw_f  = 0.0f;

float slider_fil_f = 0.0f;
float center_fil_f = 0.0f;
float left_fil_f   = 0.0f;
float right_fil_f  = 0.0f;

// Initialization flags for the four pot filters
bool  init_slider  = false;
bool  init_center  = false;
bool  init_left    = false;
bool  init_right   = false;

// Filtering coefficient for pots (slider, center, left, right)
static const float ALPHA_POTS = 0.06f;

// Final volume outputs
uint16_t volume       = 0;
uint16_t volume_left  = 0;
uint16_t volume_right = 0;
uint16_t superdebug   = 0;

// Plateau boundaries
float center_from_slider_f = 0.0f;
float left_slide_point_f   = 0.0f;
float right_slide_point_f  = 0.0f;

// Refresh rate measurement
volatile uint32_t refresh     = 0;
volatile uint32_t old_refresh = 0;

// ======================== Mbed objects =========================

// Analog inputs
AnalogIn cv_input(A6);
AnalogIn slider_input(A2);
AnalogIn center_input(D3);
AnalogIn left_input(A0);
AnalogIn right_input(A1);

// Analog output (DAC)
AnalogOut filtered_output(PA_4);

// Digital inputs (buttons)
DigitalIn but_r_lin_log(PB_5);
DigitalIn but_l_lin_log(PB_4);

// Timer (no more thread)
Timer t;

/**
 * Extended console debug: prints info every 1 second.
 * The CV is shown raw, while the pot values are displayed as filtered ones.
 */
static void big_console_debug()
{
    float cv_volt = (cv_raw_f / UI16_MAX_F) * VREF;

    printf(
        "CV=%.1f/65535(%.2fV) OUT=%.3f SLIDER=%.3f CENTER=%.3f L=%.3f R=%.3f "
        "Hz=%lu vol=%u left=%u right=%u debug=%u\n",
        cv_raw_f,          // Raw CV input
        cv_volt,           // Approx voltage from CV
        filtered_output.read(), // DAC output in [0..1]
        slider_fil_f,      // Filtered slider
        center_fil_f,      // Filtered center pot
        left_fil_f,        // Filtered left pot
        right_fil_f,       // Filtered right pot
        (unsigned long)old_refresh,
        volume,
        volume_left,
        volume_right,
        superdebug
    );
    fflush(stdout);
}

/**
 * Plateau logic:
 *  - We define a middle zone around 'center_from_slider_f'
 *    with half-width 'center_width_float'.
 *  - left_slide_point_f  = center_from_slider_f - center_width_float
 *  - right_slide_point_f = center_from_slider_f + center_width_float
 * If CV < left_slide_point_f => left region
 * If CV > right_slide_point_f => right region
 * Else => center region
 *
 * We also multiply the final volume by 'center_fil_f / UI16_MAX_F'
 * to scale with the center pot.
 */

int main()
{
    // Configure pull-ups
    but_r_lin_log.mode(PullUp);
    but_l_lin_log.mode(PullUp);

    // Start the timer
    t.start();

    uint32_t lastBigConsoleMs   = 0;
    uint32_t lastRefreshCheckMs = 0;

    while (true) {
        // 1) Read ADC in float [0..1] then scale to [0..65535]
        // CV is unfiltered => stored in cv_raw_f
        cv_raw_f      = cv_input.read()     * UI16_MAX_F;

        // Pots => raw + filtered
        slider_raw_f  = slider_input.read() * UI16_MAX_F;
        center_raw_f  = center_input.read() * UI16_MAX_F;
        left_raw_f    = left_input.read()   * UI16_MAX_F;
        right_raw_f   = right_input.read()  * UI16_MAX_F;

        // 2) Filter the pot values (EWMA)
        slider_fil_f  = ewma_filter_float(slider_raw_f, slider_fil_f, ALPHA_POTS, init_slider);
        center_fil_f  = ewma_filter_float(center_raw_f, center_fil_f, ALPHA_POTS, init_center);
        left_fil_f    = ewma_filter_float(left_raw_f,   left_fil_f,   ALPHA_POTS, init_left);
        right_fil_f   = ewma_filter_float(right_raw_f,  right_fil_f,  ALPHA_POTS, init_right);

        // 3) Calculate center_from_slider in [0..65535]
        float fraction_slider = slider_fil_f / UI16_MAX_F;
        center_from_slider_f  = center_width_float + fraction_slider * slider_length_minus_center_float;

        left_slide_point_f  = center_from_slider_f - center_width_float;
        right_slide_point_f = center_from_slider_f + center_width_float;

        // 4) center_scale => factor from the center pot
        float center_scale = center_fil_f / UI16_MAX_F;

        // 5) Main plateau logic
        if (cv_raw_f < left_slide_point_f) {
            superdebug = 1;

            float left_span = UI16_MAX_F - left_fil_f;
            float denom_l   = left_slide_point_f;
            float left_scale = 0.0f;

            if (denom_l > 0.0f) {
                left_scale = left_span / denom_l;
            }

            // volume_left = offset + scaled portion
            volume_left  = (uint16_t)(left_scale * cv_raw_f + left_fil_f);
            volume       = (uint16_t)((float)volume_left * center_scale);
            volume_right = 0;

        } else if (cv_raw_f > right_slide_point_f) {
            superdebug = 2;

            float right_span = UI16_MAX_F - right_fil_f;
            float denom_r    = UI16_MAX_F - right_slide_point_f;
            float right_scale= 0.0f;

            if (denom_r > 0.0f) {
                right_scale = right_span / denom_r;
            }

            float diff_cv = cv_raw_f - right_slide_point_f;
            float vr_f    = UI16_MAX_F - (right_scale * diff_cv);

            // Clamp
            if (vr_f < 0.0f) {
                vr_f = 0.0f;
            } else if (vr_f > UI16_MAX_F) {
                vr_f = UI16_MAX_F;
            }

            volume_right = (uint16_t)vr_f;
            volume       = (uint16_t)((float)volume_right * center_scale);
            volume_left  = 0;

        } else {
            // center zone
            superdebug   = 0;
            volume       = (uint16_t)(center_fil_f);
            volume_left  = 0;
            volume_right = 0;
        }

        // 6) DAC output
        filtered_output.write_u16(volume);

        // 7) Increment iteration counter
        refresh++;

        // 8) Timer checks
        uint32_t nowMs = t.read_ms();

        // a) Once per second => update old_refresh
        if ((nowMs - lastRefreshCheckMs) >= 1000) {
            lastRefreshCheckMs = nowMs;
            old_refresh = refresh;
            refresh = 0;
        }

        // b) Print debug once per second
        if ((nowMs - lastBigConsoleMs) >= 1000) {
            lastBigConsoleMs = nowMs;
            //big_console_debug();
            printf("%lu Hz | %f%%\n",
                   (unsigned long)old_refresh,
                   filtered_output.read());
        }
    }
}