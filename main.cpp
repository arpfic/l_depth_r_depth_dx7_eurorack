/*
 * DX7 like L&R Depth.
 * Linear & exponential LUT-based shaping on left/right volumes,
 * with a center "plateau" zone. If the side pot is larger than
 * the center pot, we invert that side’s slope, allowing \_/
 * shapes as well as /¯\ shapes.
 */

#include "mbed.h"
#include <cstdint>
#include <cstdio>
#include <cmath>

// ===================== DEBUG MODE MACRO =====================
#define DEBUG_LINLOG 1  // set to 1 to enable extended debug prints

// ===================== Constants & LUT Config =====================
static const float UI16_MAX_F   = 65535.0f;
static const float CENTER_WIDTH = 0.1f;  // fraction of [0..1]
static const float VREF         = 3.3f;  // for debug display

// Half the plateau (in float range)
static const float center_width_float = CENTER_WIDTH * UI16_MAX_F * 0.5f;

// Remaining slider range after removing the plateau from both ends
static const float slider_length_minus_center_float =
    UI16_MAX_F - (2.0f * center_width_float);

// LUT resolution
#define LUT_SIZE 2048
static float lutExpUp[LUT_SIZE + 1];

// Shaping constant for exponential slope
static const float SHAPE_C = 4.0f;

// ===================== EWMA for pot smoothing =====================
inline float ewma_filter_float(float in, float out_prev, float alpha, bool &initDone)
{
    if (!initDone) {
        initDone = true;
        return in;
    }
    return out_prev + alpha * (in - out_prev);
}

/*
 * Build the LUT so at x=0 => 0, at x=1 => 1:
 *   y = (1 - exp(-c*x)) / (1 - exp(-c))
 */
static void buildLUT(float c)
{
    for (int i = 0; i <= LUT_SIZE; i++) {
        float x = (float)i / (float)LUT_SIZE;
        float denom = 1.0f - expf(-c);
        float up = (1.0f - expf(-c * x)) / denom;
        lutExpUp[i] = up;
    }
}

/*
 * Interpolate in lutExpUp[] for x in [0..1].
 */
inline float interpolateLUT(float x)
{
    if (x < 0.f) x = 0.f;
    if (x > 1.f) x = 1.f;
    float fx = x * (float)LUT_SIZE;
    int idx = (int)fx;
    if (idx >= LUT_SIZE) {
        idx = LUT_SIZE - 1;
    }
    float frac = fx - (float)idx;
    float v0 = lutExpUp[idx];
    float v1 = lutExpUp[idx + 1];
    return v0 + frac * (v1 - v0);
}

// ===================== Shaping Functions =====================
/*
 * shapeLeft(ratio, useLog):
 *  - if useLog => LUT(ratio)
 *  - else => ratio (linear)
 */
inline float shapeLeft(float ratio, bool useLog)
{
    return useLog ? interpolateLUT(ratio) : ratio;
}

/*
 * shapeRight(ratio, useLog):
 *  - if useLog => LUT(1-ratio) => invert exp
 *  - else => 1-ratio => invert linear
 */
inline float shapeRight(float ratio, bool useLog)
{
    return useLog ? interpolateLUT(1.f - ratio) : (1.f - ratio);
}

// ===================== computeVolume Helpers =====================
/**
 * @brief computeVolumeLeft
 *  - If center > left pot => we do the "usual" slope from left pot up to center pot,
 *    or up to a higher amplitude. 
 *  - If center < left pot => we invert the slope so it goes downward from left pot to center.
 *    e.g. \ shape.
 */
inline uint16_t computeVolumeLeft(float ratio, float center_val, float left_val, bool useLog)
{
    // If center is bigger, offset = left_val, range = (center_val - left_val), shaping is normal
    // If left_val is bigger, offset = center_val, range = (left_val - center_val), shaping is "inverted"
    bool sideIsBigger = (left_val > center_val);

    float offset = sideIsBigger ? center_val : left_val;
    float range  = fabsf(center_val - left_val);

    // shape factor
    float shaped = shapeLeft(ratio, useLog);
    // if side is bigger => shaped=0 => center_val, shaped=1 => left_val => reversed slope
    // so if sideIsBigger => final = offset + (1 - shaped)*range
    // else => final = offset + shaped*range
    float final;
    if (sideIsBigger) {
        final = offset + (1.f - shaped) * range;
    } else {
        final = offset + shaped * range;
    }

    // clamp
    if (final < 0.f) final = 0.f;
    if (final > UI16_MAX_F) final = UI16_MAX_F;
    return (uint16_t)(final);
}

/**
 * @brief computeVolumeRight
 * Similar idea but for the right pot vs. center pot.
 */
inline uint16_t computeVolumeRight(float ratio, float center_val, float right_val, bool useLog)
{
    bool sideIsBigger = (right_val > center_val);

    float offset = sideIsBigger ? center_val : right_val;
    float range  = fabsf(center_val - right_val);

    float shaped = shapeRight(ratio, useLog);
    // If right_val is bigger => final= offset+(1 - shaped)*range
    // else => offset+ shaped*range
    float final;
    if (sideIsBigger) {
        final = offset + (1.f - shaped) * range;
    } else {
        final = offset + shaped * range;
    }

    if (final < 0.f) final = 0.f;
    if (final > UI16_MAX_F) final = UI16_MAX_F;
    return (uint16_t)(final);
}

// ===================== Globals & MBED objects =====================
static const float ALPHA_POTS = 0.06f;

float cv_raw_f     = 0.0f;
float slider_raw_f = 0.0f, slider_fil_f = 0.0f; bool init_slider = false;
float center_raw_f = 0.0f, center_fil_f = 0.0f; bool init_center = false;
float left_raw_f   = 0.0f, left_fil_f   = 0.0f; bool init_left   = false;
float right_raw_f  = 0.0f, right_fil_f  = 0.0f; bool init_right  = false;

// Output volumes
uint16_t volume = 0, volume_left = 0, volume_right = 0;

float center_from_slider_f = 0.0f;
float left_slide_point_f   = 0.0f;
float right_slide_point_f  = 0.0f;

volatile uint32_t refresh = 0, old_refresh = 0;

// I/O
AnalogIn  cv_input(A6);
AnalogIn  slider_input(A2);
AnalogIn  center_input(D3);
AnalogIn  left_input(A0);
AnalogIn  right_input(A1);

AnalogOut filtered_output(PA_4);

DigitalIn but_r_lin_log(PB_5);
DigitalIn but_l_lin_log(PB_4);

Timer t;

#if DEBUG_LINLOG
static void big_console_debug()
{
    float cv_volt = (cv_raw_f / UI16_MAX_F) * VREF;
 printf(
        "CV=%.1f/%.2fV vol=%u L=%u/%f R=%u/%f Hz=%lu\n",
        cv_raw_f,
        cv_volt,
        volume,
        volume_left,
        (float)volume_left / UI16_MAX_F,
        volume_right,
        (float)volume_right / UI16_MAX_F,
        (unsigned long)old_refresh
    );
    fflush(stdout);
}
#endif

// ===================== MAIN =====================
int main()
{
    but_r_lin_log.mode(PullUp);
    but_l_lin_log.mode(PullUp);

    t.start();
    buildLUT(SHAPE_C);

#if DEBUG_LINLOG
    uint32_t lastDbgMs = 0;
    uint32_t lastRefMs = 0;
#endif

    while (true) {

        // 1) ADC readings => [0..65535]
        cv_raw_f     = cv_input.read()     * UI16_MAX_F;
        slider_raw_f = slider_input.read() * UI16_MAX_F;
        center_raw_f = center_input.read() * UI16_MAX_F;
        left_raw_f   = left_input.read()   * UI16_MAX_F;
        right_raw_f  = right_input.read()  * UI16_MAX_F;

        // 2) Filter pot values (EWMA)
        slider_fil_f = ewma_filter_float(slider_raw_f, slider_fil_f, ALPHA_POTS, init_slider);
        center_fil_f = ewma_filter_float(center_raw_f, center_fil_f, ALPHA_POTS, init_center);
        left_fil_f   = ewma_filter_float(left_raw_f,   left_fil_f,   ALPHA_POTS, init_left);
        right_fil_f  = ewma_filter_float(right_raw_f,  right_fil_f,  ALPHA_POTS, init_right);

        // 3) Plateau boundaries from slider
        float fraction_slider = slider_fil_f / UI16_MAX_F;
        center_from_slider_f  = center_width_float + fraction_slider * slider_length_minus_center_float;
        left_slide_point_f    = center_from_slider_f - center_width_float;
        right_slide_point_f   = center_from_slider_f + center_width_float;

        // 4) Decide shaping from buttons
        bool useLogLeft  = (but_l_lin_log.read() == 1);
        bool useLogRight = (but_r_lin_log.read() == 1);

        // 5) Main logic
        if (cv_raw_f < left_slide_point_f) {
            // LEFT ZONE => ratio in [0..1]
            float ratio_l = 0.f;
            if (left_slide_point_f > 0.f) {
                ratio_l = cv_raw_f / left_slide_point_f;
            }
            if (ratio_l < 0.f) ratio_l = 0.f;
            if (ratio_l > 1.f) ratio_l = 1.f;

            volume_left = computeVolumeLeft(ratio_l, center_fil_f, left_fil_f, useLogLeft);
            volume = volume_left;
            volume_right = 0;
        }
        else if (cv_raw_f > right_slide_point_f) {
            // RIGHT ZONE => ratio in [0..1]
            float diff_cv = cv_raw_f - right_slide_point_f;
            float denom_r = (UI16_MAX_F - right_slide_point_f);
            float ratio_r = (denom_r > 0.f) ? (diff_cv / denom_r) : 0.f;
            if (ratio_r < 0.f) ratio_r = 0.f;
            if (ratio_r > 1.f) ratio_r = 1.f;

            volume_right = computeVolumeRight(ratio_r, center_fil_f, right_fil_f, useLogRight);
            volume = volume_right;
            volume_left = 0;
        }
        else {
            // CENTER ZONE => volume = center pot
            volume       = (uint16_t)center_fil_f;
            volume_left  = 0;
            volume_right = 0;
        }

        // 6) DAC output
        filtered_output.write_u16(volume);

#if DEBUG_LINLOG
        // Optional debug/measurement
        refresh++;
        uint32_t nowMs = t.read_ms();
        if ((nowMs - lastRefMs) >= 1000) {
            lastRefMs = nowMs;
            old_refresh = refresh;
            refresh = 0;
        }
        if ((nowMs - lastDbgMs) >= 50) {
            lastDbgMs = nowMs;
            big_console_debug();
        }
#endif
    }
}
