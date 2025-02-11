/*
 * Linear and exponential LUT-based shaping on left/right volumes,
 * with a "plateau" center zone. By default, the code runs in a high-speed loop,
 * reading ADCs, applying EWMA filtering to pot inputs, then applying either
 * linear or exponential shaping on each side (controlled by digital inputs).
 *
 * If DEBUG_LINLOG == 1, additional debug prints are performed, showing
 * the iteration frequency etc. The LUT is normalized so that at ratio=1,
 * we reach exactly 1.0 (no leftover mismatch).
 */

#include "mbed.h"
#include <cstdint>
#include <cstdio>
#include <cmath>

// ===================== DEBUG MODE MACRO =====================
// Set to 1 to enable extended debug
// Then you can parse logs with a Python script if desired.
#define DEBUG_LINLOG 0

// ===================== Constants & LUT Config =====================
static const float UI16_MAX_F   = 65535.0f;    // 16-bit max as float
static const float CENTER_WIDTH = 0.1f;        // "Plateau" fraction
static const float VREF         = 3.3f;        // Voltage reference for debug display

// Compute half the plateau in float range
static const float center_width_float = CENTER_WIDTH * UI16_MAX_F * 0.5f;

// Remaining slider range after removing the plateau from both ends
static const float slider_length_minus_center_float =
    UI16_MAX_F - (2.0f * center_width_float);

// LUT resolution
#define LUT_SIZE 2048
static float lutExpUp[LUT_SIZE + 1]; // Single LUT array for "up" shape

// Shaping constant: adjust for steeper or gentler exponential curve
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
 * Build the LUT:
 *   y = (1 - exp(-c*x)) / (1 - exp(-c))
 * ensures at x=0 => y=0, and x=1 => y=1
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

// ===================== LUT interpolation =====================
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
 * Left side shape: linear (ratio) or exponential (LUT).
 */
inline float shapeLeftLUT(float ratio, bool useLog)
{
    if (!useLog) {
        // Linear
        return ratio;
    } else {
        // Exponential up
        return interpolateLUT(ratio);
    }
}

/*
 * Right side shape: invert linear or invert exp by sampling LUT at (1 - ratio).
 */
inline float shapeRightLUT(float ratio, bool useLog)
{
    if (!useLog) {
        // Linear invert => 1 - ratio
        return 1.0f - ratio;
    } else {
        // Invert exponential => use LUT(1 - ratio)
        return interpolateLUT(1.0f - ratio);
    }
}

// ===================== computeVolume() helpers =====================
/**
 * Compute the final scaled volume on the left side.
 *
 * ratio: [0..1]
 * offset: e.g. left_fil_f
 * range: (65535 - left_fil_f)
 * centerScale: factor from center pot
 * useLog: true => exponential, false => linear
 */
inline uint16_t computeVolumeLeft(float ratio, float offset, float range, float centerScale, bool useLog)
{
    float shaped = shapeLeftLUT(ratio, useLog);
    float volume_f = offset + shaped * range;
    float volumeScaled_f = volume_f * centerScale;

    if (volumeScaled_f < 0.f) {
        volumeScaled_f = 0.f;
    } else if (volumeScaled_f > UI16_MAX_F) {
        volumeScaled_f = UI16_MAX_F;
    }
    return (uint16_t)volumeScaled_f;
}

/**
 * Compute the final scaled volume on the right side.
 */
inline uint16_t computeVolumeRight(float ratio, float offset, float range, float centerScale, bool useLog)
{
    float shaped = shapeRightLUT(ratio, useLog);
    float volume_f = offset + shaped * range;

    if (volume_f < 0.f) {
        volume_f = 0.f;
    }
    if (volume_f > UI16_MAX_F) {
        volume_f = UI16_MAX_F;
    }

    float volumeScaled_f = volume_f * centerScale;
    if (volumeScaled_f < 0.f) {
        volumeScaled_f = 0.f;
    }
    if (volumeScaled_f > UI16_MAX_F) {
        volumeScaled_f = UI16_MAX_F;
    }
    return (uint16_t)volumeScaled_f;
}

// ===================== Globals & MBED objects =====================
static const float ALPHA_POTS = 0.06f;

// Raw/filt values
float cv_raw_f       = 0.0f;
float slider_raw_f   = 0.0f, slider_fil_f   = 0.0f; bool init_slider   = false;
float center_raw_f   = 0.0f, center_fil_f   = 0.0f; bool init_center   = false;
float left_raw_f     = 0.0f, left_fil_f     = 0.0f; bool init_left     = false;
float right_raw_f    = 0.0f, right_fil_f    = 0.0f; bool init_right    = false;

// Volumes
uint16_t volume = 0, volume_left = 0, volume_right = 0;

// "Plateau" boundaries
float center_from_slider_f = 0.0f;
float left_slide_point_f   = 0.0f;
float right_slide_point_f  = 0.0f;

// For iteration measurement
volatile uint32_t refresh = 0, old_refresh = 0;

// Analog/digital IO
AnalogIn  cv_input(A6);
AnalogIn  slider_input(A2);
AnalogIn  center_input(D3);
AnalogIn  left_input(A0);
AnalogIn  right_input(A1);

AnalogOut filtered_output(PA_4);

DigitalIn but_r_lin_log(PB_5);
DigitalIn but_l_lin_log(PB_4);

// Timer
Timer t;

// ===================== Debug function =====================
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

// ===================== MAIN =====================
int main()
{
    but_r_lin_log.mode(PullUp);
    but_l_lin_log.mode(PullUp);

    t.start();
    buildLUT(SHAPE_C);

    uint32_t lastDbgMs = 0;
    uint32_t lastRefMs = 0;

    while (true) {

        // 1) Read raw ADC & convert to [0..65535]
        cv_raw_f      = cv_input.read()     * UI16_MAX_F;
        slider_raw_f  = slider_input.read() * UI16_MAX_F;
        center_raw_f  = center_input.read() * UI16_MAX_F;
        left_raw_f    = left_input.read()   * UI16_MAX_F;
        right_raw_f   = right_input.read()  * UI16_MAX_F;

        // 2) Filter pot values via EWMA
        slider_fil_f = ewma_filter_float(slider_raw_f, slider_fil_f, ALPHA_POTS, init_slider);
        center_fil_f = ewma_filter_float(center_raw_f, center_fil_f, ALPHA_POTS, init_center);
        left_fil_f   = ewma_filter_float(left_raw_f,   left_fil_f,   ALPHA_POTS, init_left);
        right_fil_f  = ewma_filter_float(right_raw_f,  right_fil_f,  ALPHA_POTS, init_right);

        // 3) Compute plateau boundaries from slider
        float fraction_slider = slider_fil_f / UI16_MAX_F;
        center_from_slider_f  = center_width_float + fraction_slider * slider_length_minus_center_float;
        left_slide_point_f    = center_from_slider_f - center_width_float;
        right_slide_point_f   = center_from_slider_f + center_width_float;

        // Factor from center pot
        float center_scale = center_fil_f / UI16_MAX_F;

        // Decide shaping:
        // e.g. if read()==1 => use exponential
        bool useLogLeft  = (but_l_lin_log.read() == 1);
        bool useLogRight = (but_r_lin_log.read() == 1);

        // 4) Plateau logic
        if (cv_raw_f < left_slide_point_f) {
            // -- LEFT ZONE --
            float ratio_l = 0.f;
            if (left_slide_point_f > 0.f) {
                ratio_l = cv_raw_f / left_slide_point_f;
            }
            if (ratio_l < 0.f) ratio_l = 0.f;
            if (ratio_l > 1.f) ratio_l = 1.f;

            float offset_l = left_fil_f;
            float range_l  = (UI16_MAX_F - left_fil_f);

            volume_left  = computeVolumeLeft(ratio_l, offset_l, range_l, center_scale, useLogLeft);
            volume       = volume_left;
            volume_right = 0;

        } else if (cv_raw_f > right_slide_point_f) {
            // -- RIGHT ZONE --
            float diff_cv = cv_raw_f - right_slide_point_f;
            float denom_r = (UI16_MAX_F - right_slide_point_f);
            float ratio_r = (denom_r > 0.f) ? (diff_cv / denom_r) : 0.f;
            if (ratio_r < 0.f) ratio_r = 0.f;
            if (ratio_r > 1.f) ratio_r = 1.f;

            float offset_r = right_fil_f;
            float range_r  = (UI16_MAX_F - right_fil_f);

            volume_right = computeVolumeRight(ratio_r, offset_r, range_r, center_scale, useLogRight);
            volume       = volume_right;
            volume_left  = 0;

        } else {
            // -- CENTER ZONE --
            volume       = (uint16_t)center_fil_f;
            volume_left  = 0;
            volume_right = 0;
        }

        // 5) Write to DAC
        filtered_output.write_u16(volume);

#if DEBUG_LINLOG
        // 6) Count iterations
        refresh++;

        // 7) Timer for stats
        uint32_t nowMs = t.read_ms();
        if ((nowMs - lastRefMs) >= 1000) {
            lastRefMs = nowMs;
            old_refresh = refresh;
            refresh = 0;
        }
        // Print debug e.g. every 50 ms
        if ((nowMs - lastDbgMs) >= 50) {
            lastDbgMs = nowMs;
            big_console_debug();
        }
#endif
    }
}
