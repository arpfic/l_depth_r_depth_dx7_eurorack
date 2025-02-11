#include "mbed.h"
#include <cstdint>
#include <cstdio>
#include <cmath>

// ===================== DEBUG MODE MACRO =====================
#define DEBUG_LINLOG 1  // Set to 1 to enable double-calculation & extended debug

// ===================== Constants & LUT Config =====================
static const float UI16_MAX_F   = 65535.0f;
// "Plateau" width
static const float CENTER_WIDTH = 0.1f;
static const float VREF         = 3.3f;

static const float center_width_float = CENTER_WIDTH * UI16_MAX_F * 0.5f;
static const float slider_length_minus_center_float =
    UI16_MAX_F - (2.0f * center_width_float);

// LUT Resolution
#define LUT_SIZE 2048
static float lutExpUp[LUT_SIZE + 1];

// Shaping constant
static const float SHAPE_C = 4.0f;  // Adjust for steeper or gentler curve

// EWMA for pot smoothing
inline float ewma_filter_float(float in, float out_prev, float alpha, bool &initDone)
{
    if (!initDone) {
        initDone = true;
        return in;
    }
    return out_prev + alpha * (in - out_prev);
}

// Build LUT => y = 1 - exp(-c*x)
static void buildLUT(float c)
{
    for (int i = 0; i <= LUT_SIZE; i++) {
        float x = (float)i / (float)LUT_SIZE;
        lutExpUp[i] = 1.0f - expf(-c * x);
    }
}

// Linear interpolation in the LUT
inline float interpolateLUT(float x)
{
    if (x < 0.f) x = 0.f;
    if (x > 1.f) x = 1.f;
    float fx = x * (float)LUT_SIZE;
    int idx = (int)fx;
    if (idx >= LUT_SIZE) idx = LUT_SIZE - 1;
    float frac = fx - (float)idx;
    float v0 = lutExpUp[idx];
    float v1 = lutExpUp[idx+1];
    return v0 + frac*(v1 - v0);
}

// shapeLeft => linear or exp up
inline float shapeLeftLUT(float ratio, bool useLog)
{
    if (!useLog) {
        return ratio; // linear
    } else {
        // /¯
        return interpolateLUT(ratio);
    }
}

// shapeRight => invert linear or invert exp
inline float shapeRightLUT(float ratio, bool useLog)
{
    if (!useLog) {
        // linear invert => 1 - ratio
        return 1.0f - ratio;
    } else {
        // exp invert => sample the LUT at (1 - ratio)
        return interpolateLUT(1.0f - ratio);
    }
}

/**
 * @brief Compute the final scaled volume for the "left side".
 *
 * @param ratio       The normalized ratio in [0..1]
 * @param offset      E.g. left_fil_f
 * @param range       E.g. (65535 - left_fil_f)
 * @param centerScale Multiplicative factor from center pot
 * @param useLog      True => use exponential shaping, false => linear
 * @return uint16_t final scaled volume
 */
inline uint16_t computeVolumeLeft(float ratio, float offset, float range, float centerScale, bool useLog)
{
    // 1) main shaping
    float shaped = shapeLeftLUT(ratio, useLog);
    float volume_f = offset + shaped * range;
    float volumeScaled_f = volume_f * centerScale;

    // clamp
    if (volumeScaled_f < 0.f)        volumeScaled_f = 0.f;
    else if (volumeScaled_f > UI16_MAX_F) volumeScaled_f = UI16_MAX_F;

    uint16_t finalVol = (uint16_t)(volumeScaled_f);

    return finalVol;
}

/**
 * @brief Similar function for the "right side".
 */
inline uint16_t computeVolumeRight(float ratio, float offset, float range, float centerScale, bool useLog)
{
    float shaped = shapeRightLUT(ratio, useLog);
    float volume_f = offset + shaped * range;

    if (volume_f<0.f) volume_f=0.f;
    if (volume_f>UI16_MAX_F) volume_f=UI16_MAX_F;

    float volumeScaled_f = volume_f * centerScale;
    if (volumeScaled_f<0.f) volumeScaled_f=0.f; 
    if (volumeScaled_f>UI16_MAX_F) volumeScaled_f=UI16_MAX_F;

    uint16_t finalVol = (uint16_t)(volumeScaled_f);

    return finalVol;
}

// ===================== Globals for main loop =====================
// (Your existing variables)
static const float ALPHA_POTS = 0.06f;

float cv_raw_f = 0.0f;
float slider_raw_f=0.0f, slider_fil_f=0.0f; bool init_slider=false;
float center_raw_f=0.0f, center_fil_f=0.0f; bool init_center=false;
float left_raw_f  =0.0f, left_fil_f  =0.0f; bool init_left=false;
float right_raw_f =0.0f, right_fil_f =0.0f; bool init_right=false;

uint16_t volume=0, volume_left=0, volume_right=0;

float center_from_slider_f=0.0f;
float left_slide_point_f  =0.0f;
float right_slide_point_f =0.0f;

volatile uint32_t refresh=0, old_refresh=0;

// Mbed objects
AnalogIn  cv_input(A6);
AnalogIn  slider_input(A2);
AnalogIn  center_input(D3);
AnalogIn  left_input(A0);
AnalogIn  right_input(A1);

AnalogOut filtered_output(PA_4);

DigitalIn but_r_lin_log(PB_5);
DigitalIn but_l_lin_log(PB_4);

Timer t;

// ===================== Debug function =====================

static void big_console_debug()
{
    float cv_volt = (cv_raw_f / UI16_MAX_F) * VREF;

    printf(
        "CV=%.1f/%.2fV vol=%u L=%u/%f R=%u/%f Hz=%lu\n",
        cv_raw_f, cv_volt,
        volume,
        volume_left,
        (float)volume_left/UI16_MAX_F,
        volume_right,
        (float)volume_right/UI16_MAX_F,
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
        // 1) ADC reads
        cv_raw_f      = cv_input.read()     * UI16_MAX_F;
        slider_raw_f  = slider_input.read() * UI16_MAX_F;
        center_raw_f  = center_input.read() * UI16_MAX_F;
        left_raw_f    = left_input.read()   * UI16_MAX_F;
        right_raw_f   = right_input.read()  * UI16_MAX_F;

        // 2) Filter pots
        slider_fil_f = ewma_filter_float(slider_raw_f, slider_fil_f, ALPHA_POTS, init_slider);
        center_fil_f = ewma_filter_float(center_raw_f, center_fil_f, ALPHA_POTS, init_center);
        left_fil_f   = ewma_filter_float(left_raw_f,   left_fil_f,   ALPHA_POTS, init_left);
        right_fil_f  = ewma_filter_float(right_raw_f,  right_fil_f,  ALPHA_POTS, init_right);

        // 3) Plateau
        float fraction_slider = slider_fil_f / UI16_MAX_F;
        center_from_slider_f  = center_width_float + fraction_slider*slider_length_minus_center_float;

        left_slide_point_f  = center_from_slider_f - center_width_float;
        right_slide_point_f = center_from_slider_f + center_width_float;

        float center_scale = center_fil_f / UI16_MAX_F;

        // 4) Decide shaping (button pressed => ?). Up to you which way:
        //    If read()==0 => log? or read()==1 => log? 
        //    For example:
        bool useLogLeft  = (but_l_lin_log.read() == 1);
        bool useLogRight = (but_r_lin_log.read() == 1);

        // 5) Plateau logic
        if (cv_raw_f < left_slide_point_f) {
            float ratio_l = 0.f;
            if (left_slide_point_f>0.f) {
                ratio_l = cv_raw_f / left_slide_point_f;
            }
            if (ratio_l<0.f) ratio_l=0.f; 
            if (ratio_l>1.f) ratio_l=1.f;

            float offset_l = left_fil_f;
            float range_l  = (UI16_MAX_F - left_fil_f);
            volume_left    = computeVolumeLeft(ratio_l, offset_l, range_l, center_scale, useLogLeft);
            volume         = volume_left;
            volume_right   = 0;

        } else if (cv_raw_f > right_slide_point_f) {
            float diff_cv = cv_raw_f - right_slide_point_f;
            float denom_r = (UI16_MAX_F - right_slide_point_f);
            float ratio_r = (denom_r>0.f) ? (diff_cv/denom_r) : 0.f;
            if (ratio_r<0.f) ratio_r=0.f;
            if (ratio_r>1.f) ratio_r=1.f;

            float offset_r = right_fil_f;
            float range_r  = (UI16_MAX_F - right_fil_f);
            volume_right   = computeVolumeRight(ratio_r, offset_r, range_r, center_scale, useLogRight);
            volume         = volume_right;
            volume_left    = 0;

        } else {
            volume      = (uint16_t)center_fil_f;
            volume_left = 0;
            volume_right= 0;
        }

        // 6) DAC
        filtered_output.write_u16(volume);

#if DEBUG_LINLOG
        // 7) refresh
        refresh++;

        // 8) Timer checks
        uint32_t nowMs = t.read_ms();
        if ((nowMs - lastRefMs) >= 1000) {
            lastRefMs = nowMs;
            old_refresh = refresh;
            refresh = 0;
        }
        if ((nowMs - lastDbgMs) >= 1000) {
            lastDbgMs = nowMs;
            big_console_debug();
        }
#endif
    }
}
