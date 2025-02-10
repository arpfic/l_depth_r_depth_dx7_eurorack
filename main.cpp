/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

// NUCLEO 32bit :
// REMOVE SB16 and SB18 solder bridges in order to use PA_5 (A4) and PA_6 (A5)

#include "AnalogOut.h"
#include "Thread.h"
#include "mbed.h"
#include "SoftPWM.h"
#include "EwmaT.h"
#include <cstdint>
#include <iterator>

#define UI16_MAX                    65535
#define BLINKING_RATE               5ms
#define CONSOLE_RATE                1000ms
#define FILTER_CV_WEIGHT            1 // [0, 100] Higher the value - less smoothing (higher the latest reading impact)
#define FILTER_POTS_WEIGHT          3

// Pente de profondeur selon l'entrée CV
// Largeur du plateau, en %
#define CENTER_WIDTH                0.2
#define CENTER_WIDTH_UI16           (uint16_t)(CENTER_WIDTH * UI16_MAX) / 2
#define SLIDER_LENGTH_MINUS_CENTER  (uint16_t)(UI16_MAX - (CENTER_WIDTH_UI16 * 2))
#define LEFT_SILDER_ADJ             50 // Pour ajuster le point où le plateau recouvre tout à gauche (dépend des valeurs absolues)
#define RIGHT_SILDER_ADJ            50
#define SCALE_SLIDER                ((float)(SLIDER_LENGTH_MINUS_CENTER) / (float)(UI16_MAX))


//SoftPWM                             led(LED1);  // TO COMMENT
AnalogIn                            cv_input(A6); // CV input
AnalogIn                            slider_input(A2); // SLIDER input
AnalogIn                            center_input(D3); // POT CENTER input
AnalogIn                            left_input(A0); // POT Left input
AnalogIn                            right_input(A1); // POT Right input
//AnalogOut                         raw_output(PA_4); // DAC 1  // TO COMMENT
AnalogOut                           filtered_output(PA_4); // DAC 2

DigitalIn                           but_r_lin_log(PB_5); // Lin/Log algo to R depth
DigitalIn                           but_l_lin_log(PB_4); // Lin/Log algo to L depth

// Exponentially Weighted Moving Average filter
// https://github.com/jonnieZG/EWMA
EwmaT <int>                         ewma_cv(FILTER_CV_WEIGHT, 100);
EwmaT <int>                         ewma_slider(FILTER_POTS_WEIGHT, 100);
EwmaT <int>                         ewma_center(FILTER_POTS_WEIGHT, 100);
EwmaT <int>                         ewma_left(FILTER_POTS_WEIGHT, 100);
EwmaT <int>                         ewma_right(FILTER_POTS_WEIGHT, 100);

Thread                              threadRefresh;
Thread                              threadLed;
Thread                              threadConsole;

uint16_t                            raw_cv_input, raw_slider_input, raw_center_input, raw_left_input, raw_right_input;
volatile uint32_t                   refresh,old_refresh;
uint32_t                            filtered_raw_cv_input, filtered_raw_slider_input, filtered_raw_center_input, filtered_raw_left_input, filtered_raw_right_input;
uint16_t                            center_from_slider;
uint16_t                            volume, volume_right, volume_left;
uint16_t                            superdebug;
uint16_t                            left_slide_point, right_slide_point;

float                               left_cv_calc, right_cv_calc;

void refresh_thread(void)
{
    while (true) {
        old_refresh = refresh;
        refresh = 0;
        ThisThread::sleep_for(1000ms);
    }
}

void led_thread(void)
{
    while (true) {
        //led.write(((float)volume)/(float)UI16_MAX);  // TO COMMENT
        ThisThread::sleep_for(BLINKING_RATE);
    }
}

void console_thread(void)
{
    while (true) {
        printf("%iHz | %f\%\n",
        old_refresh,
        filtered_output.read()
        );

        ThisThread::sleep_for(CONSOLE_RATE);
    }
}

void big_console_thread(void)
{
    while (true) {
        printf("CV INPUT: 0x%04X, %05i/UI16_MAX, %fV | OUTPUT: %f | SLIDER: %f/%f, CENTER:%i/%i| L%d-R%d | CENTER: %f/%f | LEFT: %f/%f | RIGHT: %f/%f | %iHz | %d | %d | %f * CV + %d = %d | %f * (CV - %d) + %d = %d\n",
        raw_cv_input,
        raw_cv_input,
        3.3*((float)raw_cv_input)/((float)UI16_MAX),
        //raw_output.read(),
        filtered_output.read(),
        slider_input.read(),
        (float)filtered_raw_slider_input / (float)UI16_MAX,
        center_from_slider,
        (uint16_t)CENTER_WIDTH_UI16,
        but_l_lin_log.read(),
        but_r_lin_log.read(),
        center_input.read(),
        (float)filtered_raw_center_input / (float)UI16_MAX,
        left_input.read(),
        (float)filtered_raw_left_input / (float)UI16_MAX,
        right_input.read(),
        (float)filtered_raw_right_input / (float)UI16_MAX,
        old_refresh,
        volume,
        superdebug,
        left_cv_calc,
        filtered_raw_left_input,
        volume_left,
        right_cv_calc,
        right_slide_point,
        filtered_raw_right_input,
        volume_right
        );

        ThisThread::sleep_for(CONSOLE_RATE);
    }
}


int main()
{
    old_refresh = 0;
    raw_cv_input = 0;
    filtered_raw_cv_input = 0;
    filtered_raw_slider_input = 0;
    filtered_raw_left_input = 0;
    filtered_raw_right_input = 0;
    superdebug = 0;
    volume = 0;
    volume_left = 0;
    volume_right = 0;
    //printf("-- START --");

    but_r_lin_log.mode(PullUp);
    but_l_lin_log.mode(PullUp);

    //led.period_ms(10); // TO COMMENT

    threadRefresh.start(refresh_thread);
    //threadLed.start(led_thread);
    threadConsole.start(big_console_thread);  // TO COMMENT
    //threadConsole.start(console_thread);  // TO COMMENT

    while (true) {
        // check inputs
        raw_cv_input = cv_input.read_u16();
        raw_slider_input = slider_input.read_u16();
        raw_center_input = center_input.read_u16();
        raw_left_input = left_input.read_u16();
        raw_right_input = right_input.read_u16();

        filtered_raw_slider_input = ewma_slider.filter(raw_slider_input);
        filtered_raw_center_input = ewma_center.filter(raw_center_input);
        filtered_raw_left_input = ewma_left.filter(raw_left_input);
        filtered_raw_right_input = ewma_right.filter(raw_right_input);

        filtered_raw_cv_input = ewma_cv.filter(raw_cv_input); // 2nd DAC, filtered version
        filtered_output.write_u16(volume);

        center_from_slider = CENTER_WIDTH_UI16 + (uint16_t)(filtered_raw_slider_input * SCALE_SLIDER);
        left_slide_point = center_from_slider - CENTER_WIDTH_UI16;
        right_slide_point = center_from_slider + CENTER_WIDTH_UI16;

        if (filtered_raw_cv_input < ((center_from_slider - CENTER_WIDTH_UI16))) {
            superdebug = 1;
            left_cv_calc = ((float)(UI16_MAX - filtered_raw_left_input)) / ((float)(left_slide_point));
            volume_left = ((uint16_t)(left_cv_calc * (float)filtered_raw_cv_input)) + filtered_raw_left_input;
            volume = (uint16_t)((float)volume_left * ((float)filtered_raw_center_input / (float)UI16_MAX));
            volume_right = 0;
        } else if (filtered_raw_cv_input > ((center_from_slider + CENTER_WIDTH_UI16))) {
            superdebug = 2;
            right_cv_calc = -((float)(0 - (filtered_raw_right_input - UI16_MAX)) / (float)(UI16_MAX - (right_slide_point)));
            volume_right = ((uint16_t)(right_cv_calc * (float)(filtered_raw_cv_input - right_slide_point))) + UI16_MAX;
            volume = (uint16_t)((float)volume_right * ((float)filtered_raw_center_input / (float)UI16_MAX));
            volume_left = 0;
        } else {
            volume = filtered_raw_center_input;
            volume_left = 0;
            volume_right = 0;
            superdebug = 0;
        }
        refresh++;
    }
}
