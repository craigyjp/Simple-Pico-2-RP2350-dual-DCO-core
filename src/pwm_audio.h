/*
 * pwm_audio.h
 *
 * Dual PWM audio output for RP2350 using DMA
 * DCO1: GPIO2, DCO2: GPIO4
 */

#ifndef PWM_AUDIO_H_
#define PWM_AUDIO_H_

#include <stdint.h>
#include <Arduino.h>

/* --------------------------------------------------------
 * Callbacks - implement in main application
 * Called from main loop to fill next audio buffer
 * output: float buffer -1.0 to +1.0, len: number of samples
 * -------------------------------------------------------- */
void PWM_CB_FillBuffer_DCO1(float *output, int len);
void PWM_CB_FillBuffer_DCO2(float *output, int len);

/* --------------------------------------------------------
 * API
 * -------------------------------------------------------- */
void PWMAudio_Init(uint8_t pin1, uint8_t pin2, uint32_t sample_rate);
void PWMAudio_Process(void);

#endif /* PWM_AUDIO_H_ */
