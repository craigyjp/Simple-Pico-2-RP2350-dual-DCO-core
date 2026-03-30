/*
 * dco_engine.h
 *
 * Standalone DCO engine for RP2350 stamp
 * Provides: Multi-saw, PWM/pulse, sub oscillator, LFO
 *
 * No external library dependencies - float throughout
 * Sample rate: 48000 Hz
 */

#ifndef DCO_ENGINE_H_
#define DCO_ENGINE_H_

#include <stdint.h>

/* --------------------------------------------------------
 * Configuration
 * -------------------------------------------------------- */
#define DCO_SAMPLE_RATE     48000
#define DCO_MAX_SAW_VOICES  5
#define DCO_BUFFER_SIZE     48

/* LFO waveform constants */
#define LFO_TRIANGLE    0
#define LFO_SQUARE      1
#define LFO_SAWTOOTH    2

/* --------------------------------------------------------
 * Initialisation
 * -------------------------------------------------------- */
void DCO_Init(float sample_rate);

/* --------------------------------------------------------
 * Note control
 * -------------------------------------------------------- */
void DCO_NoteOn(uint8_t note, uint8_t vel);
void DCO_NoteOff(uint8_t note);

/* --------------------------------------------------------
 * Pitch
 * -------------------------------------------------------- */
void DCO_PitchBend(uint16_t bend);          /* 0-16383, centre=8192 */
void DCO_SetPitchBendRange(uint8_t semitones);

/* --------------------------------------------------------
 * Portamento
 * -------------------------------------------------------- */
void DCO_SetPortamento(uint8_t value);      /* CC65: >=64 on, <64 off */
void DCO_SetPortamentoRate(uint8_t value);  /* CC5: 0=fast, 127=slow */

/* --------------------------------------------------------
 * Multi-saw parameters
 * -------------------------------------------------------- */
void DCO_SetSawCount(uint8_t value);        /* 0-127 -> 1-5 voices */
void DCO_SetSawDetune(uint8_t value);       /* 0-127 -> 0-100 cents spread */
void DCO_SetSawLevel(uint8_t value);        /* 0-127 */

/* --------------------------------------------------------
 * Pulse / PWM oscillator
 * -------------------------------------------------------- */
void DCO_SetPulseWidth(uint8_t value);      /* 0-127 -> 1%-99% */
void DCO_SetPWMDepth(uint8_t value);        /* 0-127, mod wheel PWM scaling */
void DCO_SetPulseLevel(uint8_t value);      /* 0-127 */

/* --------------------------------------------------------
 * Sub oscillator
 * -------------------------------------------------------- */
void DCO_SetSubLevel(uint8_t value);        /* 0-127, one octave down square */

/* --------------------------------------------------------
 * MIDI Modulation
 * -------------------------------------------------------- */
void DCO_SetModWheel(uint8_t value);        /* 0-127, pressure value */
void DCO_SetModWheelFMDepth(uint8_t value);  /* 0-127, mod wheel -> FM depth */
void DCO_SetAftertouch(uint8_t value);      /* 0-127, pressure value */
void DCO_SetAftertouchFMDepth(uint8_t value); /* 0-127, aftertouch -> FM depth */

/* --------------------------------------------------------
 * ADC modulation inputs
 * Call from ADC timer (~1kHz) with raw 12-bit values
 * Both centred at 1.65V = ADC ~2048 = no modulation
 * -------------------------------------------------------- */
void DCO_SetADC_FM(uint16_t raw);           /* GPIO26 ADC0 - FM input */
void DCO_SetADC_PWM(uint16_t raw);          /* GPIO27 ADC1 - PWM input */
void DCO_SetFMDepth(uint8_t value);         /* 0-127, scales ADC FM input */
void DCO_SetADCPWMDepth(uint8_t value);     /* 0-127, scales ADC PWM input */

/* --------------------------------------------------------
 * X-MOD - DCO2 output -> DCO1 frequency only
 * Feed DCO2 PWM output (GPIO4) via RC filter into GPIO28 (ADC2)
 * Centred at 1.65V = no modulation
 * -------------------------------------------------------- */
void DCO_SetADC_XMod(uint16_t raw);         /* GPIO28 ADC2 - X-MOD input */
void DCO_SetXModDepth(uint8_t value);       /* 0-127, scales X-MOD depth */
void DCO_CalibrateXMod(uint16_t raw);       /* call at startup to set zero point */

/* --------------------------------------------------------
 * LFO
 * -------------------------------------------------------- */
void DCO_SetLFORate(uint8_t value);         /* 0-127 -> 0.1Hz-20Hz exponential */
void DCO_SetLFOWaveform(uint8_t value);     /* 0-42=triangle 43-84=square 85-127=sawtooth */
void DCO_SetLFOFMDepth(uint8_t value);      /* 0-127, LFO1 -> FM pitch depth */
void DCO_SetLFOPWMDepth(uint8_t value);     /* 0-127, kept for compatibility */

/* --------------------------------------------------------
 * LFO2 - dedicated to PWM modulation
 * -------------------------------------------------------- */
void DCO_SetLFO2Rate(uint8_t value);        /* 0-127 -> 0.1Hz-20Hz exponential */
void DCO_SetLFO2Waveform(uint8_t value);    /* 0-42=triangle 43-84=square 85-127=sawtooth */
void DCO_SetLFO2PWMDepth(uint8_t value);    /* 0-127, LFO2 -> DCO1 PWM depth */
void DCO_SetLFO2DCO2PWMDepth(uint8_t value); /* 0-127, LFO2 -> DCO2 PWM depth */

/* --------------------------------------------------------
 * Oscillator sync
 * -------------------------------------------------------- */
void DCO_SetSyncMode(uint8_t value);        /* 0-42=off, 43-84=soft, 85-127=hard */

/* --------------------------------------------------------
 * DCO2 sweep envelope (ADSR)
 * Triggered by NoteOn/NoteOff, sweeps DCO2 pitch upward
 * -------------------------------------------------------- */
void DCO_SetEnvAttack(uint8_t value);       /* 0-127, 0=slow 127=fast */
void DCO_SetEnvDecay(uint8_t value);        /* 0-127, 0=slow 127=fast */
void DCO_SetEnvSustain(uint8_t value);      /* 0-127, sustain level */
void DCO_SetEnvRelease(uint8_t value);      /* 0-127, 0=slow 127=fast */
void DCO_SetEnvSweepDepth(uint8_t value);    /* 0-127, envelope -> DCO2 pitch depth */
void DCO_SetEnvDCO1PWMDepth(uint8_t value);  /* 0-127, envelope -> DCO1 PWM depth */
void DCO_SetEnvDCO2PWMDepth(uint8_t value);  /* 0-127, envelope -> DCO2 PWM depth */

/* --------------------------------------------------------
 * Audio processing - DCO1
 * -------------------------------------------------------- */
void DCO_Process(float *output, int len);   /* output range -1.0 to +1.0 */
/* --------------------------------------------------------
 * DCO2 - independent pulse/sub at offset pitch
 * Shares note, FM modulation and LFO with DCO1
 * DCO2_Process() must be called after DCO_Process() each block
 * -------------------------------------------------------- */
void DCO2_SetSawLevel(uint8_t value);       /* 0-127, single sawtooth */
void DCO2_SetPulseWidth(uint8_t value);     /* 0-127 -> 1%-99% */
void DCO2_SetPulseLevel(uint8_t value);     /* 0-127 */
void DCO2_SetPWMDepth(uint8_t value);       /* 0-127, mod wheel PWM depth */
void DCO2_SetADCPWMDepth(uint8_t value);    /* 0-127, ADC PWM input depth */
void DCO2_SetSubLevel(uint8_t value);       /* 0-127 */
void DCO2_SetDetune(uint8_t value);         /* 0-127, centre=64 -> 0 cents, range ±100 */
void DCO2_SetInterval(uint8_t value);       /* 0-127, centre=64 -> 0 semitones, range ±24 */
void DCO2_Process(float *output, int len);  /* output range -1.0 to +1.0 */

/* Use this instead of DCO_Process + DCO2_Process when sync is active */
void DCO_ProcessBoth(float *out1, float *out2, int len);

#endif /* DCO_ENGINE_H_ */
