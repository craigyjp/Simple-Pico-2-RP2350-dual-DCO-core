/*
 * dco_engine.cpp
 *
 * Standalone DCO engine for RP2350 stamp
 *
 * Oscillator architecture:
 *   - Multi-saw: up to 5 detuned bandlimited sawtooth voices
 *   - Pulse/PWM: variable width square wave with modulatable PW
 *   - Sub: one octave down square wave
 *   - LFO: sine/triangle/square/sawtooth, routes to FM and/or PWM
 *
 * All oscillators run as phase accumulators.
 * Bandlimiting via PolyBLEP to reduce aliasing.
 */

#include "dco_engine.h"
#include <math.h>
#include <string.h>

/* --------------------------------------------------------
 * Constants
 * -------------------------------------------------------- */
#define TWO_PI          (2.0f * 3.14159265358979f)
#define MIDI_NOTE_A4    69
#define FREQ_A4         440.0f

/* LFO rate range */
#define LFO_RATE_MIN    0.1f    /* Hz */
#define LFO_RATE_MAX    20.0f   /* Hz */

/* FM pitch modulation range at full depth - semitones */
#define FM_SEMITONE_RANGE   12.0f

/* --------------------------------------------------------
 * PolyBLEP helper
 * -------------------------------------------------------- */
static float polyblep(float t, float dt)
{
    if (t < dt)
    {
        t /= dt;
        return t + t - t * t - 1.0f;
    }
    else if (t > 1.0f - dt)
    {
        t = (t - 1.0f) / dt;
        return t * t + t + t + 1.0f;
    }
    return 0.0f;
}

/* --------------------------------------------------------
 * Convert MIDI note to frequency
 * -------------------------------------------------------- */
static float noteToFreq(uint8_t note)
{
    return FREQ_A4 * powf(2.0f, (float)(note - MIDI_NOTE_A4) / 12.0f);
}

/* --------------------------------------------------------
 * Internal state
 * -------------------------------------------------------- */

/* --- Multi-saw --- */
typedef struct
{
    float phase;
    float phaseInc;
    float detuneRatio;
} SawVoice;

static SawVoice sawVoices[DCO_MAX_SAW_VOICES];
static int      sawCount    = 1;
static float    sawDetune   = 0.0f;     /* cents */
static float    sawLevel    = 1.0f;

/* --- Pulse / PWM --- */
static float    pulsePhase      = 0.0f;
static float    pulsePhaseInc   = 0.0f;
static float    pulseWidth      = 0.5f;
static float    pwmDepth        = 0.0f;
static float    pulseLevel      = 0.0f;

/* --- Sub oscillator --- */
static float    subPhase        = 0.0f;
static float    subPhaseInc     = 0.0f;
static float    subLevel        = 0.0f;

/* --- DCO2 Pulse / PWM --- */
static float    dco2PulsePhase  = 0.0f;
static float    dco2PulseInc    = 0.0f;
static float    dco2PulseWidth  = 0.5f;
static float    dco2PulseLevel  = 0.0f;
static float    dco2PWMDepth    = 0.0f;   /* mod wheel PWM depth */
static float    dco2LFOPWMDepth = 0.0f;   /* LFO PWM depth */
static float    dco2ADCPWMDepth = 0.0f;   /* ADC PWM depth */

/* --- DCO2 Sub --- */
static float    dco2SubPhase    = 0.0f;
static float    dco2SubInc      = 0.0f;
static float    dco2SubLevel    = 0.0f;

/* --- DCO2 Pitch offset --- */
static float    dco2Detune      = 0.0f;   /* cents, -100 to +100 */
static int8_t   dco2Interval    = 0;      /* semitones, -24 to +24 */
static float    dco2PitchRatio  = 1.0f;   /* combined detune+interval ratio */

/* --- Pitch --- */
static float    sampleRate      = 48000.0f;
static float    baseFreq        = 0.0f;
static float    bendRatio       = 1.0f;
static uint8_t  bendRange       = 2;
static uint8_t  currentNote     = 255;
static bool     noteActive      = false;

/* --- Portamento --- */
static bool     portoEnabled    = false;    /* on/off */
static float    portoRate       = 0.0f;     /* octaves per sample */
static float    currentFreq     = 0.0f;     /* current (slewed) frequency */
static float    targetFreq      = 0.0f;     /* target frequency from NoteOn */

/* --- MIDI Modulation --- */
static float    modWheel        = 0.0f;     /* 0.0 - 1.0 */
static float    aftertouch      = 0.0f;     /* 0.0 - 1.0 */

/* --- ADC Modulation --- */
static float    adcFM           = 0.0f;     /* -1.0 to +1.0 */
static float    adcPWM          = 0.0f;     /* -1.0 to +1.0 */
static float    fmDepth         = 0.0f;     /* 0.0 - 1.0, scales ADC FM input */
static float    adcPWMDepth     = 0.0f;     /* 0.0 - 1.0, scales ADC PWM input */

/* --- LFO --- */
static float    lfoPhase        = 0.0f;     /* 0.0 - 1.0 */
static float    lfoPhaseInc     = 0.0f;     /* per sample */
static uint8_t  lfoWaveform     = LFO_TRIANGLE;
static float    lfoFMDepth      = 0.0f;     /* 0.0 - 1.0 */
static float    lfoPWMDepth     = 0.0f;     /* 0.0 - 1.0 */
static float    lfoOutput       = 0.0f;     /* current LFO value -1.0 to +1.0 */

/* --------------------------------------------------------
 * Internal: generate LFO sample and advance phase
 * -------------------------------------------------------- */
static float lfoTick(void)
{
    float out = 0.0f;

    switch (lfoWaveform)
    {
        case LFO_TRIANGLE:
            /* 0-0.5: ramp up -1 to +1, 0.5-1.0: ramp down +1 to -1 */
            if (lfoPhase < 0.5f)
                out = 4.0f * lfoPhase - 1.0f;
            else
                out = 3.0f - 4.0f * lfoPhase;
            break;

        case LFO_SQUARE:
            out = (lfoPhase < 0.5f) ? 1.0f : -1.0f;
            break;

        case LFO_SAWTOOTH:
            out = 2.0f * lfoPhase - 1.0f;
            break;

        default:
            out = 0.0f;
            break;
    }

    lfoPhase += lfoPhaseInc;
    if (lfoPhase >= 1.0f)
        lfoPhase -= 1.0f;

    return out;
}

/* --------------------------------------------------------
 * Internal: recalculate DCO2 pitch ratio from detune and interval
 * -------------------------------------------------------- */
static void recalcDCO2PitchRatio(void)
{
    float semitones = (float)dco2Interval + dco2Detune / 100.0f;
    dco2PitchRatio  = powf(2.0f, semitones / 12.0f);
}

/* --------------------------------------------------------
 * Internal: recalculate all phase increments
 * -------------------------------------------------------- */
static void recalcPhaseIncs(void)
{
    float freq = currentFreq * bendRatio;

    pulsePhaseInc = freq / sampleRate;
    subPhaseInc   = (freq * 0.5f) / sampleRate;

    float freq2      = freq * dco2PitchRatio;
    dco2PulseInc     = freq2 / sampleRate;
    dco2SubInc       = (freq2 * 0.5f) / sampleRate;

    if (sawCount == 1)
    {
        sawVoices[0].detuneRatio = 1.0f;
        sawVoices[0].phaseInc    = freq / sampleRate;
    }
    else
    {
        float minDetune       = 2.0f * (float)(sawCount - 1);
        float effectiveDetune = (sawDetune > minDetune) ? sawDetune : minDetune;

        for (int i = 0; i < sawCount; i++)
        {
            float position   = (float)i / (float)(sawCount - 1);
            float centOffset = (position - 0.5f) * 2.0f * effectiveDetune;
            float ratio      = powf(2.0f, centOffset / 1200.0f);
            sawVoices[i].detuneRatio = ratio;
            sawVoices[i].phaseInc    = (freq * ratio) / sampleRate;
        }
    }
}

/* --------------------------------------------------------
 * Initialisation
 * -------------------------------------------------------- */
void DCO_Init(float sample_rate)
{
    sampleRate = sample_rate;

    memset(sawVoices, 0, sizeof(sawVoices));

    sawCount  = 1;
    sawDetune = 0.0f;
    sawLevel  = 0.8f;

    pulsePhase    = 0.0f;
    pulsePhaseInc = 0.0f;
    pulseWidth    = 0.5f;
    pwmDepth      = 0.0f;
    pulseLevel    = 0.0f;

    subPhase    = 0.0f;
    subPhaseInc = 0.0f;
    subLevel    = 0.0f;

    dco2PulsePhase  = 0.0f;
    dco2PulseInc    = 0.0f;
    dco2PulseWidth  = 0.5f;
    dco2PulseLevel  = 0.0f;
    dco2PWMDepth    = 0.0f;
    dco2LFOPWMDepth = 0.0f;
    dco2ADCPWMDepth = 0.0f;
    dco2SubPhase   = 0.0f;
    dco2SubInc     = 0.0f;
    dco2SubLevel   = 0.0f;
    dco2Detune     = 0.0f;
    dco2Interval   = 0;
    dco2PitchRatio = 1.0f;

    baseFreq     = 0.0f;
    currentFreq  = 0.0f;
    targetFreq   = 0.0f;
    bendRatio    = 1.0f;
    portoEnabled = false;
    portoRate    = 0.0f;
    bendRange   = 2;
    noteActive  = false;
    currentNote = 255;

    modWheel    = 0.0f;
    aftertouch  = 0.0f;
    adcFM       = 0.0f;
    adcPWM      = 0.0f;
    fmDepth     = 0.0f;

    lfoPhase    = 0.0f;
    lfoPhaseInc = LFO_RATE_MIN / sample_rate;
    lfoWaveform = LFO_TRIANGLE;
    lfoFMDepth  = 0.0f;
    lfoPWMDepth = 0.0f;
    lfoOutput   = 0.0f;
}

/* --------------------------------------------------------
 * Note control
 * -------------------------------------------------------- */
void DCO_NoteOn(uint8_t note, uint8_t vel)
{
    (void)vel;          /* velocity unused - DCO runs at constant level */
    currentNote = note;
    noteActive  = true;
    targetFreq  = noteToFreq(note);
    baseFreq    = targetFreq;
    if (!portoEnabled)
        currentFreq = targetFreq;   /* snap immediately if portamento off */
    else if (currentFreq < 1.0f)
        currentFreq = targetFreq;   /* snap on first note */

    for (int i = 0; i < DCO_MAX_SAW_VOICES; i++)
        sawVoices[i].phase = (float)i / (float)DCO_MAX_SAW_VOICES;

    pulsePhase     = 0.0f;
    subPhase       = 0.0f;
    dco2PulsePhase = 0.0f;
    dco2SubPhase   = 0.0f;

    recalcPhaseIncs();
}

void DCO_NoteOff(uint8_t note)
{
    if (note == currentNote)
        noteActive = false;
}

/* --------------------------------------------------------
 * Pitch bend
 * -------------------------------------------------------- */
void DCO_PitchBend(uint16_t bend)
{
    float normalised = ((float)bend - 8192.0f) / 8192.0f;
    float semitones  = normalised * (float)bendRange;
    bendRatio = powf(2.0f, semitones / 12.0f);
    recalcPhaseIncs();
}

void DCO_SetPitchBendRange(uint8_t semitones)
{
    if (semitones < 1)  semitones = 1;
    if (semitones > 12) semitones = 12;
    bendRange = semitones;
}

/* --------------------------------------------------------
 * Multi-saw parameters
 * -------------------------------------------------------- */
void DCO_SetSawCount(uint8_t value)
{
    int count = 1 + (int)((float)value / 127.0f * (DCO_MAX_SAW_VOICES - 1) + 0.5f);
    if (count < 1)               count = 1;
    if (count > DCO_MAX_SAW_VOICES) count = DCO_MAX_SAW_VOICES;
    sawCount = count;
    recalcPhaseIncs();
}

void DCO_SetSawDetune(uint8_t value)
{
    sawDetune = (float)value / 127.0f * 100.0f;
    recalcPhaseIncs();
}

void DCO_SetSawLevel(uint8_t value)
{
    sawLevel = (float)value / 127.0f;
}

/* --------------------------------------------------------
 * Pulse / PWM parameters
 * -------------------------------------------------------- */
void DCO_SetPulseWidth(uint8_t value)
{
    pulseWidth = 0.01f + (float)value / 127.0f * 0.98f;
}

void DCO_SetPWMDepth(uint8_t value)
{
    pwmDepth = (float)value / 127.0f;
}

void DCO_SetPulseLevel(uint8_t value)
{
    pulseLevel = (float)value / 127.0f;
}

/* --------------------------------------------------------
 * Sub oscillator
 * -------------------------------------------------------- */
void DCO_SetSubLevel(uint8_t value)
{
    subLevel = (float)value / 127.0f;
}

/* --------------------------------------------------------
 * MIDI Modulation
 * -------------------------------------------------------- */
void DCO_SetModWheel(uint8_t value)
{
    modWheel = (float)value / 127.0f;
}

void DCO_SetAftertouch(uint8_t value)
{
    aftertouch = (float)value / 127.0f;
}

/* --------------------------------------------------------
 * ADC modulation inputs
 * -------------------------------------------------------- */
void DCO_SetADC_FM(uint16_t raw)
{
    adcFM = ((float)raw - 2048.0f) / 2048.0f;
}

void DCO_SetADC_PWM(uint16_t raw)
{
    adcPWM = ((float)raw - 2048.0f) / 2048.0f;
}

void DCO_SetFMDepth(uint8_t value)
{
    fmDepth = (float)value / 127.0f;
}

void DCO_SetADCPWMDepth(uint8_t value)
{
    adcPWMDepth = (float)value / 127.0f;
}

/* --------------------------------------------------------
 * LFO parameters
 * -------------------------------------------------------- */
void DCO_SetLFORate(uint8_t value)
{
    /* map 0-127 exponentially to LFO_RATE_MIN - LFO_RATE_MAX Hz
     * exponential gives finer control at low rates */
    float t    = (float)value / 127.0f;
    float rate = LFO_RATE_MIN * powf(LFO_RATE_MAX / LFO_RATE_MIN, t);
    lfoPhaseInc = rate / sampleRate;
}

void DCO_SetLFOWaveform(uint8_t value)
{
    /* 0-42=triangle, 43-84=square, 85-127=sawtooth
     * divides CC range into 3 equal bands */
    if      (value < 43)  lfoWaveform = LFO_TRIANGLE;
    else if (value < 85)  lfoWaveform = LFO_SQUARE;
    else                  lfoWaveform = LFO_SAWTOOTH;
}

void DCO_SetLFOFMDepth(uint8_t value)
{
    lfoFMDepth = (float)value / 127.0f;
}

void DCO_SetLFOPWMDepth(uint8_t value)
{
    lfoPWMDepth = (float)value / 127.0f;
}

/* --------------------------------------------------------
 * Portamento
 * -------------------------------------------------------- */
void DCO_SetPortamento(uint8_t value)
{
    /* CC65 convention: value >= 64 = on, < 64 = off */
    portoEnabled = (value >= 64);
    if (!portoEnabled)
        currentFreq = targetFreq;   /* snap to target when switched off */
}

void DCO_SetPortamentoRate(uint8_t value)
{
    /* CC5: 0 = instant (very fast), 127 = very slow
     * map to octaves per sample: fast = 0.01 oct/sample, slow = 0.00005 */
    if (value == 0)
    {
        portoRate = 1.0f;   /* effectively instant */
        return;
    }
    float t   = (float)value / 127.0f;
    portoRate = 0.01f * powf(0.005f, t);   /* exponential: 0.01 -> 0.00005 */
}

/* --------------------------------------------------------
 * DCO2 parameters
 * -------------------------------------------------------- */
void DCO2_SetPulseWidth(uint8_t value)
{
    dco2PulseWidth = 0.01f + (float)value / 127.0f * 0.98f;
}

void DCO2_SetPulseLevel(uint8_t value)
{
    dco2PulseLevel = (float)value / 127.0f;
}

void DCO2_SetSubLevel(uint8_t value)
{
    dco2SubLevel = (float)value / 127.0f;
}

void DCO2_SetPWMDepth(uint8_t value)
{
    dco2PWMDepth = (float)value / 127.0f;
}

void DCO2_SetLFOPWMDepth(uint8_t value)
{
    dco2LFOPWMDepth = (float)value / 127.0f;
}

void DCO2_SetADCPWMDepth(uint8_t value)
{
    dco2ADCPWMDepth = (float)value / 127.0f;
}

void DCO2_SetDetune(uint8_t value)
{
    /* centre=64 -> 0 cents, 0 -> -100 cents, 127 -> +100 cents */
    dco2Detune = ((float)value - 64.0f) / 64.0f * 100.0f;
    recalcDCO2PitchRatio();
    recalcPhaseIncs();
}

void DCO2_SetInterval(uint8_t value)
{
    /* centre=64 -> 0 semitones, range -24 to +24 */
    dco2Interval = (int8_t)(((float)value - 64.0f) / 64.0f * 24.0f);
    recalcDCO2PitchRatio();
    recalcPhaseIncs();
}

/* --------------------------------------------------------
 * Audio processing - DCO1 (saw + pulse + sub)
 * -------------------------------------------------------- */
void DCO_Process(float *output, int len)
{

    /* --- Portamento slew ---
     * currentFreq glides toward targetFreq at portoRate (octaves/sample)
     * Done once per buffer for efficiency, error is inaudible at 256 samples */
    if (portoEnabled && currentFreq != targetFreq)
    {
        float ratio = targetFreq / currentFreq;
        float step  = powf(2.0f, portoRate);   /* max frequency ratio per sample */
        if (ratio > step)
            currentFreq *= step;
        else if (ratio < 1.0f / step)
            currentFreq /= step;
        else
            currentFreq = targetFreq;
        baseFreq = currentFreq;
        recalcPhaseIncs();
    }

    for (int n = 0; n < len; n++)
    {
        /* --- LFO tick (triangle/square/saw - no trig functions) --- */
        lfoOutput = lfoTick();

        /* --- FM modulation ---
         * LFO + ADC (scaled by fmDepth) + mod wheel + aftertouch
         * fmRatio computed once per sample outside inner loops */
        float fmMod = (lfoOutput * lfoFMDepth)
                    + (adcFM * fmDepth)   /* zero if fmDepth=0 */
                    + modWheel
                    + aftertouch;
        if (fmMod >  1.0f) fmMod =  1.0f;
        if (fmMod < -1.0f) fmMod = -1.0f;

        /* ±1.0 maps to ±FM_SEMITONE_RANGE semitones */
        float fmRatio = powf(2.0f, fmMod * FM_SEMITONE_RANGE / 12.0f);

        /* --- PWM modulation --- */
        float pwmMod = (lfoOutput * lfoPWMDepth)
                     + (adcPWM * adcPWMDepth)  /* zero unless depth > 0 */
                     + (modWheel * pwmDepth);
        if (pwmMod >  1.0f) pwmMod =  1.0f;
        if (pwmMod < -1.0f) pwmMod = -1.0f;

        float sample = 0.0f;

        /* --- Multi-saw --- */
        if (sawLevel > 0.0f)
        {
            float sawSample = 0.0f;
            for (int i = 0; i < sawCount; i++)
            {
                float p  = sawVoices[i].phase;
                float dt = sawVoices[i].phaseInc * fmRatio;

                float v = 2.0f * p - 1.0f;
                v -= polyblep(p, dt);
                sawSample += v;

                sawVoices[i].phase += dt;
                if (sawVoices[i].phase >= 1.0f)
                    sawVoices[i].phase -= 1.0f;
            }
            sawSample /= (float)sawCount;
            sample += sawSample * sawLevel;
        }

        /* --- Pulse / PWM ---
         * Single accumulator PolyBLEP pulse.
         * Rising edge at phase=0, falling edge at phase=pw.
         * Falling edge PolyBLEP uses explicit phase shift instead of fmodf. */
        if (pulseLevel > 0.0f)
        {
            float pw = pulseWidth + pwmMod * 0.49f;
            if (pw < 0.01f) pw = 0.01f;
            if (pw > 0.99f) pw = 0.99f;

            float dt = pulsePhaseInc * fmRatio;
            float p  = pulsePhase;

            float v = (p < pw) ? 1.0f : -1.0f;

            /* rising edge PolyBLEP at phase=0 */
            v += polyblep(p, dt);

            /* falling edge PolyBLEP at phase=pw
             * shift so falling edge is at 0 for the helper */
            float p2 = p - pw;
            if (p2 < 0.0f) p2 += 1.0f;
            v -= polyblep(p2, dt);

            sample += v * pulseLevel;

            pulsePhase += dt;
            if (pulsePhase >= 1.0f)
                pulsePhase -= 1.0f;
        }

        /* --- Sub oscillator --- */
        if (subLevel > 0.0f)
        {
            float dt = subPhaseInc * fmRatio;
            float v  = (subPhase < 0.5f) ? 1.0f : -1.0f;
            sample += v * subLevel;

            subPhase += dt;
            if (subPhase >= 1.0f)
                subPhase -= 1.0f;
        }

        output[n] = sample;
    }
}

/* --------------------------------------------------------
 * Audio processing - DCO2 (pulse + sub at offset pitch)
 * Shares note, FM modulation and LFO with DCO1
 * -------------------------------------------------------- */
void DCO2_Process(float *output, int len)
{

    for (int n = 0; n < len; n++)
    {
        /* reuse lfoOutput computed in DCO_Process this sample
         * DCO2_Process must be called after DCO_Process each block */
        float fmMod = (lfoOutput * lfoFMDepth)
                    + (adcFM * fmDepth)
                    + modWheel
                    + aftertouch;
        if (fmMod >  1.0f) fmMod =  1.0f;
        if (fmMod < -1.0f) fmMod = -1.0f;
        float fmRatio = powf(2.0f, fmMod * FM_SEMITONE_RANGE / 12.0f);

        float pwmMod = (lfoOutput * dco2LFOPWMDepth)
                     + (adcPWM * dco2ADCPWMDepth)
                     + (modWheel * dco2PWMDepth);
        if (pwmMod >  1.0f) pwmMod =  1.0f;
        if (pwmMod < -1.0f) pwmMod = -1.0f;

        float sample = 0.0f;

        /* --- DCO2 Pulse --- */
        if (dco2PulseLevel > 0.0f)
        {
            float pw = dco2PulseWidth + pwmMod * 0.49f;
            if (pw < 0.01f) pw = 0.01f;
            if (pw > 0.99f) pw = 0.99f;

            float dt = dco2PulseInc * fmRatio;
            float p  = dco2PulsePhase;

            float v = (p < pw) ? 1.0f : -1.0f;
            v += polyblep(p, dt);
            float p2 = p - pw;
            if (p2 < 0.0f) p2 += 1.0f;
            v -= polyblep(p2, dt);

            sample += v * dco2PulseLevel;

            dco2PulsePhase += dt;
            if (dco2PulsePhase >= 1.0f)
                dco2PulsePhase -= 1.0f;
        }

        /* --- DCO2 Sub --- */
        if (dco2SubLevel > 0.0f)
        {
            float dt = dco2SubInc * fmRatio;
            float v  = (dco2SubPhase < 0.5f) ? 1.0f : -1.0f;
            sample += v * dco2SubLevel;

            dco2SubPhase += dt;
            if (dco2SubPhase >= 1.0f)
                dco2SubPhase -= 1.0f;
        }

        output[n] = sample;
    }
}
