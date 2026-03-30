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
static float    dco2ADCPWMDepth = 0.0f;   /* ADC PWM depth */

/* --- DCO2 Sub --- */
static float    dco2SubPhase    = 0.0f;
static float    dco2SubInc      = 0.0f;
static float    dco2SubLevel    = 0.0f;

/* --- DCO2 Sawtooth --- */
static float    dco2SawPhase    = 0.0f;
static float    dco2SawInc      = 0.0f;
static float    dco2SawLevel    = 0.0f;

/* --- DCO2 Pitch offset --- */
static float    dco2Detune      = 0.0f;   /* cents, -100 to +100 */
static int8_t   dco2Interval    = 0;      /* semitones, -24 to +24 */
static float    dco2PitchRatio  = 1.0f;   /* combined detune+interval ratio */

/* --- Oscillator Sync --- */
#define SYNC_OFF    0
#define SYNC_SOFT   1
#define SYNC_HARD   2
static uint8_t  syncMode        = SYNC_OFF;
static bool     dco1ResetFlag   = false;   /* set when DCO1 saw resets this buffer */

/* --- DCO2 Sweep Envelope (ADSR) --- */
typedef enum { ENV_IDLE, ENV_ATTACK, ENV_DECAY, ENV_SUSTAIN, ENV_RELEASE } EnvState;

static EnvState envState       = ENV_IDLE;
static float    envOutput      = 0.0f;   /* 0.0 - 1.0 */
static float    envAttackRate  = 0.0f;   /* per sample increment */
static float    envDecayRate   = 0.0f;   /* per sample decrement */
static float    envSustainLevel= 0.8f;   /* 0.0 - 1.0 */
static float    envReleaseRate = 0.0f;   /* per sample decrement */
static float    envSweepDepth  = 0.0f;   /* 0.0 - 1.0, scales DCO2 pitch range */
static float    dco2SweepRatio = 1.0f;   /* pitch ratio from envelope */
static float    envDCO1PWMDepth= 0.0f;   /* 0.0 - 1.0, envelope -> DCO1 PWM */
static float    envDCO2PWMDepth= 0.0f;   /* 0.0 - 1.0, envelope -> DCO2 PWM */

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
static float    modWheelFMDepth = 0.0f;     /* 0.0 - 1.0, mod wheel -> FM depth */
static float    aftertouch      = 0.0f;     /* 0.0 - 1.0 */
static float    atFMDepth       = 0.0f;     /* 0.0 - 1.0, aftertouch -> FM depth */

/* --- ADC Modulation --- */
static float    adcFM           = 0.0f;     /* -1.0 to +1.0 */
static float    adcPWM          = 0.0f;     /* -1.0 to +1.0 */
static float    fmDepth         = 0.0f;     /* 0.0 - 1.0, scales ADC FM input */
static float    adcPWMDepth     = 0.0f;     /* 0.0 - 1.0, scales ADC PWM input */

/* --- X-MOD (DCO2 -> DCO1 frequency only) --- */
static float    adcXMod         = 0.0f;     /* -1.0 to +1.0, from ADC2 */
static float    xModDepth       = 0.0f;     /* 0.0 - 1.0, scales X-MOD input */

/* --- LFO --- */
static float    lfoPhase        = 0.0f;     /* 0.0 - 1.0 */
static float    lfoPhaseInc     = 0.0f;     /* per sample */
static uint8_t  lfoWaveform     = LFO_TRIANGLE;
static float    lfoFMDepth      = 0.0f;     /* 0.0 - 1.0 */
static float    lfoPWMDepth     = 0.0f;     /* 0.0 - 1.0 */
static float    lfoOutput       = 0.0f;     /* current LFO value -1.0 to +1.0 */

/* --- LFO2 (dedicated to PWM) --- */
static float    lfo2Phase       = 0.0f;
static float    lfo2PhaseInc    = 0.0f;
static uint8_t  lfo2Waveform    = LFO_TRIANGLE;
static float    lfo2PWMDepth    = 0.0f;     /* LFO2 -> DCO1 PWM depth */
static float    lfo2DCO2PWMDepth= 0.0f;     /* LFO2 -> DCO2 PWM depth */
static float    lfo2Output      = 0.0f;

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

    return out;
}

/* --------------------------------------------------------
 * Internal: generate LFO2 sample (no phase advance - done externally)
 * -------------------------------------------------------- */
static float lfo2Tick(void)
{
    float out = 0.0f;
    switch (lfo2Waveform)
    {
        case LFO_TRIANGLE:
            if (lfo2Phase < 0.5f)
                out = 4.0f * lfo2Phase - 1.0f;
            else
                out = 3.0f - 4.0f * lfo2Phase;
            break;
        case LFO_SQUARE:
            out = (lfo2Phase < 0.5f) ? 1.0f : -1.0f;
            break;
        case LFO_SAWTOOTH:
            out = 2.0f * lfo2Phase - 1.0f;
            break;
        default:
            out = 0.0f;
            break;
    }
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

    /* dco2SweepRatio is applied per-sample in DCO2_Process
     * so recalcPhaseIncs only uses the static pitch offset */
    float freq2      = freq * dco2PitchRatio;
    dco2PulseInc     = freq2 / sampleRate;
    dco2SubInc       = (freq2 * 0.5f) / sampleRate;
    dco2SawInc       = freq2 / sampleRate;

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
    dco2ADCPWMDepth = 0.0f;
    dco2SubPhase   = 0.0f;
    dco2SubInc     = 0.0f;
    dco2SubLevel   = 0.0f;
    dco2SawPhase   = 0.0f;
    dco2SawInc     = 0.0f;
    dco2SawLevel   = 0.0f;
    dco2Detune     = 0.0f;
    dco2Interval   = 0;
    dco2PitchRatio = 1.0f;

    syncMode       = SYNC_OFF;
    dco1ResetFlag  = false;

    envState       = ENV_IDLE;
    envOutput      = 0.0f;
    envAttackRate  = 0.001f;  /* ~1s attack default */
    envDecayRate   = 0.002f;  /* ~0.5s decay default */
    envSustainLevel= 0.8f;
    envReleaseRate = 0.001f;  /* ~1s release default */
    envSweepDepth   = 0.0f;
    dco2SweepRatio  = 1.0f;
    envDCO1PWMDepth = 0.0f;
    envDCO2PWMDepth = 0.0f;

    baseFreq     = 0.0f;
    currentFreq  = 0.0f;
    targetFreq   = 0.0f;
    bendRatio    = 1.0f;
    portoEnabled = false;
    portoRate    = 0.0f;
    bendRange   = 2;
    noteActive  = false;
    currentNote = 255;

    modWheel        = 0.0f;
    modWheelFMDepth = 0.0f;
    aftertouch      = 0.0f;
    atFMDepth       = 0.0f;
    adcFM       = 0.0f;
    adcPWM      = 0.0f;
    fmDepth     = 0.0f;
    adcPWMDepth = 0.0f;
    adcXMod     = 0.0f;
    xModDepth   = 0.0f;

    lfoPhase    = 0.5f;   /* sawtooth at 0.5 = 0 output */
    lfoPhaseInc = LFO_RATE_MIN / sample_rate;
    lfoWaveform = LFO_SAWTOOTH;
    lfoFMDepth  = 0.0f;
    lfoPWMDepth = 0.0f;     /* unused - kept for compatibility */
    lfoOutput   = 0.0f;

    lfo2Phase        = 0.25f;  /* triangle at 0.25 = 0 output */
    lfo2PhaseInc     = LFO_RATE_MIN / sample_rate;
    lfo2Waveform     = LFO_TRIANGLE;
    lfo2PWMDepth     = 0.0f;
    lfo2DCO2PWMDepth = 0.0f;
    lfo2Output       = 0.0f;
}

/* --------------------------------------------------------
 * Note control
 * -------------------------------------------------------- */
void DCO_NoteOn(uint8_t note, uint8_t vel)
{
    (void)vel;          /* velocity unused - DCO runs at constant level */
    currentNote = note;
    noteActive  = true;
    envState    = ENV_ATTACK;   /* trigger sweep envelope */
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
    dco2SawPhase   = 0.0f;

    recalcPhaseIncs();
}

void DCO_NoteOff(uint8_t note)
{
    if (note == currentNote)
    {
        noteActive = false;
        aftertouch = 0.0f;        /* stop aftertouch vibrato */
        if (envState != ENV_IDLE)
            envState = ENV_RELEASE;   /* trigger envelope release */
    }
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

void DCO_SetModWheelFMDepth(uint8_t value)
{
    modWheelFMDepth = (float)value / 127.0f;
}

void DCO_SetAftertouch(uint8_t value)
{
    aftertouch = (float)value / 127.0f;
}

void DCO_SetAftertouchFMDepth(uint8_t value)
{
    atFMDepth = (float)value / 127.0f;
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

static uint16_t xModZero = 2048;    /* calibrated zero point, default = 1.65V */
static uint32_t xModCalSum   = 0;
static uint16_t xModCalCount = 0;

void DCO_CalibrateXMod(uint16_t raw)
{
    /* accumulate readings then set zero point directly */
    xModCalSum += raw;
    xModCalCount++;
    xModZero = (uint16_t)(xModCalSum / xModCalCount);
}

void DCO_SetADC_XMod(uint16_t raw)
{
    /* use calibrated zero point instead of hardcoded 2048 */
    float v = ((float)raw - (float)xModZero) / 2048.0f;
    if (v >  1.0f) v =  1.0f;
    if (v < -1.0f) v = -1.0f;
    if (v > -0.05f && v < 0.05f) v = 0.0f;  /* ±5% deadband suppresses noise */
    adcXMod = v;
}

void DCO_SetXModDepth(uint8_t value)
{
    xModDepth = (float)value / 127.0f;
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

    /* reset phase and smoother to neutral on waveform change
     * triangle at 0.25 = 0 output, square/saw at 0.5 = 0 output */
    if (lfoWaveform == LFO_TRIANGLE)
        lfoPhase = 0.25f;
    else if (lfoWaveform == LFO_SQUARE)
        lfoPhase = 0.0f;    /* starts +1 but smoother handles transition */
    else
        lfoPhase = 0.5f;    /* sawtooth at 0.5 = 0 output */

    lfoOutput   = 0.0f;
}

void DCO_SetLFOFMDepth(uint8_t value)
{
    lfoFMDepth = (float)value / 127.0f;
}

void DCO_SetLFOPWMDepth(uint8_t value)
{
    lfoPWMDepth = (float)value / 127.0f;  /* kept for compatibility */
}

/* --------------------------------------------------------
 * LFO2 parameters (dedicated PWM LFO)
 * -------------------------------------------------------- */
void DCO_SetLFO2Rate(uint8_t value)
{
    float t    = (float)value / 127.0f;
    float rate = LFO_RATE_MIN * powf(LFO_RATE_MAX / LFO_RATE_MIN, t);
    lfo2PhaseInc = rate / sampleRate;
}

void DCO_SetLFO2Waveform(uint8_t value)
{
    if      (value < 43)  lfo2Waveform = LFO_TRIANGLE;
    else if (value < 85)  lfo2Waveform = LFO_SQUARE;
    else                  lfo2Waveform = LFO_SAWTOOTH;

    if (lfo2Waveform == LFO_TRIANGLE)
        lfo2Phase = 0.25f;
    else
        lfo2Phase = 0.5f;

    lfo2Output = 0.0f;
}

void DCO_SetLFO2PWMDepth(uint8_t value)
{
    lfo2PWMDepth = (float)value / 127.0f;
}

void DCO_SetLFO2DCO2PWMDepth(uint8_t value)
{
    lfo2DCO2PWMDepth = (float)value / 127.0f;
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
 * Envelope helper - call once per sample
 * -------------------------------------------------------- */
static void envTick(void)
{
    switch (envState)
    {
        case ENV_ATTACK:
            envOutput += envAttackRate;
            if (envOutput >= 1.0f)
            {
                envOutput = 1.0f;
                envState  = ENV_DECAY;
            }
            break;

        case ENV_DECAY:
            envOutput -= envDecayRate;
            if (envOutput <= envSustainLevel)
            {
                envOutput = envSustainLevel;
                envState  = ENV_SUSTAIN;
            }
            break;

        case ENV_SUSTAIN:
            envOutput = envSustainLevel;
            break;

        case ENV_RELEASE:
            envOutput -= envReleaseRate;
            if (envOutput <= 0.0f)
            {
                envOutput = 0.0f;
                envState  = ENV_IDLE;
            }
            break;

        case ENV_IDLE:
        default:
            envOutput = 0.0f;
            break;
    }

    /* dco2SweepRatio is computed once per buffer in DCO_ProcessBoth */
}

/* --------------------------------------------------------
 * Oscillator sync
 * -------------------------------------------------------- */
void DCO_SetSyncMode(uint8_t value)
{
    /* 0-42=off, 43-84=soft sync, 85-127=hard sync */
    if      (value < 43)  syncMode = SYNC_OFF;
    else if (value < 85)  syncMode = SYNC_SOFT;
    else                  syncMode = SYNC_HARD;
}

/* --------------------------------------------------------
 * DCO2 sweep envelope parameters
 * Time CCs map 0-127 to rate: 0=slowest, 127=fastest
 * Rate = 1 / (time_samples) where time ranges from ~10ms to ~10s
 * -------------------------------------------------------- */
static float timeToRate(uint8_t value)
{
    /* map 0-127 to time 10s down to 10ms exponentially
     * rate = 1/time_in_samples */
    float t       = (float)value / 127.0f;
    float timeSec = 10.0f * powf(0.001f, t);   /* 10s -> 0.01s */
    return 1.0f / (timeSec * DCO_SAMPLE_RATE);
}

void DCO_SetEnvAttack(uint8_t value)
{
    /* invert: 0=long attack, 127=short attack */
    envAttackRate = timeToRate(127 - value);
}

void DCO_SetEnvDecay(uint8_t value)
{
    /* invert: 0=long decay, 127=short decay */
    envDecayRate = timeToRate(127 - value);
}

void DCO_SetEnvSustain(uint8_t value)
{
    envSustainLevel = (float)value / 127.0f;
}

void DCO_SetEnvRelease(uint8_t value)
{
    /* invert: 0=long release, 127=short release */
    envReleaseRate = timeToRate(127 - value);
}

void DCO_SetEnvSweepDepth(uint8_t value)
{
    envSweepDepth = (float)value / 127.0f;
}

void DCO_SetEnvDCO1PWMDepth(uint8_t value)
{
    envDCO1PWMDepth = (float)value / 127.0f;
}

void DCO_SetEnvDCO2PWMDepth(uint8_t value)
{
    envDCO2PWMDepth = (float)value / 127.0f;
}

/* --------------------------------------------------------
 * DCO2 parameters
 * -------------------------------------------------------- */
void DCO2_SetSawLevel(uint8_t value)
{
    dco2SawLevel = (float)value / 127.0f;
}

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

    dco1ResetFlag = false;   /* clear sync flag at start of each buffer */

    /* --- Compute LFO, FM and PWM once per buffer ---
     * Advance LFO phase by full buffer length then sample output once.
     * fmRatio and pwmMod hold constant for the buffer duration.
     * This eliminates per-sample discontinuity artifacts from square/saw. */
    /* advance LFO phase by full buffer then compute output */
    lfoPhase += lfoPhaseInc * (float)len;
    while (lfoPhase >= 1.0f) lfoPhase -= 1.0f;
    lfoOutput = lfoTick();

    float effectiveLFOFMDepth = lfoFMDepth + (aftertouch * atFMDepth);
    if (effectiveLFOFMDepth > 1.0f) effectiveLFOFMDepth = 1.0f;

    float fmMod = (lfoOutput * effectiveLFOFMDepth)
                + (adcFM * fmDepth)
                + (modWheel * modWheelFMDepth);
    if (fmMod >  1.0f) fmMod =  1.0f;
    if (fmMod < -1.0f) fmMod = -1.0f;

    float fmRatio = (fmMod == 0.0f) ? 1.0f : powf(2.0f, fmMod * FM_SEMITONE_RANGE / 12.0f);

    float pwmMod = (lfo2Output * lfo2PWMDepth)
                 + (adcPWM * adcPWMDepth)
                 + (modWheel * pwmDepth);
    if (pwmMod >  1.0f) pwmMod =  1.0f;
    if (pwmMod < -1.0f) pwmMod = -1.0f;

    for (int n = 0; n < len; n++)
    {

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
                {
                    sawVoices[i].phase -= 1.0f;
                    if (i == 0) dco1ResetFlag = true;  /* track voice 0 as sync source */
                }
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
            {
                pulsePhase -= 1.0f;
                if (sawLevel == 0.0f) dco1ResetFlag = true;  /* use pulse as sync source when saw off */
            }
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

    /* reuse lfoOutput, fmRatio and pwmMod from DCO_Process this buffer
     * DCO2_Process must be called immediately after DCO_Process */
    float effectiveLFOFMDepth = lfoFMDepth + (aftertouch * atFMDepth);
    if (effectiveLFOFMDepth > 1.0f) effectiveLFOFMDepth = 1.0f;

    float fmMod = (lfoOutput * effectiveLFOFMDepth)
                + (adcFM * fmDepth)
                + (modWheel * modWheelFMDepth);
    if (fmMod >  1.0f) fmMod =  1.0f;
    if (fmMod < -1.0f) fmMod = -1.0f;
    float fmRatio = (fmMod == 0.0f) ? 1.0f : powf(2.0f, fmMod * FM_SEMITONE_RANGE / 12.0f);

    float dco2pwmMod = (lfo2Output * lfo2DCO2PWMDepth)
                     + (adcPWM * dco2ADCPWMDepth)
                     + (modWheel * dco2PWMDepth);
    if (dco2pwmMod >  1.0f) dco2pwmMod =  1.0f;
    if (dco2pwmMod < -1.0f) dco2pwmMod = -1.0f;

    for (int n = 0; n < len; n++)
    {

        float sample = 0.0f;

        /* --- Envelope tick --- */
        envTick();

        /* --- Oscillator sync --- */
        if (dco1ResetFlag && syncMode != SYNC_OFF)
        {
            if (syncMode == SYNC_HARD)
            {
                /* hard sync: reset DCO2 phase to 0 */
                dco2PulsePhase = 0.0f;
                dco2SubPhase   = 0.0f;
            }
            else
            {
                /* soft sync: invert DCO2 phase */
                dco2PulsePhase = 1.0f - dco2PulsePhase;
                dco2SubPhase   = 1.0f - dco2SubPhase;
            }
            dco1ResetFlag = false;
        }

        /* --- DCO2 Pulse --- */
        if (dco2PulseLevel > 0.0f)
        {
            float pw = dco2PulseWidth + dco2pwmMod * 0.49f;
            if (pw < 0.01f) pw = 0.01f;
            if (pw > 0.99f) pw = 0.99f;

            float dt = dco2PulseInc * fmRatio * dco2SweepRatio;
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
            float dt = dco2SubInc * fmRatio * dco2SweepRatio;
            float v  = (dco2SubPhase < 0.5f) ? 1.0f : -1.0f;
            sample += v * dco2SubLevel;

            dco2SubPhase += dt;
            if (dco2SubPhase >= 1.0f)
                dco2SubPhase -= 1.0f;
        }

        output[n] = sample;
    }
}

/* --------------------------------------------------------
 * Combined audio processing - DCO1 and DCO2 in one loop
 * This is required for accurate oscillator sync - both DCOs
 * must run in the same sample loop so sync happens at the
 * exact sample when DCO1 resets, not at buffer boundaries.
 * Use this instead of calling DCO_Process + DCO2_Process separately.
 * -------------------------------------------------------- */
void DCO_ProcessBoth(float *out1, float *out2, int len)
{
    /* --- Portamento slew --- */
    if (portoEnabled && currentFreq != targetFreq)
    {
        float ratio = targetFreq / currentFreq;
        float step  = powf(2.0f, portoRate);
        if (ratio > step)
            currentFreq *= step;
        else if (ratio < 1.0f / step)
            currentFreq /= step;
        else
            currentFreq = targetFreq;
        baseFreq = currentFreq;
        recalcPhaseIncs();
    }

    /* advance envelope one step to get current value for buffer calculations */
    envTick();

    /* --- LFO1 (FM) and LFO2 (PWM) once per buffer --- */
    lfoPhase += lfoPhaseInc * (float)len;
    while (lfoPhase >= 1.0f) lfoPhase -= 1.0f;
    lfoOutput = lfoTick();

    lfo2Phase += lfo2PhaseInc * (float)len;
    while (lfo2Phase >= 1.0f) lfo2Phase -= 1.0f;
    lfo2Output = lfo2Tick();

    /* LFO1 -> FM only */
    float effectiveLFOFMDepth = lfoFMDepth + (aftertouch * atFMDepth);
    if (effectiveLFOFMDepth > 1.0f) effectiveLFOFMDepth = 1.0f;

    float fmMod = (lfoOutput * effectiveLFOFMDepth)
                + (adcFM * fmDepth)
                + (modWheel * modWheelFMDepth);
    if (fmMod >  1.0f) fmMod =  1.0f;
    if (fmMod < -1.0f) fmMod = -1.0f;
    /* shared FM ratio for DCO2 */
    float fmRatio = (fmMod == 0.0f) ? 1.0f : powf(2.0f, fmMod * FM_SEMITONE_RANGE / 12.0f);

    /* DCO1 gets additional X-MOD from ADC2 (DCO2 output fed back) */
    float xModMod = adcXMod * xModDepth;
    float dco1FMRatio = ((fmMod + xModMod) == 0.0f) ? 1.0f
                      : powf(2.0f, (fmMod + xModMod) * FM_SEMITONE_RANGE / 12.0f);

    /* DCO2 sweep ratio from envelope - once per buffer using current envOutput */
    if (envSweepDepth > 0.0f)
    {
        float semitones = envOutput * envSweepDepth * 12.0f;
        dco2SweepRatio  = powf(2.0f, semitones / 12.0f);
    }
    else
    {
        dco2SweepRatio = 1.0f;
    }

    /* LFO2 -> PWM for DCO1 and DCO2 independently */
    float pwmMod1 = (lfo2Output * lfo2PWMDepth)
                  + (adcPWM * adcPWMDepth)
                  + (modWheel * pwmDepth)
                  + (envOutput * envDCO1PWMDepth);
    if (pwmMod1 >  1.0f) pwmMod1 =  1.0f;
    if (pwmMod1 < -1.0f) pwmMod1 = -1.0f;

    float pwmMod2 = (lfo2Output * lfo2DCO2PWMDepth)
                  + (adcPWM * dco2ADCPWMDepth)
                  + (modWheel * dco2PWMDepth)
                  + (envOutput * envDCO2PWMDepth);
    if (pwmMod2 >  1.0f) pwmMod2 =  1.0f;
    if (pwmMod2 < -1.0f) pwmMod2 = -1.0f;

    for (int n = 0; n < len; n++)
    {
        /* --- Envelope tick (per sample) --- */
        envTick();

        float sample1 = 0.0f;
        float sample2 = 0.0f;
        bool  dco1Reset = false;

        /* --- DCO1 Multi-saw --- */
        if (sawLevel > 0.0f)
        {
            float sawSample = 0.0f;
            for (int i = 0; i < sawCount; i++)
            {
                float p  = sawVoices[i].phase;
                float dt = sawVoices[i].phaseInc * dco1FMRatio;
                float v  = 2.0f * p - 1.0f;
                v -= polyblep(p, dt);
                sawSample += v;

                sawVoices[i].phase += dt;
                if (sawVoices[i].phase >= 1.0f)
                {
                    sawVoices[i].phase -= 1.0f;
                    if (i == 0) dco1Reset = true;
                }
            }
            sawSample /= (float)sawCount;
            sample1 += sawSample * sawLevel;
        }

        /* --- DCO1 Pulse --- */
        if (pulseLevel > 0.0f)
        {
            float pw = pulseWidth + pwmMod1 * 0.49f;
            if (pw < 0.01f) pw = 0.01f;
            if (pw > 0.99f) pw = 0.99f;

            float dt = pulsePhaseInc * dco1FMRatio;
            float p  = pulsePhase;
            float v  = (p < pw) ? 1.0f : -1.0f;
            v += polyblep(p, dt);
            float p2 = p - pw;
            if (p2 < 0.0f) p2 += 1.0f;
            v -= polyblep(p2, dt);
            sample1 += v * pulseLevel;

            pulsePhase += dt;
            if (pulsePhase >= 1.0f)
            {
                pulsePhase -= 1.0f;
                if (sawLevel == 0.0f) dco1Reset = true;
            }
        }

        /* --- DCO1 Sub --- */
        if (subLevel > 0.0f)
        {
            float dt = subPhaseInc * dco1FMRatio;
            float v  = (subPhase < 0.5f) ? 1.0f : -1.0f;
            sample1 += v * subLevel;
            subPhase += dt;
            if (subPhase >= 1.0f) subPhase -= 1.0f;
        }

        /* --- Oscillator sync - apply at exact sample of DCO1 reset --- */
        if (dco1Reset && syncMode != SYNC_OFF)
        {
            if (syncMode == SYNC_HARD)
            {
                dco2PulsePhase = 0.0f;
                dco2SubPhase   = 0.0f;
            }
            else /* SYNC_SOFT */
            {
                dco2PulsePhase = 1.0f - dco2PulsePhase;
                dco2SubPhase   = 1.0f - dco2SubPhase;
            }
        }

        /* --- DCO2 Pulse --- */
        if (dco2PulseLevel > 0.0f)
        {
            float pw = dco2PulseWidth + pwmMod2 * 0.49f;
            if (pw < 0.01f) pw = 0.01f;
            if (pw > 0.99f) pw = 0.99f;

            float dt = dco2PulseInc * fmRatio * dco2SweepRatio;
            float p  = dco2PulsePhase;
            float v  = (p < pw) ? 1.0f : -1.0f;
            v += polyblep(p, dt);
            float p2 = p - pw;
            if (p2 < 0.0f) p2 += 1.0f;
            v -= polyblep(p2, dt);
            sample2 += v * dco2PulseLevel;

            dco2PulsePhase += dt;
            if (dco2PulsePhase >= 1.0f) dco2PulsePhase -= 1.0f;
        }

        /* --- DCO2 Sub --- */
        if (dco2SubLevel > 0.0f)
        {
            float dt = dco2SubInc * fmRatio * dco2SweepRatio;
            float v  = (dco2SubPhase < 0.5f) ? 1.0f : -1.0f;
            sample2 += v * dco2SubLevel;
            dco2SubPhase += dt;
            if (dco2SubPhase >= 1.0f) dco2SubPhase -= 1.0f;
        }

        /* --- DCO2 Sawtooth --- */
        if (dco2SawLevel > 0.0f)
        {
            float dt = dco2SawInc * fmRatio * dco2SweepRatio;
            float p  = dco2SawPhase;
            float v  = 2.0f * p - 1.0f;
            v -= polyblep(p, dt);
            sample2 += v * dco2SawLevel;
            dco2SawPhase += dt;
            if (dco2SawPhase >= 1.0f) dco2SawPhase -= 1.0f;
        }

        out1[n] = sample1;
        out2[n] = sample2;
    }
}
