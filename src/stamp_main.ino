/*
 * stamp_main.ino
 *
 * RP2350-Zero Dual DCO stamp - main sketch
 *
 * Wiring:
 *   MIDI IN    -> GPIO0  (Serial1 RX)
 *   DCO1 OUT   -> GPIO2  (PWM, RC filter: 1k + 47nF to GND)
 *   DCO2 OUT   -> GPIO4  (PWM, RC filter: 1k + 47nF to GND)
 *   GATE OUT   -> GPIO3  (3.3V high = note active)
 *   FM IN      -> GPIO26 (ADC0, bias to 1.65V)
 *   PWM IN     -> GPIO27 (ADC1, bias to 1.65V)
 *
 * Dependencies:
 *   - Arduino MIDI Library (FortySevenEffects) via Library Manager
 *
 * Change VOICE_CHANNEL per stamp (1-8)
 * CONTROL_CHANNEL is the same on all stamps
 */

#include <MIDI.h>
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "pwm_audio.h"
#include "dco_engine.h"

/* --------------------------------------------------------
 * Pin assignments
 * -------------------------------------------------------- */
#define PWM_DCO1_PIN    2
#define PWM_DCO2_PIN    4
#define GATE_PIN        3
#define ADC_FM_PIN      26
#define ADC_PWM_PIN     27
#define VELOCITY_PWM_PIN 5      /* GPIO5 - velocity CV output, RC filter: 1k + 10nF */
#define KEYTRACK_PWM_PIN 6      /* GPIO6 - keytrack CV output, RC filter: 1k + 10nF */

/* --------------------------------------------------------
 * MIDI channels (1-based)
 * -------------------------------------------------------- */
#define VOICE_CHANNEL    1      /* <<< change per stamp: 1-8 */
#define CONTROL_CHANNEL  9      /* shared across all stamps  */

/* --------------------------------------------------------
 * CC assignments on the control channel
 * DCO1
 * -------------------------------------------------------- */
#define CC_MOD_WHEEL        1
#define CC_PORTAMENTO_TIME  5   /* portamento rate                     */
#define CC_PORTAMENTO_SW    65  /* portamento on/off (>=64 on, <64 off) */
#define CC_SAW_DETUNE       17
#define CC_SAW_COUNT        18
#define CC_PULSE_WIDTH      19
#define CC_PWM_DEPTH        20
#define CC_SAW_LEVEL        21
#define CC_PULSE_LEVEL      22
#define CC_SUB_LEVEL        23
#define CC_PITCHBEND_RANGE  24
#define CC_FM_DEPTH         25
#define CC_LFO_RATE         26
#define CC_LFO_WAVEFORM     27
#define CC_LFO_FM_DEPTH     28
#define CC_LFO_PWM_DEPTH    29
#define CC_ADC_PWM_DEPTH    30

/* DCO2 */
#define CC_DCO2_PULSE_WIDTH 31
#define CC_DCO2_PULSE_LEVEL 32
#define CC_DCO2_SUB_LEVEL   33
#define CC_DCO2_DETUNE      34  /* centre=64 -> 0 cents, range ±100  */
#define CC_DCO2_INTERVAL    35  /* centre=64 -> 0 semitones, range ±24 */
#define CC_DCO2_PWM_DEPTH   36  /* mod wheel PWM depth                 */
#define CC_DCO2_LFO_PWM     37  /* LFO -> PWM depth                    */
#define CC_DCO2_ADC_PWM     38  /* ADC PWM input depth                 */

/* --------------------------------------------------------
 * Velocity PWM output
 * GPIO5, ~50kHz carrier, 8-bit resolution
 * RC filter: 1k + 10nF -> clean 0-3.3V DC
 * 0 velocity = 0V, 127 velocity = 3.3V
 * -------------------------------------------------------- */
static uint8_t  velPWMSlice = 0;
static uint8_t  velPWMChan  = 0;

static void VelocityPWM_Init(uint8_t pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    velPWMSlice = pwm_gpio_to_slice_num(pin);
    velPWMChan  = pwm_gpio_to_channel(pin);

    /* 8-bit wrap = 255
     * target ~50kHz: clkdiv = 150MHz / (50000 * 256) = ~11.72 */
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 11.72f);
    pwm_config_set_wrap(&cfg, 255);
    pwm_init(velPWMSlice, &cfg, true);

    /* start at zero */
    pwm_set_chan_level(velPWMSlice, velPWMChan, 0);
}

static void VelocityPWM_Set(uint8_t velocity)
{
    /* 0-127 mapped to 0-255 PWM level (full 0-3.3V range) */
    pwm_set_chan_level(velPWMSlice, velPWMChan, (uint16_t)velocity * 2);
}

/* --------------------------------------------------------
 * Keytrack PWM output
 * GPIO6, ~50kHz carrier, 8-bit resolution
 * 0.25V per octave (12 semitones)
 * 3.3V full scale = ~13 octaves
 * Note 12 = 0.25V, Note 24 = 0.5V etc.
 * RC filter: 1k + 10nF -> clean 0-3.3V DC
 * -------------------------------------------------------- */
static uint8_t  keytrackSlice = 0;
static uint8_t  keytrackChan  = 0;

static void KeytrackPWM_Init(uint8_t pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    keytrackSlice = pwm_gpio_to_slice_num(pin);
    keytrackChan  = pwm_gpio_to_channel(pin);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 11.72f);    /* ~50kHz carrier */
    pwm_config_set_wrap(&cfg, 255);
    pwm_init(keytrackSlice, &cfg, true);

    pwm_set_chan_level(keytrackSlice, keytrackChan, 0);
}

static void KeytrackPWM_Set(uint8_t note)
{
    /* 0.25V per octave = 0.25V / 12 semitones = 0.020833V per semitone
     * PWM level = note * (0.020833 / 3.3) * 255 = note * 1.6083 */
    uint16_t level = (uint16_t)((float)note * 1.6083f);
    if (level > 255) level = 255;
    pwm_set_chan_level(keytrackSlice, keytrackChan, level);
}

/* --------------------------------------------------------
 * MIDI instance on Serial1 (GPIO0=RX)
 * -------------------------------------------------------- */
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

/* --------------------------------------------------------
 * ADC timer at 1kHz
 * -------------------------------------------------------- */
static repeating_timer_t adcTimer;

static bool adcTimerCallback(repeating_timer_t *rt)
{
    (void)rt;
    DCO_SetADC_FM(analogRead(ADC_FM_PIN));
    DCO_SetADC_PWM(analogRead(ADC_PWM_PIN));
    return true;
}

/* --------------------------------------------------------
 * MIDI callbacks
 * -------------------------------------------------------- */
void myNoteOn(byte channel, byte note, byte velocity)
{
    if (channel == VOICE_CHANNEL)
    {
        VelocityPWM_Set(velocity);
        KeytrackPWM_Set(note);
        digitalWrite(GATE_PIN, HIGH);
        DCO_NoteOn(note, velocity);
    }
}

void myNoteOff(byte channel, byte note, byte velocity)
{
    if (channel == VOICE_CHANNEL)
    {
        digitalWrite(GATE_PIN, LOW);
        DCO_NoteOff(note);
    }
}

void myControlChange(byte channel, byte cc, byte value)
{
    if (channel == CONTROL_CHANNEL)
    {
        switch (cc)
        {
            /* DCO1 */
            case CC_MOD_WHEEL:       DCO_SetModWheel(value);        break;
            case CC_PORTAMENTO_TIME: DCO_SetPortamentoRate(value);  break;
            case CC_PORTAMENTO_SW:   DCO_SetPortamento(value);      break;
            case CC_SAW_DETUNE:      DCO_SetSawDetune(value);       break;
            case CC_SAW_COUNT:       DCO_SetSawCount(value);        break;
            case CC_PULSE_WIDTH:     DCO_SetPulseWidth(value);      break;
            case CC_PWM_DEPTH:       DCO_SetPWMDepth(value);        break;
            case CC_SAW_LEVEL:       DCO_SetSawLevel(value);        break;
            case CC_PULSE_LEVEL:     DCO_SetPulseLevel(value);      break;
            case CC_SUB_LEVEL:       DCO_SetSubLevel(value);        break;
            case CC_PITCHBEND_RANGE: DCO_SetPitchBendRange(value);  break;
            case CC_FM_DEPTH:        DCO_SetFMDepth(value);         break;
            case CC_LFO_RATE:        DCO_SetLFORate(value);         break;
            case CC_LFO_WAVEFORM:    DCO_SetLFOWaveform(value);     break;
            case CC_LFO_FM_DEPTH:    DCO_SetLFOFMDepth(value);      break;
            case CC_LFO_PWM_DEPTH:   DCO_SetLFOPWMDepth(value);     break;
            case CC_ADC_PWM_DEPTH:   DCO_SetADCPWMDepth(value);     break;
            /* DCO2 */
            case CC_DCO2_PULSE_WIDTH: DCO2_SetPulseWidth(value);    break;
            case CC_DCO2_PULSE_LEVEL: DCO2_SetPulseLevel(value);    break;
            case CC_DCO2_SUB_LEVEL:   DCO2_SetSubLevel(value);      break;
            case CC_DCO2_DETUNE:      DCO2_SetDetune(value);        break;
            case CC_DCO2_INTERVAL:    DCO2_SetInterval(value);      break;
            case CC_DCO2_PWM_DEPTH:   DCO2_SetPWMDepth(value);      break;
            case CC_DCO2_LFO_PWM:     DCO2_SetLFOPWMDepth(value);   break;
            case CC_DCO2_ADC_PWM:     DCO2_SetADCPWMDepth(value);   break;
            default: break;
        }
    }
}

void myPitchBend(byte channel, int bend)
{
    if (channel == CONTROL_CHANNEL)
        DCO_PitchBend((uint16_t)(bend + 8192));
}

void myAfterTouch(byte channel, byte pressure)
{
    if (channel == CONTROL_CHANNEL)
        DCO_SetAftertouch(pressure);
}

/* --------------------------------------------------------
 * PWM audio callbacks
 * -------------------------------------------------------- */
void PWM_CB_FillBuffer_DCO1(float *output, int len)
{
    DCO_Process(output, len);
}

void PWM_CB_FillBuffer_DCO2(float *output, int len)
{
    DCO2_Process(output, len);
}

/* --------------------------------------------------------
 * Setup
 * -------------------------------------------------------- */
void setup()
{
    pinMode(GATE_PIN, OUTPUT);
    digitalWrite(GATE_PIN, LOW);

    /* Velocity PWM */
    VelocityPWM_Init(VELOCITY_PWM_PIN);

    /* Keytrack PWM */
    KeytrackPWM_Init(KEYTRACK_PWM_PIN);

    /* ADC */
    analogReadResolution(12);
    pinMode(ADC_FM_PIN,  INPUT);
    pinMode(ADC_PWM_PIN, INPUT);

    /* MIDI */
    MIDI.begin(0);
    MIDI.setHandleNoteOn(myNoteOn);
    MIDI.setHandleNoteOff(myNoteOff);
    MIDI.setHandleControlChange(myControlChange);
    MIDI.setHandlePitchBend(myPitchBend);
    MIDI.setHandleAfterTouchChannel(myAfterTouch);

    /* DCO engine */
    DCO_Init(48000.0f);

    /* DCO1 defaults */
    DCO_SetPortamento(0);       /* off by default */
    DCO_SetPortamentoRate(0);   /* fastest rate   */
    DCO_SetSawLevel(100);
    DCO_SetSawCount(20);
    DCO_SetSawDetune(0);
    DCO_SetPulseLevel(0);
    DCO_SetSubLevel(0);
    DCO_SetPulseWidth(64);
    DCO_SetPWMDepth(0);
    DCO_SetFMDepth(0);
    DCO_SetADCPWMDepth(0);
    DCO_SetPitchBendRange(2);
    DCO_SetLFORate(20);
    DCO_SetLFOWaveform(0);      /* triangle */
    DCO_SetLFOFMDepth(0);
    DCO_SetLFOPWMDepth(0);

    /* DCO2 defaults - silent until enabled via CC */
    DCO2_SetPulseWidth(64);     /* 50% square */
    DCO2_SetPulseLevel(0);      /* off until enabled */
    DCO2_SetSubLevel(0);        /* off until enabled */
    DCO2_SetDetune(64);         /* centre = 0 cents */
    DCO2_SetInterval(64);       /* centre = 0 semitones */
    DCO2_SetPWMDepth(0);
    DCO2_SetLFOPWMDepth(0);
    DCO2_SetADCPWMDepth(0);

    /* ADC timer at 1kHz */
    add_repeating_timer_ms(-1, adcTimerCallback, nullptr, &adcTimer);

    /* PWM audio - dual output, init last */
    PWMAudio_Init(PWM_DCO1_PIN, PWM_DCO2_PIN, 48000);
}

/* --------------------------------------------------------
 * Loop
 * -------------------------------------------------------- */
void loop()
{
    MIDI.read();
    PWMAudio_Process();
}
