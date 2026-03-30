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
 *   X-MOD IN   -> GPIO28 (ADC2, feed DCO2 output via 10k resistor)
 *
 * Dependencies:
 *   - Arduino MIDI Library (FortySevenEffects) via Library Manager
 *
 * Change VOICE_CHANNEL per stamp (1-8)
 * CONTROL_CHANNEL is the same on all stamps
 */

#include <MIDI.h>
#include <string.h>
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
#define ADC_XMOD_PIN    28      /* GPIO28 ADC2 - X-MOD input (DCO2->DCO1) */
#define VELOCITY_PWM_PIN 5      /* GPIO5 - velocity CV output, RC filter: 1k + 10nF */
#define KEYTRACK_PWM_PIN 6      /* GPIO6 - keytrack CV output, RC filter: 1k + 10nF */
#define AFTERTOUCH_PWM_PIN 7    /* GPIO7 - aftertouch CV output, RC filter: 1k + 10nF */

/* --------------------------------------------------------
 * MIDI channels (1-based)
 * -------------------------------------------------------- */
#define VOICE_CHANNEL    1      /* <<< change per stamp: 1-8 */
#define CONTROL_CHANNEL  9      /* shared across all stamps  */

/* --------------------------------------------------------
 * CC assignments on the control channel
 * DCO1
 * -------------------------------------------------------- */
/* --- Standard MIDI --- */
#define CC_MOD_WHEEL        1   /* modulation wheel                    */
#define CC_PORTAMENTO_TIME  5   /* portamento rate                     */
#define CC_PORTAMENTO_SW    65  /* portamento on/off (>=64=on)         */

/* --- DCO1 oscillator --- */
#define CC_DCO1_SAW_DETUNE  17  /* saw detune spread                   */
#define CC_DCO1_SAW_COUNT   18  /* saw voice count 1-5                 */
#define CC_DCO1_PULSE_WIDTH 19  /* DCO1 pulse width                    */
#define CC_DCO1_PWM_DEPTH   20  /* DCO1 mod wheel PWM depth            */
#define CC_DCO1_SAW_LEVEL   21  /* saw level in mix                    */
#define CC_DCO1_PULSE_LEVEL 22  /* DCO1 pulse level in mix             */
#define CC_DCO1_SUB_LEVEL   23  /* DCO1 sub level                      */

/* --- Pitch --- */
#define CC_PITCHBEND_RANGE  24  /* pitchbend range 1-12 semitones      */

/* --- LFO1 (FM/vibrato) --- */
#define CC_LFO1_RATE        25  /* LFO1 rate 0.1-20Hz                  */
#define CC_LFO1_WAVEFORM    26  /* LFO1 waveform tri/sq/saw            */
#define CC_LFO1_FM_DEPTH    27  /* LFO1 -> FM depth                    */
#define CC_AT_FM_DEPTH      28  /* aftertouch -> vibrato depth         */
#define CC_MW_FM_DEPTH      29  /* mod wheel -> FM depth               */
#define CC_ADC_FM_DEPTH     30  /* ADC FM input depth                  */
#define CC_XMOD_DEPTH       53  /* X-MOD depth DCO2->DCO1 freq         */

/* --- LFO2 (PWM) --- */
#define CC_LFO2_RATE        31  /* LFO2 rate 0.1-20Hz                  */
#define CC_LFO2_WAVEFORM    32  /* LFO2 waveform tri/sq/saw            */
#define CC_DCO1_LFO2_PWM    33  /* LFO2 -> DCO1 PWM depth              */
#define CC_DCO2_LFO2_PWM    34  /* LFO2 -> DCO2 PWM depth              */
#define CC_DCO1_ADC_PWM     35  /* DCO1 ADC PWM input depth            */

/* --- DCO2 oscillator --- */
#define CC_DCO2_SAW_LEVEL   36  /* DCO2 sawtooth level                 */
#define CC_DCO2_PULSE_WIDTH 37  /* DCO2 pulse width                    */
#define CC_DCO2_PULSE_LEVEL 38  /* DCO2 pulse level                    */
#define CC_DCO2_SUB_LEVEL   39  /* DCO2 sub level                      */
#define CC_DCO2_PWM_DEPTH   40  /* DCO2 mod wheel PWM depth            */
#define CC_DCO2_ADC_PWM     41  /* DCO2 ADC PWM input depth            */
#define CC_DCO2_DETUNE      42  /* DCO2 detune cents, centre=64        */
#define CC_DCO2_INTERVAL    43  /* DCO2 interval semitones, centre=64  */

/* --- Oscillator sync --- */
#define CC_SYNC_MODE        44  /* 0-42=off 43-84=soft 85-127=hard     */

/* --- DCO2 sweep envelope --- */
#define CC_ENV_ATTACK       45  /* attack  0=slow 127=fast             */
#define CC_ENV_DECAY        46  /* decay   0=slow 127=fast             */
#define CC_ENV_SUSTAIN      47  /* sustain level                       */
#define CC_ENV_RELEASE      48  /* release 0=slow 127=fast             */
#define CC_ENV_DEPTH        49  /* envelope -> DCO2 pitch depth        */
#define CC_ENV_DCO1_PWM     50  /* envelope -> DCO1 PWM depth          */
#define CC_ENV_DCO2_PWM     51  /* envelope -> DCO2 PWM depth          */

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
 * Aftertouch PWM CV output
 * GPIO7, ~50kHz carrier, 8-bit resolution
 * 0 pressure = 0V, 127 pressure = 3.3V
 * RC filter: 1k + 10nF -> clean 0-3.3V DC
 * -------------------------------------------------------- */
static uint8_t  atPWMSlice = 0;
static uint8_t  atPWMChan  = 0;

static void AftertouchPWM_Init(uint8_t pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    atPWMSlice = pwm_gpio_to_slice_num(pin);
    atPWMChan  = pwm_gpio_to_channel(pin);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 11.72f);
    pwm_config_set_wrap(&cfg, 255);
    pwm_init(atPWMSlice, &cfg, true);
    pwm_set_chan_level(atPWMSlice, atPWMChan, 0);
}

static void AftertouchPWM_Set(uint8_t pressure)
{
    pwm_set_chan_level(atPWMSlice, atPWMChan, (uint16_t)pressure * 2);
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
    /* oversample X-MOD to reduce noise */
    uint32_t xmodSum = 0;
    for (int i = 0; i < 16; i++) xmodSum += analogRead(ADC_XMOD_PIN);
    DCO_SetADC_XMod((uint16_t)(xmodSum >> 4));
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
            /* --- DCO1 --- */
            case CC_MOD_WHEEL:       DCO_SetModWheel(value);        break;
            case CC_PORTAMENTO_TIME: DCO_SetPortamentoRate(value);  break;
            case CC_PORTAMENTO_SW:   DCO_SetPortamento(value);      break;
            case CC_DCO1_SAW_DETUNE:      DCO_SetSawDetune(value);       break;
            case CC_DCO1_SAW_COUNT:       DCO_SetSawCount(value);        break;
            case CC_DCO1_PULSE_WIDTH:     DCO_SetPulseWidth(value);      break;
            case CC_DCO1_PWM_DEPTH:  DCO_SetPWMDepth(value);        break;
            case CC_DCO1_SAW_LEVEL:       DCO_SetSawLevel(value);        break;
            case CC_DCO1_PULSE_LEVEL:     DCO_SetPulseLevel(value);      break;
            case CC_DCO1_SUB_LEVEL:       DCO_SetSubLevel(value);        break;
            case CC_PITCHBEND_RANGE: DCO_SetPitchBendRange(value);  break;
            case CC_ADC_FM_DEPTH:    DCO_SetFMDepth(value);         break;
            case CC_XMOD_DEPTH:      DCO_SetXModDepth(value);       break;
            case CC_LFO1_RATE:        DCO_SetLFORate(value);         break;
            case CC_LFO1_WAVEFORM:    DCO_SetLFOWaveform(value);     break;
            case CC_LFO1_FM_DEPTH:    DCO_SetLFOFMDepth(value);      break;
            case CC_LFO2_RATE:       DCO_SetLFO2Rate(value);        break;
            case CC_LFO2_WAVEFORM:   DCO_SetLFO2Waveform(value);    break;
            case CC_DCO1_LFO2_PWM:   DCO_SetLFO2PWMDepth(value);    break;
            case CC_DCO2_LFO2_PWM:   DCO_SetLFO2DCO2PWMDepth(value); break;
            case CC_DCO1_ADC_PWM:    DCO_SetADCPWMDepth(value);     break;
            case CC_SYNC_MODE:       DCO_SetSyncMode(value);        break;
            case CC_ENV_ATTACK:      DCO_SetEnvAttack(value);       break;
            case CC_ENV_DECAY:       DCO_SetEnvDecay(value);        break;
            case CC_ENV_SUSTAIN:     DCO_SetEnvSustain(value);      break;
            case CC_ENV_RELEASE:     DCO_SetEnvRelease(value);      break;
            case CC_ENV_DEPTH:       DCO_SetEnvSweepDepth(value);    break;
            case CC_ENV_DCO1_PWM:    DCO_SetEnvDCO1PWMDepth(value);  break;
            case CC_ENV_DCO2_PWM:    DCO_SetEnvDCO2PWMDepth(value);  break;
            case CC_AT_FM_DEPTH:     DCO_SetAftertouchFMDepth(value);  break;
            case CC_MW_FM_DEPTH:     DCO_SetModWheelFMDepth(value);    break;
            /* DCO2 */
            case CC_DCO2_SAW_LEVEL:   DCO2_SetSawLevel(value);      break;
            case CC_DCO2_PULSE_WIDTH: DCO2_SetPulseWidth(value);    break;
            case CC_DCO2_PULSE_LEVEL: DCO2_SetPulseLevel(value);    break;
            case CC_DCO2_SUB_LEVEL:   DCO2_SetSubLevel(value);      break;
            case CC_DCO2_DETUNE:      DCO2_SetDetune(value);        break;
            case CC_DCO2_INTERVAL:    DCO2_SetInterval(value);      break;
            case CC_DCO2_PWM_DEPTH:   DCO2_SetPWMDepth(value);      break;
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
    {
        AftertouchPWM_Set(pressure);
        DCO_SetAftertouch(pressure);
    }
}

/* --------------------------------------------------------
 * PWM audio callbacks
 * Both DCOs processed together for sample-accurate sync
 * -------------------------------------------------------- */
static float dco1Buf[256];
static float dco2Buf[256];
static bool  bufsReady = false;

void PWM_CB_FillBuffer_DCO1(float *output, int len)
{
    /* process both DCOs together, store results */
    DCO_ProcessBoth(dco1Buf, dco2Buf, len);
    bufsReady = true;
    memcpy(output, dco1Buf, len * sizeof(float));
}

void PWM_CB_FillBuffer_DCO2(float *output, int len)
{
    /* DCO1 callback always runs first, just copy stored DCO2 result */
    if (bufsReady)
    {
        memcpy(output, dco2Buf, len * sizeof(float));
        bufsReady = false;
    }
    else
    {
        memset(output, 0, len * sizeof(float));
    }
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

    /* Aftertouch PWM */
    AftertouchPWM_Init(AFTERTOUCH_PWM_PIN);

    /* ADC */
    analogReadResolution(12);
    pinMode(ADC_FM_PIN,  INPUT);
    pinMode(ADC_PWM_PIN, INPUT);
    pinMode(ADC_XMOD_PIN, INPUT);

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
    DCO_SetSyncMode(0);         /* sync off by default */
    DCO_SetEnvAttack(64);       /* medium attack       */
    DCO_SetEnvDecay(64);        /* medium decay        */
    DCO_SetEnvSustain(80);      /* sustain at 63%      */
    DCO_SetEnvRelease(64);      /* medium release      */
    DCO_SetEnvSweepDepth(0);    /* DCO2 pitch sweep off    */
    DCO_SetEnvDCO1PWMDepth(0);  /* DCO1 PWM sweep off      */
    DCO_SetEnvDCO2PWMDepth(0);  /* DCO2 PWM sweep off      */
    DCO_SetPortamento(0);       /* off by default */
    DCO_SetPortamentoRate(0);   /* fastest rate   */
    DCO_SetAftertouchFMDepth(0);
    DCO_SetModWheelFMDepth(0);
    DCO_SetSawLevel(100);
    DCO_SetSawCount(20);
    DCO_SetSawDetune(0);
    DCO_SetPulseLevel(0);
    DCO_SetSubLevel(0);
    DCO_SetPulseWidth(64);
    DCO_SetPWMDepth(0);
    DCO_SetFMDepth(0);
    DCO_SetADCPWMDepth(0);      /* CC_DCO1_ADC_PWM */
    DCO_SetXModDepth(0);        /* X-MOD off until enabled */

    /* calibrate X-MOD zero point from actual DC bias voltage
     * take 256 readings with small delays for ADC to settle */
    delay(50);
    for (int i = 0; i < 256; i++)
        DCO_CalibrateXMod(analogRead(ADC_XMOD_PIN));
    DCO_SetPitchBendRange(2);
    DCO_SetLFORate(20);
    DCO_SetLFOWaveform(127);    /* sawtooth */
    DCO_SetLFOFMDepth(0);
    DCO_SetLFO2Rate(20);
    DCO_SetLFO2Waveform(0);     /* triangle */
    DCO_SetLFO2PWMDepth(0);
    DCO_SetLFO2DCO2PWMDepth(0);

    /* DCO2 defaults - silent until enabled via CC */
    DCO2_SetSawLevel(0);        /* saw off until enabled */
    DCO2_SetPulseWidth(64);     /* 50% square */
    DCO2_SetPulseLevel(0);      /* off until enabled */
    DCO2_SetSubLevel(0);        /* off until enabled */
    DCO2_SetDetune(64);         /* centre = 0 cents */
    DCO2_SetInterval(64);       /* centre = 0 semitones */
    DCO2_SetPWMDepth(0);
    /* DCO2 LFO PWM now controlled via CC_LFO2_DCO2_PWM (CC52) */
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
