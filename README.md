I wanted to build a polysynth oscillator core with the least amount of parts and make it stable, so here is an RP2350 based dual DCO.

It can create 5 waveforms, saw and super saw upto 5 oscillators deep, PW1, SUB1, PW2 and SUB2. 

The second DCO has detune and interval settings. 

Their is an onboard LFO that can modulate FM or the PW of either DCO, the PW can be set manually and you can modulate from the Mod Wheel FM or PWM. 

There is pitchbend with a depth of 0-12 semitones and a glide function upto 20 seconds. 

It outputs PWM on two separate pins for DCO1 & 2, a gate signal, Velocity as PWM 0-3.3v and CV as 0.25v per octave for filter tracking etc. 

It can also receive 0-3.3v centered at 1.65v for FM and PWM modulation if you desire. 

Ive fixed a bug with the LFO and added portamento and soft/hard sync with an onboard Envelope to sweep DCO2

I might build an 8 or 16 voice poly with it.

# DCO1
* CC_MOD_WHEEL        1
* CC_PORTAMENTO_TIME  5   /* portamento rate                     */
* CC_PORTAMENTO_SW    65  /* portamento on/off (>=64 on, <64 off) */
* CC_SAW_DETUNE       17
* CC_SAW_COUNT        18
* CC_PULSE_WIDTH      19
* CC_PWM_DEPTH        20
* CC_SAW_LEVEL        21
* CC_PULSE_LEVEL      22
* CC_SUB_LEVEL        23

# LFO etc
* CC_PITCHBEND_RANGE  24
* CC_FM_DEPTH         25
* CC_LFO_RATE         26
* CC_LFO_WAVEFORM     27
* CC_LFO_FM_DEPTH     28
* CC_LFO_PWM_DEPTH    29
* CC_ADC_PWM_DEPTH    30
* CC_AT_FM_DEPTH      31  /* aftertouch -> FM depth               */
* CC_MW_FM_DEPTH      43  /* mod wheel -> FM depth                */

# DCO2
* CC_DCO2_PULSE_WIDTH 33
* CC_DCO2_PULSE_LEVEL 34
* CC_DCO2_SUB_LEVEL   35
* CC_DCO2_DETUNE      36  /* centre=64 -> 0 cents, range ±100  */
* CC_DCO2_INTERVAL    37  /* centre=64 -> 0 semitones, range ±24 */
* CC_DCO2_PWM_DEPTH   38  /* mod wheel PWM depth                 */
* CC_DCO2_LFO_PWM     39  /* LFO -> PWM depth                    */
* CC_DCO2_ADC_PWM     40  /* ADC PWM input depth                 */

# SYNC and ADSR
* CC_SYNC_MODE        41  /* sync: 0-42=off 43-84=soft 85-127=hard */
* CC_ENV_ATTACK       44  /* DCO2 sweep envelope attack              */
* CC_ENV_DECAY        45  /* DCO2 sweep envelope decay               */
* CC_ENV_SUSTAIN      46  /* DCO2 sweep envelope sustain level       */
* CC_ENV_RELEASE      47  /* DCO2 sweep envelope release             */
* CC_ENV_DEPTH        48  /* DCO2 sweep envelope depth (semitones)   */
