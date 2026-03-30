# Dual DCO based around a Waveshare Zero stamp RP2350

I wanted to build a polysynth oscillator core with the least amount of parts and make it stable, so here is an RP2350 based dual DCO.

It can create 5 waveforms, SAW1 and super SAW1 upto 5 oscillators deep, PW1, SUB1, SAW2, PW2 and SUB2. 

The second DCO has detune and interval settings. 

There are 2 onboard LFOs that can modulate FM and the PW of either DCO, the PW can be set manually and you can modulate from the Mod Wheel FM or PWM or envelpe. 

Onboard ADSR for pitch mod of DCO2 and PWM of DCO1 & 2, full ADSR and depth.

There is pitchbend with a depth of 0-12 semitones and a glide function upto 20 seconds. 

Hard and soft sync, plus a crossmod of DCO2 to DCO1

It outputs PWM on two separate pins for DCO1 & 2, a gate signal, Velocity as PWM 0-3.3v and CV as 0.25v per octave for filter tracking etc. 

It can also receive 0-3.3v centered at 1.65v for FM and PWM modulation if you desire. 

I might build an 8 or 16 voice poly with it.

# Summary

* DCO1 — 5-voice multisaw + pulse + sub + all modulation
* DCO2 — saw + pulse + sub, ±24 semitones, ±100 cents detune
* Hard/soft oscillator sync
* ADSR sweep envelope → DCO2 pitch + DCO1/DCO2 PWM
* LFO1 → FM vibrato with aftertouch depth
* LFO2 → PWM animation independently per DCO
* X-MOD (DCO2 → DCO1 frequency)
* Portamento
* Pitchbend with configurable range
* Gate, velocity, keytrack and aftertouch CV outputs
* Full MIDI voice/control channel architecture

# Performance
* CC_MOD_WHEEL          1   /* modulation wheel                    */
* CC_PORTAMENTO_TIME    5   /* portamento rate                     */
* CC_PORTAMENTO_SW      65  /* portamento on/off (>=64=on)         */
* CC_PITCHBEND_RANGE    24  /* pitchbend range 1-12 semitones      */

# DCO1
* CC_DCO1_SAW_DETUNE  17  /* saw detune spread                   */
* CC_DCO1_SAW_COUNT   18  /* saw voice count 1-5                 */
* CC_DCO1_PULSE_WIDTH 19  /* DCO1 pulse width                    */
* CC_DCO1_PWM_DEPTH   20  /* DCO1 mod wheel PWM depth            */
* CC_DCO1_SAW_LEVEL   21  /* saw level in mix                    */
* CC_DCO1_PULSE_LEVEL 22  /* DCO1 pulse level in mix             */
* CC_DCO1_SUB_LEVEL   23  /* DCO1 sub level                      */

# LFO1

* CC_LFO1_RATE        25  /* LFO1 rate 0.1-20Hz                  */
* CC_LFO1_WAVEFORM    26  /* LFO1 waveform tri/sq/saw            */
* CC_LFO1_FM_DEPTH    27  /* LFO1 -> FM depth                    */
* CC_AT_FM_DEPTH      28  /* aftertouch -> vibrato depth         */
* CC_MW_FM_DEPTH      29  /* mod wheel -> FM depth               */
* CC_ADC_FM_DEPTH     30  /* ADC FM input depth                  */
* CC_XMOD_DEPTH       53  /* X-MOD depth DCO2->DCO1 freq         */

# LFO2

* CC_LFO2_RATE        31  /* LFO2 rate 0.1-20Hz                  */
* CC_LFO2_WAVEFORM    32  /* LFO2 waveform tri/sq/saw            */
* CC_DCO1_LFO2_PWM    33  /* LFO2 -> DCO1 PWM depth              */
* CC_DCO2_LFO2_PWM    34  /* LFO2 -> DCO2 PWM depth              */
* CC_DCO1_ADC_PWM     35  /* DCO1 ADC PWM input depth            */

# DCO2
* CC_DCO2_SAW_LEVEL   36  /* DCO2 sawtooth level                 */
* CC_DCO2_PULSE_WIDTH 37  /* DCO2 pulse width                    */
* CC_DCO2_PULSE_LEVEL 38  /* DCO2 pulse level                    */
* CC_DCO2_SUB_LEVEL   39  /* DCO2 sub level                      */
* CC_DCO2_PWM_DEPTH   40  /* DCO2 mod wheel PWM depth            */
* CC_DCO2_ADC_PWM     41  /* DCO2 ADC PWM input depth            */
* CC_DCO2_DETUNE      42  /* DCO2 detune cents, centre=64        */
* CC_DCO2_INTERVAL    43  /* DCO2 interval semitones, centre=64  */

# Sync
* CC_SYNC_MODE        44  /* 0-42=off 43-84=soft 85-127=hard     */

# ADSR
* CC_ENV_ATTACK       45  /* attack  0=slow 127=fast             */
* CC_ENV_DECAY        46  /* decay   0=slow 127=fast             */
* CC_ENV_SUSTAIN      47  /* sustain level                       */
* CC_ENV_RELEASE      48  /* release 0=slow 127=fast             */
* CC_ENV_DEPTH        49  /* envelope -> DCO2 pitch depth        */
* CC_ENV_DCO1_PWM     50  /* envelope -> DCO1 PWM depth          */
* CC_ENV_DCO2_PWM     51  /* envelope -> DCO2 PWM depth          */
