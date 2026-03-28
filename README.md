I wanted to build a polysynth oscillator core with the least amount of parts and make it stable, so here is an RP2350 based dual DCO.

It can create 5 waveforms, saw and super saw upto 5 oscillators deep, PW1, SUB1, PW2 and SUB2. 

The second DCO has detune and interval settings. 

Their is an onboard LFO that can modulate FM or the PW of either DCO, the PW can be set manually and you can modulate from the Mod Wheel FM or PWM. 

There is pitchbend with a depth of 0-12 semitones and a glide function upto 20 seconds. 

It outputs PWM on two separate pins for DCO1 & 2, a gate signal, Velocity as PWM 0-3.3v and CV as 0.25v per octave for filter tracking etc. 

It can also receive 0-3.3v centered at 1.65v for FM and PWM modulation if you desire. 

It's a bit rough and ready as it took me about 3 hours to create, but it seems to work ok. 

I might build an 8 or 16 voice poly with it.
