# Micro Drums

A homemade custom electronic drum kit trigger that send a MIDI signal to any connected sampler via USB. Just some piezo sensors connected to a Raspberry Pi Pico really.

## Goals
- Convert piezo hits into velocity-sensitive drum triggers
- Send MIDI signals to a DAW or drum software
- Build a full playable kit

---

# Versions

---

## v0.1 — Project Start

![v0.1]
**Date:** 2026.03.08

<img width="2048" height="1536" alt="image" src="https://github.com/user-attachments/assets/21a78068-1225-4eae-8c0f-fbdcbaceaecc" />


### Description
Initial proof of concept and test of piezo sensors. Basic trigger detection on mutliple channels at the same time. 

### Notes
- Latency very tolerable and probably mostly caused by third party sampler.
- Piezos required voltage clamp, because the spike would sometimes get over 3.3V and overlfow the ADC on the Pico.
- Need to assemble proper drum pads now.
- Pico has only 3 ADC channels so an external ADC is required for further expansion.

