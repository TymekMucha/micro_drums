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

<img src="https://github.com/user-attachments/assets/21a78068-1225-4eae-8c0f-fbdcbaceaecc" width='400'>


### Description
Initial proof of concept and test of piezo sensors. Basic trigger detection on mutliple channels at the same time. 

### Notes
- Latency very tolerable and probably mostly caused by third party sampler.
- The Piezos required a voltage clamp, because the spike would sometimes get over 3.3V and overlfow the ADC on the Pico.
- Need to assemble proper drum pads now.
- The Pico has only 3 ADC channels so an external ADC is required for further expansion.

## v0.2 — Pad Prototype

### Description
Tested the simplest design of a pad prototype, consisting of flat layers:
- Rubber
- Aluminium sheet
- Piezo
- Mouse pad rubber
- Wooden base

<img src="https://github.com/user-attachments/assets/c466e639-2c0b-497f-836b-bc9442df4cab" width="400">


After connecting the pad to the Pico and the Pico to a MIDI player, the note velocity seemed to be inconsistent. To test whether this was a software problem, I made a simple visualizer of the raw data that the Pico reads from the piezo.

<img src="https://github.com/user-attachments/assets/e88c393e-41d5-4683-a052-b0f8cdbb0cab" width='800'>

This chart shows the raw data from a sample recording consisting of a series of 4 soft, 4 hard, 2 soft, 2 hard, 1 soft, 1 hard, 1 soft, 1 hard hits. It's clear that the signal from the piezo itself is inconsistent when it comes to differentiating hit strength, so the problem is hardware-based. The pad probably requires a more complex or more precise design, likely using a cone pushing on the piezo instead of the aluminium sheet.

### Notes
- Pad design needs revision for better consistency.
- Testing the Pico with a real edrum pad would provide a good benchmark.
- Further prototypes will probably require a cone that transfers the impact from the rubber to the Pico
- Schoemaker's glue turned out to be a very well suited adhesive for building the prototype. It's very strong but also not brittle.

