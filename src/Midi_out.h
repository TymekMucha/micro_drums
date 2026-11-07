#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <cstdint>

namespace midi {

void noteOn(uint8_t channel, uint8_t note, uint8_t velocity);
void noteOff(uint8_t channel, uint8_t note);

}  // namespace midi