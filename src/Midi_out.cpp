#include "midi_out.h"
#include "tusb.h"

namespace midi {

void noteOn(uint8_t channel, uint8_t note, uint8_t velocity) {
    const uint8_t msg[3] = { uint8_t(0x90 | (channel & 0x0F)),
                             uint8_t(note & 0x7F),
                             uint8_t(velocity & 0x7F) };
    tud_midi_stream_write(0, msg, 3);
}

void noteOff(uint8_t channel, uint8_t note) {
    const uint8_t msg[3] = { uint8_t(0x80 | (channel & 0x0F)),
                             uint8_t(note & 0x7F),
                             0 };
    tud_midi_stream_write(0, msg, 3);
}

}  // namespace midi