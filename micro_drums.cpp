#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

#include "tusb.h"
#include "bsp/board.h"


#define PIEZO_ADC_CHANNEL0 0
#define PIEZO_ADC_CHANNEL1 1
#define PIEZO_ADC_CHANNEL2 2
#define PIEZO_GPIO0 26
#define PIEZO_GPIO1 27
#define PIEZO_GPIO2 28
#define THRESHOLD 500
#define RETRIGGER_MS 88

#define MIDI_CHANNEL  0   
#define MIDI_SNARE     38  
#define MIDI_HH     42  
#define MIDI_KICK     36  
#define MIDI_VELOCITY 127

typedef struct {
    uint32_t last_hit_time;
    uint16_t peak_val;
    bool hit_active;
} PiezoState;

uint8_t read_piezo(PiezoState *state) {
    uint16_t value = adc_read();

    if (value < THRESHOLD) {
        return 0;
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());
    if ((now - state->last_hit_time) < RETRIGGER_MS) {
        return 0;
    }

    // Detect hit start
    if (!state->hit_active) {
        if (value >= THRESHOLD) {
            state->hit_active = true;
            state->peak_val = value;
        }
        return 0;
    }
    
    // Track peak while rising
    if (state->peak_val < value) {
        state->peak_val = value;
        return 0;
    }

    uint8_t velocity = state->peak_val / 32;

    state->last_hit_time = now;
    state->peak_val = 0;
    state->hit_active = false;

    return velocity;
}


void send_midi_note_on(uint8_t channel, uint8_t note, uint8_t velocity) {
    // TinyUSB sends MIDI in 4-byte USB-MIDI packets
    // [cable_num << 4 | code_index, status, note, velocity]
    uint8_t msg[3] = {
        0x90 | channel,  // Note On status byte
        note,
        velocity
    };
    tud_midi_stream_write(0, msg, 3);
}

void send_midi_note_off(uint8_t channel, uint8_t note) {
    uint8_t msg[3] = {
        0x80 | channel,  
        note,
        0  
    };
    tud_midi_stream_write(0, msg, 3);
}

int main() {

    // Enable MIDI
    board_init();
    tusb_init();
    stdio_init_all();

    // Enable Piezo ADC
    stdio_init_all();

    adc_init();
    adc_gpio_init(PIEZO_GPIO0);
    adc_gpio_init(PIEZO_GPIO1);
    adc_gpio_init(PIEZO_GPIO2);

    PiezoState state0 = {0, 0, false};
    PiezoState state1 = {0, 0, false};
    PiezoState state2 = {0, 0, false};

    while (true) {
        tud_task();

        // Piezo reading
        uint8_t velocity;

        // Read channel 0
        adc_select_input(PIEZO_ADC_CHANNEL0);
        velocity = read_piezo(&state0);
        if(velocity)
        {
            send_midi_note_on(MIDI_CHANNEL, MIDI_SNARE, velocity);
        }

        // Read channel 1
        adc_select_input(PIEZO_ADC_CHANNEL1);
        velocity = read_piezo(&state1);
        if(velocity)
        {
            send_midi_note_on(MIDI_CHANNEL, MIDI_HH, velocity);
        }

        // Read channel 2
        adc_select_input(PIEZO_ADC_CHANNEL2);
        velocity = read_piezo(&state2);
        if(velocity)
        {
            send_midi_note_on(MIDI_CHANNEL, MIDI_KICK, velocity);
        }
    }
    
    return 0;
}