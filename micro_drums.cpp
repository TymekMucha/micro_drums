#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "tusb.h"
#include "bsp/board.h"
#include "config.h"

#define THRESHOLD           300
#define PEAK_WINDOW_MS      7      
#define DECAY_MS            100     
#define RETRIGGER_MS        15     
#define ADC_OVERSAMPLE      4      

// Re-trigger if a new hit during FIRED state exceeds the last peak by this much.
#define RETRIGGER_THRESHOLD_RATIO  0.6f

#define VEL_ADC_MIN         300
#define VEL_ADC_MAX         4000

#define MIDI_CHANNEL        0
#define MIDI_SNARE          38
#define MIDI_HH             42
#define MIDI_KICK           36

typedef enum {
    STATE_IDLE,
    STATE_PEAK_TRACKING,
    STATE_NOTE_FIRED,
} HitState;

typedef struct {
    HitState  state;
    uint32_t  window_start_ms;
    uint32_t  note_fired_ms;
    uint16_t  peak_val;
    uint16_t  last_fired_peak; 
    uint8_t   midi_note;
} TriggerState;

uint16_t adc_read_averaged(uint8_t channel) {
    adc_select_input(channel);
    uint32_t sum = 0;
    for (int i = 0; i < ADC_OVERSAMPLE; i++) {
        sum += adc_read();
    }
    return (uint16_t)(sum / ADC_OVERSAMPLE);
}

uint8_t adc_to_velocity(uint16_t peak) {
    if (peak <= VEL_ADC_MIN) return 1;
    if (peak >= VEL_ADC_MAX) return 127;
    float normalized = (float)(peak - VEL_ADC_MIN) / (float)(VEL_ADC_MAX - VEL_ADC_MIN);
    float curved     = sqrtf(normalized);
    return (uint8_t)(1 + normalized * 126.0f);
}

void send_midi_note_on(uint8_t channel, uint8_t note, uint8_t velocity) {
    uint8_t msg[3] = { 0x90 | (channel & 0x0F), note & 0x7F, velocity & 0x7F };
    tud_midi_stream_write(0, msg, 3);
}

void send_midi_note_off(uint8_t channel, uint8_t note) {
    uint8_t msg[3] = { 0x80 | (channel & 0x0F), note & 0x7F, 0 };
    tud_midi_stream_write(0, msg, 3);
}

static inline void begin_tracking(TriggerState *s, uint16_t value, uint32_t now) {
    s->state           = STATE_PEAK_TRACKING;
    s->window_start_ms = now;
    s->peak_val        = value;
}

void process_pad(uint8_t adc_channel, TriggerState *s) {
    uint32_t now   = to_ms_since_boot(get_absolute_time());
    uint16_t value = adc_read_averaged(adc_channel);

    switch (s->state) {

        case STATE_IDLE:
            if (value >= THRESHOLD) {
                begin_tracking(s, value, now);
            }
            break;

        case STATE_PEAK_TRACKING:
            if (value > s->peak_val) {
                s->peak_val = value;
            }

            if ((now - s->window_start_ms) >= PEAK_WINDOW_MS) {
                // Fire the note
                uint8_t velocity   = adc_to_velocity(s->peak_val);
                send_midi_note_on(MIDI_CHANNEL, s->midi_note, velocity);
                s->last_fired_peak = s->peak_val;
                s->note_fired_ms   = now;
                s->state           = STATE_NOTE_FIRED;
            }
            break;

        case STATE_NOTE_FIRED:
            // Send note off once delay has passed
            if ((now - s->note_fired_ms) >= DECAY_MS) {
                send_midi_note_off(MIDI_CHANNEL, s->midi_note);
                s->state    = STATE_IDLE;
                s->peak_val = 0;
                break;
            }

            // If the signal spikes significantly above the ringing/decay noise
            // from the last hit, treat it as a new hit immediately.
            if ((now - s->note_fired_ms) >= RETRIGGER_MS) {
                float retrigger_min = s->last_fired_peak * RETRIGGER_THRESHOLD_RATIO;
                if (value >= THRESHOLD && value >= (uint16_t)retrigger_min) {
                    send_midi_note_off(MIDI_CHANNEL, s->midi_note); // ensure off first
                    begin_tracking(s, value, now);
                    break;
                }
            }
    }
}

int main() {
    board_init();
    tusb_init();
    stdio_init_all();

    adc_init();
    adc_gpio_init(TRIGGER_GPIO0);
    adc_gpio_init(TRIGGER_GPIO1);
    adc_gpio_init(TRIGGER_GPIO2);

    TriggerState pads[3] = {
        { STATE_IDLE, 0, 0, 0, 0, MIDI_SNARE },
        { STATE_IDLE, 0, 0, 0, 0, MIDI_HH    },
        { STATE_IDLE, 0, 0, 0, 0, MIDI_KICK  },
    };

    uint8_t channels[3] = {
        TRIGGER_ADC_CHANNEL0,
        TRIGGER_ADC_CHANNEL1,
        TRIGGER_ADC_CHANNEL2,
    };

    while (true) {
        tud_task();
        for (int i = 0; i < 3; i++) {
            process_pad(channels[i], &pads[i]);
        }
    }

    return 0;
}