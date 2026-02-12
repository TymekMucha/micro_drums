#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

#include "tusb.h"
#include "bsp/board.h"


#define PIEZO_ADC_CHANNEL 0
#define PIEZO_GPIO 26
#define THRESHOLD 1600
#define RETRIGGER_MS 90

#define LED_PIN 15

#define BTN_PIN 11
#define DEBOUNCE_MS 5

#define MIDI_CHANNEL  0   
#define MIDI_NOTE     38  
#define MIDI_VELOCITY 127


bool debounce_button() {
    if (gpio_get(BTN_PIN) == 0) {
        sleep_ms(DEBOUNCE_MS);
        return gpio_get(BTN_PIN) == 0;
    }
    return false;
}

uint8_t read_piezo(){
    static uint32_t last_hit_time = 0;
    uint16_t value = adc_read();

    if(value < THRESHOLD) {
        return 0;
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());
    if((now - last_hit_time) < RETRIGGER_MS) return 0;

    last_hit_time = now;

    uint32_t clamped = value - THRESHOLD;
    uint32_t range   = 4095 - THRESHOLD;
    uint8_t velocity = (uint8_t)(1 + (clamped * 126) / range) + 90;

    //return velocity;
    return MIDI_VELOCITY;
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

    // Enable external LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Enable Button
    gpio_init(BTN_PIN);
    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_pull_up(BTN_PIN);

    // Enable Piezo ADC
    adc_init();
    adc_gpio_init(PIEZO_GPIO);
    adc_select_input(PIEZO_ADC_CHANNEL);
    
    // Enable PWM for onboard LED
    gpio_set_function(PICO_DEFAULT_LED_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PICO_DEFAULT_LED_PIN);
    pwm_set_enabled(slice_num, true);

    uint8_t led_level = 0;
    
    bool last_button_state = false;

    while (true) {
        tud_task();

        bool pressed = debounce_button();

        if (pressed && !last_button_state) {
            send_midi_note_on(MIDI_CHANNEL, MIDI_NOTE, MIDI_VELOCITY);
        } 

        last_button_state = pressed;

        if(gpio_get(BTN_PIN) == 0)
        {
            printf("CLICK");
            gpio_put(LED_PIN, 1);
        }
        else
        {
            gpio_put(LED_PIN, 0);
        }


        // Piezo reading
        uint8_t velocity = read_piezo();
        if(velocity > 0)
        {
            send_midi_note_on(MIDI_CHANNEL, MIDI_NOTE, velocity);
        }
    }
    
    return 0;
}