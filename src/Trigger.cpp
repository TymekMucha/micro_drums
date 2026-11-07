#include "Trigger.h"

#include <utility>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "midi_out.h"

// ---------------- TriggerPad ----------------

Trigger::Trigger(uint8_t gpio, uint8_t adcChannel, TriggerSettingsStore& settingsStore, int midiNote, int index)
    : gpio(gpio), adcChannel(adcChannel), m_settingsStore(settingsStore), m_midiNote(midiNote), m_index(index) {}

void Trigger::init() {
    adc_gpio_init(gpio);
    state = State::Idle;
    peakVal = 0;
    lastFiredPeak = 0;
}

uint16_t Trigger::readAveraged() const {
    adc_select_input(adcChannel);
    uint32_t sum = 0;
    for (int i = 0; i < cfg::AdcOversample; i++) {
        sum += adc_read();
    }
    return static_cast<uint16_t>(sum / cfg::AdcOversample);
}

uint8_t Trigger::toVelocity(uint16_t peak) {
    if (peak <= readSetting(cfg::SettingEnum::VEL_ADC_MIN)) return 1;
    if (peak >= readSetting(cfg::SettingEnum::VEL_ADC_MAX)) return 127;
    const float normalized = static_cast<float>(peak - readSetting(cfg::SettingEnum::VEL_ADC_MIN)) /
                             static_cast<float>(readSetting(cfg::SettingEnum::VEL_ADC_MAX) - readSetting(cfg::SettingEnum::VEL_ADC_MIN));
    return static_cast<uint8_t>(1 + normalized * 126.0f);
}

void Trigger::beginTracking(uint16_t value, uint32_t now) {
    state         = State::PeakTracking;
    windowStartMs = now;
    peakVal       = value;
}

int Trigger::readSetting(cfg::SettingEnum settingEnum) const
{
    return m_settingsStore.GetSettingRef(m_index, settingEnum).GetValue();
}

void Trigger::scan() {
    const uint32_t now   = to_ms_since_boot(get_absolute_time());
    const uint16_t value = readAveraged();

    switch (state) {

    case State::Idle:
        if (value >= readSetting(cfg::SettingEnum::THRESHOLD)) {
            beginTracking(value, now);
        }
        break;

    case State::PeakTracking:
        if (value > peakVal) {
            peakVal = value;
        }

        if ((now - windowStartMs) >= readSetting(cfg::SettingEnum::PEAK_WINDOW_MS)) {
            midi::noteOn(cfg::MidiChannel, m_midiNote, toVelocity(peakVal));
            lastFiredPeak = peakVal;
            noteFiredMs   = now;
            state         = State::NoteFired;
        }
        break;

    case State::NoteFired:
        if ((now - noteFiredMs) >= readSetting(cfg::SettingEnum::DECAY_MS)) {
            midi::noteOff(cfg::MidiChannel, m_midiNote);
            state   = State::Idle;
            peakVal = 0;
            break;
        }

        // A spike well above the ringing from the last hit = new hit.
        if ((now - noteFiredMs) >= readSetting(cfg::SettingEnum::RETRIGGER_MS)) {
            const float retriggerMin = lastFiredPeak * (readSetting(cfg::SettingEnum::RETRIGGER_RATIO_PROC) / 100.0f);
            if (value >= readSetting(cfg::SettingEnum::THRESHOLD) && value >= static_cast<uint16_t>(retriggerMin)) {
                midi::noteOff(cfg::MidiChannel, m_midiNote);
                beginTracking(value, now);
            }
        }
        break;
    }
}
