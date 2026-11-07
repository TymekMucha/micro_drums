#pragma once
#include <array>
#include <cstdint>
#include "config.h"
#include "TriggerSettingsStore.h"

// One trigger input with its hit-detection state machine.
class Trigger {
public:
    Trigger(uint8_t gpio, uint8_t adcChannel, TriggerSettingsStore& settingsStore, int midiNote, int index);

    void init();   // configure the GPIO for ADC use and reset state
    void scan();   // take one reading and advance the state machine

private:
    enum class State { Idle, PeakTracking, NoteFired };
    
    int m_midiNote;

    uint16_t readAveraged() const;
    uint8_t  toVelocity(uint16_t peak);
    void     beginTracking(uint16_t value, uint32_t now);

    int readSetting(cfg::SettingEnum settingEnum) const;

    uint8_t gpio;
    uint8_t adcChannel;
    TriggerSettingsStore& m_settingsStore; // live reference: UI edits take effect immediately

    const int m_index; // index used for retrieving correct settings

    State    state           = State::Idle;
    uint32_t windowStartMs   = 0;
    uint32_t noteFiredMs     = 0;
    uint16_t peakVal         = 0;
    uint16_t lastFiredPeak   = 0;
};
