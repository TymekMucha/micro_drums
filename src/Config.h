#pragma once
#include <cstdint>
#include <array>
#include <string_view>

namespace cfg {

// ---- Trigger inputs ----
constexpr int     NumPads        = 3;
constexpr uint8_t TriggerGpio[NumPads]       = { 26, 27, 28 };
constexpr uint8_t TriggerAdcChannel[NumPads] = { 0, 1, 2 };
constexpr int     AdcOversample  = 4;

// ---- MIDI ----
constexpr uint8_t MidiChannel    = 0;

constexpr int kMidiKick  = 36;
constexpr int kMidiSnare = 38;
constexpr int kMidiHiHat = 42;

// ---- Buttons (active low, internal pull-ups) ----
constexpr uint8_t LeftButtonGpio  = 21;
constexpr uint8_t RightButtonGpio = 20;
constexpr uint8_t MinusButtonGpio = 19;
constexpr uint8_t PlusButtonGpio  = 18;

constexpr int ButtonAmount = 4;

constexpr int DebounceMS = 20;

// ---- OLED (SSD1306 over I2C1) ----
constexpr uint8_t OledSclGpio = 15;
constexpr uint8_t OledSdaGpio = 14;

// ---- SETTINGS ----

constexpr int SettingsAmount = 8;
constexpr std::string_view SettingStrings[SettingsAmount] = 
{
    "threshold",            // ADC counts needed to start tracking a hit 
    "peakWindowMs",         // "scan time": how long to look for the peak
    "decayMs",              // time before note-off / back to idle       
    "retrig_Ms",            // minimum time before a retrigger is allowed
    "retrig_%",             // new hit must reach this % of last peak    
    "velAdcMin",            // ADC value mapped to velocity 1            
    "velAdcMax",            // ADC value mapped to velocity 127       
    "velOffset"             // value by how much the velocity is shifted
};
constexpr int SettingsDefaults[NumPads][SettingsAmount] =
{
    { // kick
        300,
        7,
        100,
        15,
        60,
        300,
        4000,
        29
    },
    { // snare
        300,
        7,
        100,
        7,
        40,
        300,
        4000,
        0
    },
    { // hh
        150,
        7,
        100,
        15,
        35,
        300,
        4000,
        30
    }
};

enum SettingEnum{
    THRESHOLD,
    PEAK_WINDOW_MS,
    DECAY_MS,
    RETRIGGER_MS,
    RETRIGGER_RATIO_PROC,
    VEL_ADC_MIN,
    VEL_ADC_MAX,
    VEL_OFFSET
};

constexpr const char* PadNames[NumPads] =
{
    "Kick",
    "Snare",
    "HH"
};

}  // namespace cfg