#pragma once

#include <array>
#include <cstdint>
#include <stdint.h>
#include <inttypes.h>
#include <string>
#include "config.h"

extern "C"{
    #include "ssd1306/ssd1306.h"
}

class Display
{
private:
    ssd1306_t oled;

public:
    Display();

    bool Init();
    void DrawTest();
    void DrawSetting(std::string settingString, int settingValue, std::string padName);
    void DrawPadSelection(int currentlySelectedPad);
};