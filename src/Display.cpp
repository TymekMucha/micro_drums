#include "Display.h"

Display::Display()
{
i2c_init(i2c1, 400000);
gpio_pull_up(cfg::OledSclGpio);
gpio_pull_up(cfg::OledSdaGpio);
gpio_set_function(cfg::OledSclGpio, GPIO_FUNC_I2C);
gpio_set_function(cfg::OledSdaGpio, GPIO_FUNC_I2C);

oled.external_vcc=false;
}

bool Display::Init()
{
    return ssd1306_init(&oled, 128, 64, 0x3c, i2c1);
}

void Display::DrawTest()
{
    ssd1306_clear(&oled);
    ssd1306_draw_string(&oled, 0, 2, 2, "Hello");
    ssd1306_draw_line(&oled, 2, 25, 80, 25);
    ssd1306_show(&oled);
}

void Display::DrawSetting(std::string settingString, int settingValue, std::string padName)
{
    ssd1306_clear(&oled);
    ssd1306_draw_string(&oled, 0, 2, 2, settingString.c_str());
    ssd1306_draw_string(&oled, 0, 20, 2, std::to_string(settingValue).c_str());
    ssd1306_draw_string(&oled, 100, 20, 1, padName.c_str());
    ssd1306_show(&oled);
}

void Display::DrawPadSelection(int currentlySelectedPad)
{
    ssd1306_clear(&oled);
    ssd1306_draw_string(&oled, 0, 2, 2, cfg::PadNames[currentlySelectedPad]);
    ssd1306_show(&oled);
}
