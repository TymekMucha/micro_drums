#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "bsp/board.h"
#include "tusb.h"
#include "Config.h"
#include "Trigger.h"
#include "Display.h"
#include "TriggerSettingsStore.h"
#include "MenuContoller.h"

int main()
{

    board_init();
    tusb_init();
    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    adc_init();

    TriggerSettingsStore triggerSettingsStore;

    std::array<Trigger, cfg::NumPads> triggers{
        Trigger(cfg::TriggerGpio[0], cfg::TriggerAdcChannel[0], triggerSettingsStore, cfg::kMidiKick, 0),
        Trigger(cfg::TriggerGpio[1], cfg::TriggerAdcChannel[1], triggerSettingsStore, cfg::kMidiSnare, 1),
        Trigger(cfg::TriggerGpio[2], cfg::TriggerAdcChannel[2], triggerSettingsStore, cfg::kMidiHiHat, 2),
    };

    for (Trigger& trigger : triggers)
    {
        trigger.init();
    }

    MenuContoller menuController(triggerSettingsStore);
    menuController.Init();

    while (true)
    {
        tud_task();
        for (Trigger& trigger : triggers)
        {
            trigger.scan();
        }
        menuController.Update();
    }
}