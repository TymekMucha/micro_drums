#pragma once
#include "pico/stdlib.h"
#include "pico/time.h"
#include "Config.h"

enum class ButtonEvent {None, Left, Right, Plus, Minus};

class Button
{
private:
    int m_gpio;
    const ButtonEvent m_setEvent;
    ButtonEvent m_currentEvent;
    bool m_lastRaw;
    uint32_t m_lastChangeMs;
    bool m_stableState;

public:
    Button();
    Button(int gpio, ButtonEvent event);
    
    void Init();
    void Scan();
    ButtonEvent CheckEvent();
};