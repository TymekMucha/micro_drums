#include "Button.h"

Button::Button(int gpio, ButtonEvent event)
    : m_gpio(gpio), m_setEvent(event), m_currentEvent(ButtonEvent::None),
      m_lastRaw(false), m_lastChangeMs(0), m_stableState(false)
{
}

void Button::Init()
{
    gpio_init(m_gpio);
    gpio_set_dir(m_gpio, GPIO_IN);
    gpio_pull_up(m_gpio);
}

void Button::Scan()
{
    const bool pressed = !gpio_get(m_gpio);          // active low
    const uint32_t now = to_ms_since_boot(get_absolute_time());

    if (pressed != m_lastRaw)
    {
        m_lastRaw = pressed;
        m_lastChangeMs = now;
        return;
    }

    if (pressed != m_stableState && (now - m_lastChangeMs) >= cfg::DebounceMS)
    {
        m_stableState = pressed;
        if (m_stableState)                            // fire on press only
            m_currentEvent = m_setEvent;
    }
}

ButtonEvent Button::CheckEvent()
{
    ButtonEvent event = m_currentEvent;
    m_currentEvent = ButtonEvent::None;
    return event;
}
