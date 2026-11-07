#include "MenuContoller.h"

void MenuContoller::HandleEvents()
{
    while (!m_eventQueue.empty())
    {
        ButtonEvent event = m_eventQueue.front();
        switch (event)
        {
        case ButtonEvent::Right:
            OnRightButtonClick();
            break;

        case ButtonEvent::Left:
            OnLeftButtonClick();
            break;

        case ButtonEvent::Plus:
            OnPlusButtonClick();
            break;

        case ButtonEvent::Minus:
            OnMinusButtonClick();
            break;

        default:
            break;
        }
        m_eventQueue.pop();
        Draw();
    }
}

MenuContoller::MenuContoller(TriggerSettingsStore &triggerSettingsStoreRef) : m_buttonArray{Button(cfg::LeftButtonGpio, ButtonEvent::Left),
                                                                                            Button(cfg::RightButtonGpio, ButtonEvent::Right),
                                                                                            Button(cfg::PlusButtonGpio, ButtonEvent::Plus),
                                                                                            Button(cfg::MinusButtonGpio, ButtonEvent::Minus)},
                                                                              m_triggerSettingsStoreRef(triggerSettingsStoreRef)
{
    m_currentlySelectedPad = 0;
    m_currentlySelectedSetting = 0;
    m_display = Display();
}

void MenuContoller::Init()
{
    m_display.Init();
    for (Button &button : m_buttonArray)
    {
        button.Init();
    }
    Draw();
}

void MenuContoller::Draw()
{
    if(m_currentlySelectedSetting < cfg::SettingsAmount)
    {
        TriggerSetting &currentSetting = m_triggerSettingsStoreRef.GetSettingRef(m_currentlySelectedPad, m_currentlySelectedSetting);
        m_display.DrawSetting(currentSetting.GetName(), currentSetting.GetValue(), cfg::PadNames[m_currentlySelectedPad]);
    }
    else // this is a weird approsch since the pad selection risks with adressing the array out of bounds...
    {
        m_display.DrawPadSelection(m_currentlySelectedPad);
    }

}

void MenuContoller::Update()
{
    for (Button &button : m_buttonArray)
    {
        button.Scan();
        ButtonEvent event = button.CheckEvent();
        if (event != ButtonEvent::None)
        {
            m_eventQueue.emplace(event);
        }
    }
    HandleEvents();
}

void MenuContoller::OnLeftButtonClick()
{
    m_currentlySelectedSetting--;
    if (m_currentlySelectedSetting < 0)
    {
        m_currentlySelectedSetting = cfg::SettingsAmount;
    }
}

void MenuContoller::OnRightButtonClick()
{
    m_currentlySelectedSetting++;
    if (m_currentlySelectedSetting >= cfg::SettingsAmount + 1)
    {
        m_currentlySelectedSetting = 0;
    }
}

void MenuContoller::OnPlusButtonClick()
{
    if (m_currentlySelectedSetting == cfg::SettingsAmount)
    {
        if (m_currentlySelectedPad++ >= cfg::NumPads - 1)
        {
            m_currentlySelectedPad = 0;
        }
    }
    else
    {
        m_triggerSettingsStoreRef.GetSettingRef(m_currentlySelectedPad, m_currentlySelectedSetting).IncrementValue();
    }
}

void MenuContoller::OnMinusButtonClick()
{
    if (m_currentlySelectedSetting == cfg::SettingsAmount)
    {
        if (m_currentlySelectedPad-- <= 0)
        {
            m_currentlySelectedPad = cfg::NumPads - 1;
        }
    }
    else
    {
        m_triggerSettingsStoreRef.GetSettingRef(m_currentlySelectedPad, m_currentlySelectedSetting).DecrementValue();
    }
}