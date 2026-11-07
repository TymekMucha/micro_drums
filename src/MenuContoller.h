#pragma once
#include <queue>
#include <deque>
#include "Config.h"
#include "Button.h"
#include "Display.h"
#include "TriggerSettingsStore.h"

class MenuContoller
{
private:
    Display m_display;
    std::array<Button, cfg::ButtonAmount> m_buttonArray;
    TriggerSettingsStore& m_triggerSettingsStoreRef;

    int m_currentlySelectedPad;
    int m_currentlySelectedSetting;

    std::queue<ButtonEvent, std::deque<ButtonEvent>> m_eventQueue;

    void HandleEvents();
    
public:
    MenuContoller(TriggerSettingsStore& triggerSettingsStoreRef);
    void Init();
    void Draw();
    void Update();

    void OnLeftButtonClick();
    void OnRightButtonClick();
    void OnPlusButtonClick();
    void OnMinusButtonClick();
};


