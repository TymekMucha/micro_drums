#pragma once
#include <string>
#include <array>
#include "Config.h"
#include "TriggerSetting.h"

class TriggerSettingsStore
{
private:
     std::array<std::array<TriggerSetting, cfg::SettingsAmount>, cfg::NumPads> m_settingsOfAllPads;

public:
     TriggerSettingsStore();

     TriggerSetting& GetSettingRef(int padIndex, int settingIndex) ;
};