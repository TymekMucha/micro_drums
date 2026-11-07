#include "TriggerSettingsStore.h"

TriggerSettingsStore::TriggerSettingsStore()
{
    for(auto& arr : m_settingsOfAllPads)
    {
        for(int j = 0; j < arr.size(); j++)
        {
            arr.at(j) = TriggerSetting(cfg::SettingsDefaults[j], cfg::SettingStrings[j]);
        }
    }
}

TriggerSetting &TriggerSettingsStore::GetSettingRef(int padIndex, int settingIndex)
{
    return m_settingsOfAllPads.at(padIndex).at(settingIndex);
}

