#include "TriggerSettingsStore.h"

TriggerSettingsStore::TriggerSettingsStore()
{
    for(int i = 0; i < cfg::NumPads; i++)
    {
        for(int j = 0; j < cfg::SettingsAmount; j++)
        {
            m_settingsOfAllPads.at(i).at(j) = TriggerSetting(cfg::SettingsDefaults[i][j], cfg::SettingStrings[j]);
        }
    }
}

TriggerSetting &TriggerSettingsStore::GetSettingRef(int padIndex, int settingIndex)
{
    return m_settingsOfAllPads.at(padIndex).at(settingIndex);
}

