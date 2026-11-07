#include "TriggerSetting.h"

TriggerSetting::TriggerSetting(int defaultValue, std::string_view name) : m_value(defaultValue), m_name(name)
{
}

TriggerSetting::TriggerSetting()
{
}

int TriggerSetting::GetValue() const
{
    return m_value;
}

const std::string TriggerSetting::GetName() const
{
    return m_name;
}

void TriggerSetting::IncrementValue()
{
    m_value++;
}

void TriggerSetting::DecrementValue()
{
    m_value--;
}
