#pragma once
#include <string>

class TriggerSetting{
private:
    int m_value;
    std::string m_name;

public:
    TriggerSetting();
    TriggerSetting(int defaultValue, std::string_view name);

    int GetValue() const;
    const std::string GetName() const;

    void IncrementValue();
    void DecrementValue();
}; 


