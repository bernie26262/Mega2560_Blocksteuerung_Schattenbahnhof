#pragma once

#include <Arduino.h>
#include "PowerControl.h"

class Mega2PowerControl : public PowerControl
{
public:
    void begin();

    void setBlock5ToSBhf(bool on) override;
    void setSbhfGleis(uint8_t gleis, bool on) override;
    void setNothalt(bool on) override;

    void emergencyShutdown();

    // ----------------------------------------
    // Status-Getter (logischer Zustand)
    // ----------------------------------------
    bool isTrafoAEnabled() const { return m_trafoAEnabled; }
    bool isTrafoBEnabled() const { return m_trafoBEnabled; }

private:
    // zuletzt gesetzter Zustand (keine HW-Rückmeldung)
    bool m_trafoAEnabled = false;
    bool m_trafoBEnabled = false;
};
