#pragma once
#include <Arduino.h>

// 0 = Hardware, 1 = Simulation (Debug ohne Anlage)
#ifndef MEGA2_SIM_MODE
#define MEGA2_SIM_MODE 0
#endif

class SensorKontakt {
public:
    explicit SensorKontakt(uint8_t pin)
        : m_pin(pin)
    {}

    void begin() {
        pinMode(m_pin, INPUT_PULLUP);
        m_lastRaw     = raw();
        m_state       = m_lastRaw;
        m_lastChange  = millis();
    }

    void update(uint32_t now) {
#if MEGA2_SIM_MODE
        // In SIM kann der Kontakt per Serial zwangsweise gesetzt werden
        if (m_debugForced) {
            m_lastRaw    = m_debugOcc;
            m_state      = m_debugOcc;
            m_lastChange = now;
            return;
        }
#endif
        bool r = raw();

        if (r != m_lastRaw) {
            m_lastRaw    = r;
            m_lastChange = now;
        }

        if ((now - m_lastChange) >= DEBOUNCE_MS) {
            m_state = r;
        }
    }

    bool isOccupied() const {
        return m_state; // LOW = aktiv = belegt
    }

    // raw() liefert "belegt" als bool (true wenn LOW)
    bool raw() const {
#if MEGA2_SIM_MODE
        if (m_debugForced) return m_debugOcc;
#endif
        return (digitalRead(m_pin) == LOW);
    }

#if MEGA2_SIM_MODE
    // Debug/Test: Kontaktzustand erzwingen (Simulation)
    // occupied=true  => "belegt" (LOW)
    // occupied=false => "frei"   (HIGH)
    void debugForce(bool occupied)
    {
        m_debugForced = true;
        m_debugOcc    = occupied;

        // sofort wirksam
        m_lastRaw     = occupied;
        m_state       = occupied;
        m_lastChange  = millis();
    }

    void debugRelease()
    {
        m_debugForced = false;
    }
#endif

private:
    uint8_t  m_pin;

    bool     m_lastRaw    = false;
    bool     m_state      = false;
    uint32_t m_lastChange = 0;

#if MEGA2_SIM_MODE
    bool     m_debugForced = false;
    bool     m_debugOcc    = false;
#endif

    static constexpr uint16_t DEBOUNCE_MS = 50;
};
