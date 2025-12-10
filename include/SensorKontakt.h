#pragma once
#include <Arduino.h>

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

    bool raw() const {
        return (digitalRead(m_pin) == LOW);
    }

private:
    uint8_t  m_pin;

    bool     m_lastRaw    = false;
    bool     m_state      = false;
    uint32_t m_lastChange = 0;

    static constexpr uint16_t DEBOUNCE_MS = 50;
};
