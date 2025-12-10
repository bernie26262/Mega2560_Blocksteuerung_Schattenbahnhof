#pragma once
#include <Arduino.h>

class PulseSensor {
public:
    PulseSensor(uint8_t pin)
        : m_pin(pin)
    {}

    void begin() {
        pinMode(m_pin, INPUT_PULLUP);
        m_last = digitalRead(m_pin);
    }

    void update() {} // braucht nichts

    bool fellEdge() {
        int raw = (m_sim >= 0 ? m_sim : digitalRead(m_pin));
        bool event = (m_last == HIGH && raw == LOW);
        m_last = raw;
        return event;
    }

    // Simulation:
    void simulateLow()  { m_sim = LOW;  }
    void simulateHigh() { m_sim = HIGH; }

private:
    uint8_t m_pin;
    int m_last = HIGH;

    int m_sim = -1; // -1 = echt lesen, 0/1 = Simulation
};
