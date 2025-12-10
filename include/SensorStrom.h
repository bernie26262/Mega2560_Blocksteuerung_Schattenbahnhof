#pragma once
#include <Arduino.h>

class SensorStrom {
public:
    explicit SensorStrom(uint8_t pin, uint16_t threshold = 100)
        : m_pin(pin), m_threshold(threshold)
    {}

    void begin() {}

    void update() {
        uint16_t v = analogRead(m_pin);
        m_filtered = (m_filtered * 9 + v) / 10;
    }

    uint16_t filtered() const { return m_filtered; }

    bool overThreshold() const {
        return m_filtered >= m_threshold;
    }

    void setThreshold(uint16_t t) { m_threshold = t; }

private:
    uint8_t  m_pin;
    uint16_t m_threshold;
    uint16_t m_filtered = 0;
};
