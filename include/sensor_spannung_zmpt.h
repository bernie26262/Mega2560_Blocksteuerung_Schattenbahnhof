// placeholder#pragma once
#include <Arduino.h>

/**
 * Spannungssensor für AC (Trafo-Status) basierend auf ZMPT101B.
 *
 * Misst die Wechselspannung über analogRead und bildet einen gleitenden
 * Mittelwert. Wenn der Wert über dem Schwellwert liegt → Trafo AN.
 *
 * Wichtig: Dieser Sensor ist NICHT für Blockbelegung zuständig!
 * Dafür gibt es SensorStrom.
 */
class SensorSpannungZMPT {
public:
    SensorSpannungZMPT(uint8_t pin, float threshold)
        : m_pin(pin), m_threshold(threshold) {}

    void begin() {
        m_filtered = 0.0f;
    }

    void update() {
        int raw = analogRead(m_pin);   // 0..1023
        m_filtered = m_filtered * 0.9f + (float)raw * 0.1f;
    }

    // true = Trafo liefert Spannung
    bool isSpannungAn() const {
        return m_filtered > m_threshold;
    }

    float value() const { return m_filtered; }

private:
    uint8_t m_pin;
    float   m_threshold;
    float   m_filtered = 0.0f;
};
