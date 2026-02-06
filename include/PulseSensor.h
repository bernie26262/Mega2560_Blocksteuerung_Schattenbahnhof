#pragma once
#include <Arduino.h>

// Pulse / edge sensor with counters (for diag) and a compatible fellEdge() API.
// Semantik:
//  - levelActive() == true  <=> raw electrical LOW (INPUT_PULLUP)  => logisch aktiv
//  - riseCount:  logical 0->1 transitions (HIGH->LOW electrical)
//  - fallCount:  logical 1->0 transitions (LOW->HIGH electrical)
// Counters are uint8_t and wrap naturally.
class PulseSensor {
public:
    explicit PulseSensor(uint8_t pin)
        : m_pin(pin)
    {}

    void begin() {
        pinMode(m_pin, INPUT_PULLUP);
        const int rawNow = readRaw();
        m_lastRaw = rawNow;
        m_levelActive = (rawNow == LOW);
    }

    // Optional: call periodically (not required if you call fellEdge()/roseEdge()).
    void update() { sample(); }

    // Backward compatible: HIGH->LOW electrical edge (activation on pullup inputs)
    bool fellEdge() {
        const uint8_t e = sample();
        return (e & EDGE_FALL) != 0;
    }

    // New: LOW->HIGH electrical edge (deactivation)
    bool roseEdge() {
        const uint8_t e = sample();
        return (e & EDGE_RISE) != 0;
    }

    // Current logical level (1=aktiv/LOW, 0=inaktiv/HIGH)
    bool levelActive() const { return m_levelActive; }

    uint8_t riseCount() const { return m_rise; }
    uint8_t fallCount() const { return m_fall; }

    // Simulation:
    void simulateLow()  { m_sim = LOW;  }
    void simulateHigh() { m_sim = HIGH; }

private:
    enum : uint8_t { EDGE_RISE = 1u, EDGE_FALL = 2u }; // rise: raw LOW->HIGH, fall: raw HIGH->LOW

    int readRaw() const {
        return (m_sim >= 0 ? m_sim : digitalRead(m_pin));
    }

    // Returns edge bitmask; also updates counters + level.
    uint8_t sample() {
        const int rawNow = readRaw();
        if (rawNow == m_lastRaw) {
            m_levelActive = (rawNow == LOW);
            return 0;
        }

        uint8_t edges = 0;
        // electrical: HIGH->LOW == activation
        if (m_lastRaw == HIGH && rawNow == LOW) {
            edges |= EDGE_FALL;
            m_rise++; // logical 0->1
        } else if (m_lastRaw == LOW && rawNow == HIGH) {
            edges |= EDGE_RISE;
            m_fall++; // logical 1->0
        }

        m_lastRaw = rawNow;
        m_levelActive = (rawNow == LOW);
        return edges;
    }

    uint8_t m_pin;
    int     m_lastRaw = HIGH;
    bool    m_levelActive = false;

    uint8_t m_rise = 0;
    uint8_t m_fall = 0;

    int m_sim = -1; // -1 = hardware, 0/1 = simulation
};
