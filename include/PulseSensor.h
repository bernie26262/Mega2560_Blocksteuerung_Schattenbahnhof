#pragma once
#include <Arduino.h>

// Pulse / edge sensor with counters (for diag) and a compatible fellEdge() API.
//
// Semantik (INPUT_PULLUP):
//  - levelActive() == true  <=> raw electrical LOW  => logisch aktiv
//  - riseCount: logical 0->1 transitions (HIGH->LOW electrical)
//  - fallCount: logical 1->0 transitions (LOW->HIGH electrical)
//
// Stabilisierung:
//  - Entprellung (debounce) auf Zustandswechsel
//  - Mindestabstand zwischen gezählten Edges (minEdgeGap)
// -> verhindert Ghost-Pulse / Flattern bei langen Leitungen / Kontaktprellen.
//
// Counters are uint8_t and wrap naturally.
class PulseSensor {
public:
    explicit PulseSensor(uint8_t pin)
        : m_pin(pin)
    {}

    void begin() {
        pinMode(m_pin, INPUT_PULLUP);

        const int rawNow = readRaw();
        m_stableRaw      = rawNow;
        m_candidateRaw   = rawNow;
        m_levelActive    = (rawNow == LOW);

        const uint32_t nowUs = micros();
        m_candidateSinceUs = nowUs;
        m_lastEdgeUs       = nowUs;
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

    // Optional tuning (call after ctor, before begin or anytime)
    void setDebounceUs(uint16_t us)   { m_debounceUs   = us; }
    void setMinEdgeGapUs(uint16_t us) { m_minEdgeGapUs = us; }

    // Simulation:
    void simulateLow()  { m_sim = LOW;  }
    void simulateHigh() { m_sim = HIGH; }

private:
    enum : uint8_t { EDGE_RISE = 1u, EDGE_FALL = 2u }; // rise: raw LOW->HIGH, fall: raw HIGH->LOW

    int readRaw() const {
        return (m_sim >= 0 ? m_sim : digitalRead(m_pin));
    }

    // Returns edge bitmask; debounced; updates counters + level.
    uint8_t sample() {
        const int rawNow = readRaw();
        const uint32_t nowUs = micros();

        // no change vs stable -> keep level, clear candidate
        if (rawNow == m_stableRaw) {
            m_levelActive = (m_stableRaw == LOW);
            m_candidateRaw = m_stableRaw;
            m_candidateSinceUs = nowUs;
            return 0;
        }

        // candidate tracking
        if (rawNow != m_candidateRaw) {
            m_candidateRaw = rawNow;
            m_candidateSinceUs = nowUs;
            m_levelActive = (m_stableRaw == LOW); // keep old stable level until accepted
            return 0;
        }

        // candidate has remained the same long enough?
        if ((uint32_t)(nowUs - m_candidateSinceUs) < (uint32_t)m_debounceUs) {
            m_levelActive = (m_stableRaw == LOW);
            return 0;
        }

        // enforce minimum gap between counted edges (suppresses chatter)
        if ((uint32_t)(nowUs - m_lastEdgeUs) < (uint32_t)m_minEdgeGapUs) {
            // do not accept yet -> keep stable
            m_levelActive = (m_stableRaw == LOW);
            return 0;
        }

        // accept debounced transition
        uint8_t edges = 0;

        // electrical: HIGH->LOW == activation
        if (m_stableRaw == HIGH && m_candidateRaw == LOW) {
            edges |= EDGE_FALL;
            m_rise++; // logical 0->1
        } else if (m_stableRaw == LOW && m_candidateRaw == HIGH) {
            edges |= EDGE_RISE;
            m_fall++; // logical 1->0
        }

        m_stableRaw = m_candidateRaw;
        m_lastEdgeUs = nowUs;
        m_levelActive = (m_stableRaw == LOW);
        return edges;
    }

    uint8_t  m_pin;

    int      m_stableRaw = HIGH;
    int      m_candidateRaw = HIGH;
    uint32_t m_candidateSinceUs = 0;
    uint32_t m_lastEdgeUs = 0;

    bool     m_levelActive = false;

    uint8_t  m_rise = 0;
    uint8_t  m_fall = 0;

    // defaults: conservative, good starting point for “Schaltgleis”
    uint16_t m_debounceUs   = 3000; // 3 ms stable required
    uint16_t m_minEdgeGapUs = 5000; // 5 ms between counted edges

    int m_sim = -1; // -1 = hardware, 0/1 = simulation
};