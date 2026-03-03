#pragma once
#include <Arduino.h>

/*
   SensorTrafoAC – AC-Spannungsmessung (Märklin Trafo) via ADC + Bias

   Ziel:
   - effiziente RMS-Schätzung bei (nahezu) sinusförmigem 50Hz-Signal
   - robust gegen kurze Spikes (Richtungswechsel/Relais) durch Window + Median
   - kein Blocking in loop()
   - Ergebnis ist "ADC-domain Vrms" (Volt am ADC-Pin), nicht die Trafospannung
   Methode:
   - sampling zeitbasiert mit micros(): 500Hz (alle 2000µs)
   - pro Fenster (WINDOW_MS, default 200ms) min/max sammeln
   - Vrms ≈ ((max-min)/2) * (5/1023) / sqrt(2)
   - Outlier-Filter: Median über die letzten MEDIAN_N Fenster
   - Anzeige-Glättung: EMA über Median
*/

class SensorTrafoAC {
public:
    explicit SensorTrafoAC(uint8_t pin);

    void begin();
    // Called from ADC ISR via AdcScheduler sink.
    // Must stay ISR-safe (no floats, no Serial).
    void onSampleISR(uint16_t raw);

    // Called from loop(): consumes completed windows and updates filters.
    void update(uint32_t nowMs);

    float rms() const { return m_rmsFilteredTrafo; }
    float rmsAdc() const { return m_rmsFilteredAdc; }
    // Cached integer values for cheap debug printing (updated in update())
    uint16_t rmsTrafo_cV() const { return m_rmsFilteredTrafo_cV; }  // centivolt
    uint16_t rmsAdc_mV()   const { return m_rmsFilteredAdc_mV; }    // millivolt

    // Kept for compatibility: counts dropped samples for this channel (if any).
    // With ISR min/max, this should stay 0 unless scheduling/ADC is misconfigured.
    uint32_t missedSamples() const { return m_missedSamples; }
    bool isPowered() const { return m_rmsFilteredTrafo > m_powerThreshold; }

    // Trafo-domain threshold (Vrms am Trafo-Ausgang)
    void setPowerThreshold(float v) { m_powerThreshold = v; }

    // Skalierung: ADC-domain Vrms -> Trafo Vrms
    void setScale(float k) { m_scale = k; }
    float scale() const { return m_scale; }

    // Blocking helper (nur für CALIB/Debug)
    
    void printDebug(const char* label) const;

    // Diagnostics
    int16_t minSample() const { return m_minSample; }
    int16_t maxSample() const { return m_maxSample; }
    uint16_t samplesInWindow() const { return m_sampleCount; }

private:
    uint8_t m_pin;

    int16_t  m_minSample   = 1023;
    int16_t  m_maxSample   = 0;
    uint16_t m_sampleCount = 0;

    // Window state (ISR updates)
    volatile uint16_t m_winSamples = 0;
    volatile int16_t  m_winMin = 1023;
    volatile int16_t  m_winMax = 0;
    volatile uint8_t  m_winReady = 0; // counts ready windows (bounded)
    volatile int16_t  m_readyVppCounts = 0;

    float m_rmsFilteredAdc   = 0.0f;
    float m_rmsFilteredTrafo = 0.0f;
    uint16_t m_rmsFilteredTrafo_cV = 0;
    uint16_t m_rmsFilteredAdc_mV = 0;
    volatile uint32_t m_missedSamples = 0;
    float m_scale = 1.0f;
    float m_powerThreshold = 2.0f;

    // Outlier handling
    static constexpr uint8_t  MEDIAN_N = 5;
    float m_lastRms[MEDIAN_N] = {0};
    uint8_t m_lastRmsCount = 0;
    uint8_t m_lastRmsIdx   = 0;

    static constexpr uint16_t WINDOW_MS = 200;     // 10 Perioden @50Hz
    static constexpr uint16_t SAMPLE_HZ = 500;     // per channel (via schedule)
    static constexpr uint16_t WINDOW_SAMPLES = (uint16_t)((SAMPLE_HZ * WINDOW_MS) / 1000u); // 100
};