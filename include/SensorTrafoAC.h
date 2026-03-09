#pragma once
#include <Arduino.h>

/*
   SensorTrafoAC – AC-Spannungsmessung (Märklin Trafo) via ADC

   Ziel:
   - echte RMS-Messung aus ADC-Samples (robust gegen Spikes/Dropouts)
   - Windowing (WINDOW_MS) + Median(5) + EMA für ruhige Anzeige
   - kein Blocking in loop()
   - Ergebnis ist "ADC-domain Vrms" (Volt am ADC-Pin) und "Trafo Vrms" via scale()

   Methode:
   - Sampling wird von AdcScheduler im ADC-ISR getrieben (z.B. 500 Hz / Kanal)
   - Im ISR pro Fenster Summe und Summe der Quadrate sammeln
   - Am Fensterende: var = E[x^2] - E[x]^2, rmsCounts = sqrt(var)
   - vrmsAdc = rmsCounts * (Vref/adcMax)
   - vrmsTrafo = vrmsAdc * m_scale
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

    // Dropped windows (if loop cannot keep up with window consumption).
    uint32_t missedSamples() const { return m_missedSamples; }

    bool isPowered() const { return m_rmsFilteredTrafo > m_powerThreshold; }

    // Trafo-domain threshold (Vrms am Trafo-Ausgang)
    void setPowerThreshold(float v) { m_powerThreshold = v; }

    // Skalierung: ADC-domain Vrms -> Trafo Vrms
    void setScale(float k) { m_scale = k; }
    float scale() const { return m_scale; }


    void printDebug(const char* label) const;

    // Diagnostics (last ISR window min/max, sample count)
    int16_t minSample() const { return m_minSample; }
    int16_t maxSample() const { return m_maxSample; }
    uint16_t samplesInWindow() const { return m_sampleCount; }

private:
    uint8_t m_pin;

    // Diagnostics mirror (updated in ISR)
    volatile int16_t  m_minSample   = 1023;
    volatile int16_t  m_maxSample   = 0;
    volatile uint16_t m_sampleCount = 0;

    // Window accumulators (ISR updates)
    volatile uint16_t m_winSamples = 0;

    // Window accumulators for true RMS via
    // var = E[x^2] - E[x]^2
    volatile uint32_t m_winSum   = 0;   // sum(x)
    volatile uint64_t m_winSumSq = 0;   // sum(x^2)

    // Completed-window queue (bounded, avoids losing windows when loop is busy)
    static constexpr uint8_t WIN_Q = 3;
    volatile uint8_t  m_qCount = 0;
    volatile uint8_t  m_qW = 0;
    volatile uint8_t  m_qR = 0;
    volatile uint32_t m_qSum[WIN_Q]    = {0};
    volatile uint64_t m_qSumSq[WIN_Q]  = {0};
    volatile uint16_t m_qMin[WIN_Q]    = {0};
    volatile uint16_t m_qMax[WIN_Q]    = {0};

    // Mirror for diagnostics (loop reads these)
    volatile int16_t  m_winMin = 1023;
    volatile int16_t  m_winMax = 0;
    volatile uint32_t m_missedSamples = 0;
    float m_scale = 1.0f;
    float m_powerThreshold = 2.0f;

    // Filtered results (loop-updated, read by UI/debug)
    float    m_rmsFilteredAdc   = 0.0f; // Vrms at ADC input domain
    float    m_rmsFilteredTrafo = 0.0f; // Vrms at trafo output domain
    // Cached integer values for cheap UI/debug transport
    uint16_t m_rmsFilteredTrafo_cV = 0; // centivolt
    uint16_t m_rmsFilteredAdc_mV   = 0; // millivolt


    // Outlier handling
    static constexpr uint8_t  MEDIAN_N = 5;
    float m_lastRms[MEDIAN_N] = {0};
    uint8_t m_lastRmsCount = 0;
    uint8_t m_lastRmsIdx   = 0;

    static constexpr uint16_t WINDOW_MS = 500;     // 25 Perioden @50Hz (ruhiger, robust gegen Motor/Relais)     // 10 Perioden @50Hz
    static constexpr uint16_t SAMPLE_HZ = 500;     // per channel (via schedule)
    static constexpr uint16_t WINDOW_SAMPLES = (uint16_t)((SAMPLE_HZ * WINDOW_MS) / 1000u);
};