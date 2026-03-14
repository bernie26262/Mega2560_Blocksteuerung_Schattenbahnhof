#pragma once
#include <Arduino.h>

/*
   SensorTrafoAC – AC-Spannungsmessung (Märklin Trafo) via ADC

   Ziel:
   - echte RMS-Messung aus ADC-Samples (robust gegen Spikes/Dropouts)
   - Windowing über vollständige AC-Perioden + Median + EMA für ruhige Anzeige
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
    void setDebugLabel(const char* s) { m_debugLabel = s; }
    const char* debugLabel() const { return m_debugLabel; }

    // Compile-time diagnostics for boot banner / sanity checks
    static constexpr uint16_t windowMs()      { return WINDOW_MS_NOMINAL; }
    static constexpr uint16_t sampleHz()      { return SAMPLE_HZ; }
    static constexpr uint16_t windowSamples() { return WINDOW_SAMPLES_NOMINAL; }
    static constexpr uint8_t  winQ()          { return WIN_Q; }

    // Diagnostics (last ISR window min/max, sample count)
    int16_t minSample() const { return m_minSample; }
    int16_t maxSample() const { return m_maxSample; }
    uint16_t samplesInWindow() const { return m_sampleCount; }

private:
    uint8_t m_pin;
    const char* m_debugLabel = "?";

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
    static constexpr uint8_t WIN_Q = 5;
    volatile uint8_t  m_qCount = 0;
    volatile uint8_t  m_qW = 0;
    volatile uint8_t  m_qR = 0;
    volatile uint32_t m_qSum[WIN_Q]    = {0};
    volatile uint64_t m_qSumSq[WIN_Q]  = {0};
    volatile uint16_t m_qMin[WIN_Q]    = {0};
    volatile uint16_t m_qMax[WIN_Q]    = {0};
    volatile uint16_t m_qN[WIN_Q]      = {0};

    // Mirror for diagnostics (loop reads these)
    volatile int16_t  m_winMin = 1023;
    volatile int16_t  m_winMax = 0;
    volatile uint32_t m_missedSamples = 0;
    volatile uint16_t m_lastWindowSamples = 0;
    uint32_t m_lastGoodWindowMs = 0;
    float m_scale = 1.0f;
    float m_powerThreshold = 2.0f;

    // Filtered results (loop-updated, read by UI/debug)
    float    m_rmsFilteredAdc   = 0.0f; // Vrms at ADC input domain
    float    m_rmsFilteredTrafo = 0.0f; // Vrms at trafo output domain
    // Cached integer values for cheap UI/debug transport
    uint16_t m_rmsFilteredTrafo_cV = 0; // centivolt
    uint16_t m_rmsFilteredAdc_mV   = 0; // millivolt


    // Outlier handling
    static constexpr uint8_t  MEDIAN_N = 3;
    float m_lastRms[MEDIAN_N] = {0};
    uint8_t m_lastRmsCount = 0;
    uint8_t m_lastRmsIdx   = 0;

    // RMS-Fenster über vollständige AC-Perioden.
    // 50 Hz -> 1 Periode = 20 ms, 10 Perioden ~ 200 ms nominal.
    static constexpr uint8_t  WINDOW_PERIODS = 10;
    static constexpr uint16_t WINDOW_MS_NOMINAL = (uint16_t)(WINDOW_PERIODS * 20u);
    // Diagnose-/Nominalwert fuer Boot-Banner und Fenster-Nennwert.
    // Die tatsaechlich gueltige Nutzsample-Rate pro Kanal haengt vom
    // aktuellen ADC-Schedule und den verworfenen Burst-Anfangssamples ab.
    static constexpr uint16_t SAMPLE_HZ = 1000;
    static constexpr uint16_t WINDOW_SAMPLES_NOMINAL =
        (uint16_t)(((uint32_t)SAMPLE_HZ * (uint32_t)WINDOW_MS_NOMINAL) / 1000u);

    // Zero-crossing / phase-synchronous windowing state (ISR-owned)
    // Adaptive bias tracker in Q8 fixed-point (ADC counts << 8)
    volatile int32_t m_biasQ8 = ((int32_t)512 << 8);
    volatile bool    m_prevBelow = false;
    volatile bool    m_prevValid = false;
    volatile bool    m_windowArmed = false;
    volatile uint8_t m_periodCount = 0;

    // Adaptive post-filter tuning
    // Goal:
    // - noticeably calmer display in stable regions
    // - still fast on real knob movement
    // - no regression near 0V
    static constexpr float ADAPT_SNAP_ZERO_TRAFO = 0.35f;
    static constexpr float ADAPT_SNAP_ZERO_ADC   = 0.02f;
    static constexpr float ADAPT_SMALL_DELTA_TRAFO = 0.18f;
    static constexpr float ADAPT_SMALL_DELTA_ADC   = 0.010f;
    static constexpr float ADAPT_ALPHA_SLOW = 0.18f;
    static constexpr float ADAPT_ALPHA_MID  = 0.32f;
    static constexpr float ADAPT_ALPHA_FAST = 0.58f;

    static constexpr int16_t ZC_HYST = 8;          // ADC counts hysteresis around bias
    static constexpr uint8_t BIAS_SHIFT = 7;       // IIR speed: 1/128 per sample
};