#include "SensorTrafoAC.h"
#include <math.h>
#include <string.h>

SensorTrafoAC::SensorTrafoAC(uint8_t pin)
    : m_pin(pin)
{
}

void SensorTrafoAC::begin()
{
    pinMode(m_pin, INPUT);

    m_minSample = 1023;
    m_maxSample = 0;
    m_sampleCount = 0;

    m_winSamples = 0;
    m_winMin = 1023;
    m_winMax = 0;
    m_winReady = 0;
    m_readyVppCounts = 0;

    m_rmsFilteredAdc = 0.0f;
    m_rmsFilteredTrafo = 0.0f;

    memset(m_lastRms, 0, sizeof(m_lastRms));
    m_lastRmsCount = 0;
    m_lastRmsIdx = 0;
}

static float medianOfUpTo5(const float* v, uint8_t n)
{
    // n: 1..5
    float a[5];
    for (uint8_t i = 0; i < n; i++) a[i] = v[i];

    // insertion sort (n <= 5)
    for (uint8_t i = 1; i < n; i++) {
        float key = a[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && a[j] > key) {
            a[j + 1] = a[j];
            j--;
        }
        a[j + 1] = key;
    }

    return a[n / 2];
}

void SensorTrafoAC::onSampleISR(uint16_t raw)
{
    // ISR-safe min/max + sample count.
    if ((int16_t)raw < m_winMin) m_winMin = (int16_t)raw;
    if ((int16_t)raw > m_winMax) m_winMax = (int16_t)raw;

    uint16_t n = m_winSamples + 1u;
    m_winSamples = n;

    // Mirror for diagnostics (loop reads these)
    if (m_sampleCount < 0xFFFFu) m_sampleCount++;
    m_minSample = m_winMin;
    m_maxSample = m_winMax;

    if (n >= WINDOW_SAMPLES)
    {
        const int16_t vpp = (int16_t)(m_winMax - m_winMin);
        m_readyVppCounts = vpp;
        if (m_winReady < 3) m_winReady++; // bound backlog

        // Reset ISR window
        m_winSamples = 0;
        m_winMin = 1023;
        m_winMax = 0;
    }
}

void SensorTrafoAC::update(uint32_t nowMs)
{
    (void)nowMs;

    // Consume ready windows (at most a few).
    while (m_winReady)
    {
        // Atomically claim one window.
        int16_t vppCounts = 0;
        noInterrupts();
        if (m_winReady) {
            m_winReady--;
            vppCounts = m_readyVppCounts;
        }
        interrupts();

        float vrmsAdc = 0.0f;
        if (vppCounts > 0) {
            const float vppVolts = ((float)vppCounts * 5.0f) / 1023.0f;
            vrmsAdc = vppVolts * 0.35355339059f; // Vpp/(2*sqrt(2))
        }

        if (vrmsAdc > 3.0f) { vrmsAdc = 0.0f; }
        const float vrmsTrafo = vrmsAdc * m_scale;

        // Median + EMA (display calm)
        m_lastRms[m_lastRmsIdx] = vrmsTrafo;
        if (m_lastRmsCount < MEDIAN_N) m_lastRmsCount++;
        m_lastRmsIdx = (uint8_t)((m_lastRmsIdx + 1u) % MEDIAN_N);

        float tmp[5] = {0};
        for (uint8_t i = 0; i < m_lastRmsCount; i++) tmp[i] = m_lastRms[i];
        const float med = medianOfUpTo5(tmp, m_lastRmsCount);

        const float oldTrafo = m_rmsFilteredTrafo;
        const float oldAdc   = m_rmsFilteredAdc;
        m_rmsFilteredTrafo = (oldTrafo * 0.80f) + (med * 0.20f);
        m_rmsFilteredAdc   = (oldAdc   * 0.80f) + (vrmsAdc * 0.20f);
        // Cache integer representations for cheap debug printing
        m_rmsFilteredTrafo_cV = (uint16_t)lroundf(m_rmsFilteredTrafo * 100.0f);
        m_rmsFilteredAdc_mV   = (uint16_t)lroundf(m_rmsFilteredAdc   * 1000.0f);
    }
}

void SensorTrafoAC::printDebug(const char* label) const
{
    Serial.print(label);
    Serial.print(F(" Vrms="));
    Serial.print(m_rmsFilteredTrafo, 2);
    Serial.print(F("V (trafo), adc="));
    Serial.print(m_rmsFilteredAdc, 3);
    Serial.print(F("V  powered="));
    Serial.println(isPowered() ? F("YES") : F("NO"));
}