#include "SensorTrafoAC.h"
#include <Arduino.h>
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
    m_biasInit = false;
    m_biasQ8 = 0;
    m_winEnergy = 0;
    m_winMin = 1023;
    m_winMax = 0;

    m_qCount = 0;
    m_qW = 0;
    m_qR = 0;
    for (uint8_t i = 0; i < WIN_Q; i++) { m_qEnergy[i] = 0; }

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
    // ISR-safe diagnostics
    const int16_t r = (int16_t)raw;
    if (r < m_winMin) m_winMin = r;
    if (r > m_winMax) m_winMax = r;

    // BLR-like running bias (Q8 fixed point)
    if (!m_biasInit) {
        m_biasQ8 = ((int32_t)raw) << 8;
        m_biasInit = true;
    } else {
        const int32_t rawQ8 = ((int32_t)raw) << 8;
        m_biasQ8 += (rawQ8 - m_biasQ8) >> BIAS_SHIFT;
    }

    const int16_t centered = (int16_t)((int32_t)raw - (m_biasQ8 >> 8));
    const int32_t c = (int32_t)centered;
    m_winEnergy += (uint32_t)(c * c);

    uint16_t n = m_winSamples + 1u;
    m_winSamples = n;

    // Mirror for diagnostics (loop reads these)
    if (m_sampleCount < 0xFFFFu) m_sampleCount++;
    m_minSample = m_winMin;
    m_maxSample = m_winMax;

    if (n >= WINDOW_SAMPLES)
    {
        // Push completed window into bounded queue.
        uint8_t w = m_qW;
        m_qEnergy[w] = m_winEnergy;
        m_qMin[w]    = (uint16_t)m_winMin;
        m_qMax[w]    = (uint16_t)m_winMax;

        w = (uint8_t)((w + 1u) % WIN_Q);
        m_qW = w;

        if (m_qCount < WIN_Q) {
            m_qCount++;
        } else {
            // Drop oldest to keep newest windows (avoid blocking loop)
            m_qR = (uint8_t)((m_qR + 1u) % WIN_Q);
            m_missedSamples++;
        }

        // Reset ISR window
        m_winSamples = 0;
        m_winEnergy = 0;
        m_winMin = 1023;
        m_winMax = 0;
    }
}

void SensorTrafoAC::update(uint32_t nowMs)
{
    (void)nowMs;

    // Consume completed windows (at most a few).
    while (m_qCount)
    {
        // Atomically claim one window from the queue.
        uint32_t energy = 0;
        uint16_t qmin = 1023;
        uint16_t qmax = 0;
        noInterrupts();
        if (m_qCount) {
            const uint8_t r = m_qR;
            energy = m_qEnergy[r];
            qmin   = m_qMin[r];
            qmax   = m_qMax[r];
            m_qR = (uint8_t)((r + 1u) % WIN_Q);
            m_qCount--;
        }
        interrupts();

        // BLR-like true RMS in ADC domain from centered energy
        const float n = (float)WINDOW_SAMPLES;
        const float ms = (float)energy / n;
        const float rmsCounts = sqrtf(ms);
        float vrmsAdc = (rmsCounts * 5.0f) / 1023.0f;

        // If the ADC waveform clipped within this window, ignore it.
        // Clipping creates unstable RMS (value can jump dramatically).
        if (qmin <= 1u || qmax >= 1022u) {
            continue;
        }

        // Basic sanity clamp (ADC RMS should be below ~2.5V for biased AC-coupled input)
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
        m_rmsFilteredTrafo = (oldTrafo * 0.96f) + (med * 0.04f);
        m_rmsFilteredAdc   = (oldAdc   * 0.96f) + (vrmsAdc * 0.04f);
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