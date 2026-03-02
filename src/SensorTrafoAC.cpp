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

    m_winStartUs = 0;
    m_lastSampleUs = 0;

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

void SensorTrafoAC::update(uint32_t nowMs, uint32_t nowUs)
{
    (void)nowMs;

    if (m_winStartUs == 0) {
        m_winStartUs = nowUs;
        m_lastSampleUs = nowUs;
    }

    // time-based sampling at 500 Hz
    if ((uint32_t)(nowUs - m_lastSampleUs) < SAMPLE_INTERVAL_US) {
        return;
    }
    m_lastSampleUs += SAMPLE_INTERVAL_US; // keep phase even if loop jitters

    const uint16_t raw = (uint16_t)analogRead(m_pin);

    if ((int16_t)raw < m_minSample) m_minSample = (int16_t)raw;
    if ((int16_t)raw > m_maxSample) m_maxSample = (int16_t)raw;
    if (m_sampleCount < 0xFFFFu) m_sampleCount++;

    const uint32_t winUs = (uint32_t)WINDOW_MS * 1000UL;
    if ((uint32_t)(nowUs - m_winStartUs) < winUs) {
        return;
    }

    // --- Window finished: compute Vrms from peak-to-peak (robust for clean sine) ---
    const int16_t vppCounts = (int16_t)(m_maxSample - m_minSample);
    float vrmsAdc = 0.0f;

    if (vppCounts > 0) {
        const float vppVolts = ((float)vppCounts * 5.0f) / 1023.0f;
        // Vrms ≈ Vpp / (2*sqrt(2))
        vrmsAdc = vppVolts * 0.35355339059f;
    }

    // very large spikes should not poison the output (e.g. glitch in min/max)
    // Your circuit typically stays <= ~1.3Vrms at ADC. We still allow margin.
    if (vrmsAdc > 3.0f) { vrmsAdc = 0.0f; }
    const float vrmsTrafo = vrmsAdc * m_scale;
    

    // Store for median filter (outlier rejection)
    m_lastRms[m_lastRmsIdx] = vrmsTrafo;
    if (m_lastRmsCount < MEDIAN_N) m_lastRmsCount++;
    m_lastRmsIdx = (uint8_t)((m_lastRmsIdx + 1u) % MEDIAN_N);

    float tmp[5] = {0};
    for (uint8_t i = 0; i < m_lastRmsCount; i++) tmp[i] = m_lastRms[i];
    const float med = medianOfUpTo5(tmp, m_lastRmsCount);

    // EMA on median for a calm display
    const float oldTrafo = m_rmsFilteredTrafo;
    const float oldAdc   = m_rmsFilteredAdc;
    m_rmsFilteredTrafo = (oldTrafo * 0.80f) + (med * 0.20f);
    m_rmsFilteredAdc   = (oldAdc   * 0.80f) + (vrmsAdc * 0.20f);

    // Reset window
    m_minSample = 1023;
    m_maxSample = 0;
    m_sampleCount = 0;
    m_winStartUs = nowUs;
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

float SensorTrafoAC::measureVrmsBlocking(uint16_t freqHz, uint8_t periods) const
{
    if (freqHz == 0) freqHz = 50;
    if (periods == 0) periods = 1;

    const uint32_t periodUs = 1000000UL / (uint32_t)freqHz;
    const uint32_t windowUs = periodUs * (uint32_t)periods;

    // Pass 1: mean (zero point)
    uint32_t sum = 0;
    uint16_t n = 0;
    const uint32_t t0 = micros();
    while ((uint32_t)(micros() - t0) < windowUs)
    {
        sum += (uint16_t)analogRead(m_pin);
        n++;
    }
    if (n == 0) return 0.0f;
    const uint16_t mean = (uint16_t)(sum / n);

    // Pass 2: mean square of deviation
    uint32_t sumsq = 0;
    uint16_t n2 = 0;
    const uint32_t t1 = micros();
    while ((uint32_t)(micros() - t1) < windowUs)
    {
        const int16_t v = (int16_t)analogRead(m_pin) - (int16_t)mean;
        sumsq += (uint32_t)(v * v);
        n2++;
    }
    if (n2 == 0) return 0.0f;

    const float ms = (float)sumsq / (float)n2;
    const float rmsCounts = sqrtf(ms);

    // Convert counts -> volts at ADC input (sensor-domain, uncalibrated to track volts)
    const float v = (rmsCounts * 5.0f) / 1023.0f;
    return v;
}