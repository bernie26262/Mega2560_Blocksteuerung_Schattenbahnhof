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
    m_winSum = 0;
    m_winSumSq = 0;
    m_winMin = 1023;
    m_winMax = 0;

    m_qCount = 0;
    m_qW = 0;
    m_qR = 0;
    for (uint8_t i = 0; i < WIN_Q; i++) {
        m_qSum[i] = 0;
        m_qSumSq[i] = 0;
        m_qMin[i] = 0;
        m_qMax[i] = 0;
    }

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

    // True RMS window accumulation:
    // mean  = E[x]
    // ex2   = E[x^2]
    // var   = ex2 - mean^2
    m_winSum   += (uint32_t)raw;
    m_winSumSq += (uint64_t)raw * (uint64_t)raw;

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
        m_qSum[w]   = m_winSum;
        m_qSumSq[w] = m_winSumSq;
        m_qMin[w]   = (uint16_t)m_winMin;
        m_qMax[w]   = (uint16_t)m_winMax;

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
        m_winSum = 0;
        m_winSumSq = 0;
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
        uint32_t sum = 0;
        uint64_t sumSq = 0;
        uint16_t qmin = 1023;
        uint16_t qmax = 0;
        noInterrupts();
        if (m_qCount) {
            const uint8_t r = m_qR;
            sum   = m_qSum[r];
            sumSq = m_qSumSq[r];
            qmin  = m_qMin[r];
            qmax  = m_qMax[r];
            m_qR = (uint8_t)((r + 1u) % WIN_Q);
            m_qCount--;
        }
        interrupts();

        // True RMS in ADC domain via variance:
        // var = E[x^2] - E[x]^2
        const float n = (float)WINDOW_SAMPLES;
        const float mean = (float)sum / n;
        const float ex2  = (float)sumSq / n;
        float var = ex2 - (mean * mean);
        if (var < 0.0f) var = 0.0f;
        const float rmsCounts = sqrtf(var);
        float vrmsAdc = (rmsCounts * 5.0f) / 1023.0f;

        // If the ADC waveform clipped within this window, ignore it.
        // Clipping creates unstable RMS (value can jump dramatically).
        if (qmin <= 1u || qmax >= 1022u) {
            continue;
        }

        // Basic sanity clamp (ADC RMS should be below ~2.5V for biased AC-coupled input)
        if (vrmsAdc > 3.0f) { vrmsAdc = 0.0f; }

        const float vrmsTrafo = vrmsAdc * m_scale;

        // Median + asymmetrische Glättung:
        // - im Normalbetrieb wieder ruhiger/stabiler
        // - bei deutlich fallender Spannung weiterhin schneller nach unten
        // - nahe 0V weiterhin sehr schneller Rücklauf
        m_lastRms[m_lastRmsIdx] = vrmsTrafo;
        if (m_lastRmsCount < 5u) m_lastRmsCount++;
        m_lastRmsIdx = (uint8_t)((m_lastRmsIdx + 1u) % 5u);

        float tmp[5] = {0};
        for (uint8_t i = 0; i < m_lastRmsCount; i++) tmp[i] = m_lastRms[i];
        const float med = medianOfUpTo5(tmp, m_lastRmsCount);

        const float oldTrafo = m_rmsFilteredTrafo;
        const float oldAdc   = m_rmsFilteredAdc;
        
        // ADC-domain median approximated from trafo median to keep both domains aligned.
        const float medAdc = (m_scale > 0.0001f) ? (med / m_scale) : vrmsAdc;

        // Asymmetric filter coefficients
        // up / flat: calmer in steady operation
        const float ALPHA_UP       = 0.05f; // 5% new
        // clear falling edge: faster response
        const float ALPHA_DOWN     = 0.28f; // 28% new
        // near zero / trafo off: very fast decay
        const float ALPHA_NEARZERO = 0.55f; // 55% new

        // Thresholds
        const float DROP_RATIO     = 0.85f; // "significantly lower than before"
        const float NEARZERO_TRAFO = 1.20f; // below ~1.2V treat as near-zero
        const float NEARZERO_ADC   = (m_scale > 0.0001f) ? (NEARZERO_TRAFO / m_scale) : 0.05f;
        const float HOLD_BAND_TRAFO = 0.8f; // ignore small jitter around current value
        const float HOLD_BAND_ADC   = (m_scale > 0.0001f) ? (HOLD_BAND_TRAFO / m_scale) : 0.02f;

        float alphaTrafo = ALPHA_UP;
        float alphaAdc   = ALPHA_UP;

        if (med <= NEARZERO_TRAFO) {
            alphaTrafo = ALPHA_NEARZERO;
        } else if ((oldTrafo > 0.01f) && (med < oldTrafo * DROP_RATIO)) {
            alphaTrafo = ALPHA_DOWN;
        }

        if (medAdc <= NEARZERO_ADC) {
            alphaAdc = ALPHA_NEARZERO;
        } else if ((oldAdc > 0.001f) && (medAdc < oldAdc * DROP_RATIO)) {
            alphaAdc = ALPHA_DOWN;
        }

        float targetTrafo = med;
        float targetAdc   = medAdc;

        // Hold-band against jitter in normal operation:
        // keep the displayed value if the new median only differs slightly.
        // Do NOT hold near zero and do NOT hold on clear falling edges.
        const bool trafoNearZero = (med <= NEARZERO_TRAFO);
        const bool adcNearZero   = (medAdc <= NEARZERO_ADC);
        const bool trafoFastDown = ((oldTrafo > 0.01f) && (med < oldTrafo * DROP_RATIO));
        const bool adcFastDown   = ((oldAdc > 0.001f) && (medAdc < oldAdc * DROP_RATIO));

        if (!trafoNearZero && !trafoFastDown && fabsf(med - oldTrafo) < HOLD_BAND_TRAFO) {
            targetTrafo = oldTrafo;
        }
        if (!adcNearZero && !adcFastDown && fabsf(medAdc - oldAdc) < HOLD_BAND_ADC) {
            targetAdc = oldAdc;
        }

        float nextTrafo = (oldTrafo * (1.0f - alphaTrafo)) + (targetTrafo * alphaTrafo);
        float nextAdc   = (oldAdc   * (1.0f - alphaAdc))   + (targetAdc   * alphaAdc);

        // Slew limiter for NORMAL operation only:
        // limit visible step size per completed RMS window.
        // Keep fast-down / near-zero behaviour untouched.
        const float MAX_STEP_TRAFO = 0.35f; // V per window (~0.5 s)
        const float MAX_STEP_ADC   = (m_scale > 0.0001f) ? (MAX_STEP_TRAFO / m_scale) : 0.01f;

        if (!trafoNearZero && !trafoFastDown) {
           const float d = nextTrafo - oldTrafo;
            if (d >  MAX_STEP_TRAFO) nextTrafo = oldTrafo + MAX_STEP_TRAFO;
            if (d < -MAX_STEP_TRAFO) nextTrafo = oldTrafo - MAX_STEP_TRAFO;
        }
        if (!adcNearZero && !adcFastDown) {
            const float d = nextAdc - oldAdc;
            if (d >  MAX_STEP_ADC) nextAdc = oldAdc + MAX_STEP_ADC;
            if (d < -MAX_STEP_ADC) nextAdc = oldAdc - MAX_STEP_ADC;
        }

        m_rmsFilteredTrafo = nextTrafo;
        m_rmsFilteredAdc   = nextAdc;

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