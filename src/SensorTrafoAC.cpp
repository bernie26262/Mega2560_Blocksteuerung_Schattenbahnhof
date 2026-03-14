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
        m_qN[i] = 0;
    }

    m_rmsFilteredAdc = 0.0f;
    m_rmsFilteredTrafo = 0.0f;
    m_lastWindowSamples = 0;
    m_lastGoodWindowMs = 0;

    memset(m_lastRms, 0, sizeof(m_lastRms));
    m_lastRmsCount = 0;
    m_lastRmsIdx = 0;

    m_biasQ8 = ((int32_t)512 << 8);
    m_prevBelow = false;
    m_prevValid = false;
    m_windowArmed = false;
    m_periodCount = 0;
}

static float medianOfUpTo3(const float* v, uint8_t n)
{
    float a[3];
    for (uint8_t i = 0; i < n; i++) a[i] = v[i];

    // insertion sort (n <= 3)
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
    // Adaptive bias tracker + hysteretic zero-crossing detection.
    // This is much more robust than a hard-coded midpoint of 512.
    const int32_t rawQ8 = ((int32_t)raw << 8);
    m_biasQ8 += ((rawQ8 - m_biasQ8) >> BIAS_SHIFT);
    const int16_t bias = (int16_t)(m_biasQ8 >> 8);
    const int16_t centered = (int16_t)raw - bias;

    const bool below = (centered <= -ZC_HYST);
    const bool above = (centered >= +ZC_HYST);
    bool risingCross = false;

    if (m_prevValid) {
        // Count a new period only when we really moved from below the bias band
        // to above the bias band. This suppresses chatter around the midpoint.
        risingCross = (m_prevBelow && above);
    }

    if (below) m_prevBelow = true;
    else if (above) m_prevBelow = false;

    m_prevValid = true;

    // Fenster immer an einem definierten rising zero crossing starten.
    // Vor dem ersten Crossing noch nichts sammeln, damit das erste Fenster
    // nicht mitten in einer Periode beginnt.
    if (!m_windowArmed) {
        if (risingCross) {
            m_windowArmed = true;
            m_periodCount = 0;
            m_winSamples = 0;
            m_winSum = 0;
            m_winSumSq = 0;
            m_winMin = 1023;
            m_winMax = 0;
            m_sampleCount = 0;
            m_minSample = 1023;
            m_maxSample = 0;
            m_prevBelow = false;
        }
        return;
    }

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

    if (risingCross) {
        if (m_periodCount < 0xFFu) m_periodCount++;
    }

    if (m_periodCount >= WINDOW_PERIODS)
    {
        // Push completed window into bounded queue.
        uint8_t w = m_qW;
        m_qSum[w]   = m_winSum;
        m_qSumSq[w] = m_winSumSq;
        m_qMin[w]   = (uint16_t)m_winMin;
        m_qMax[w]   = (uint16_t)m_winMax;
        m_qN[w]     = m_winSamples;

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
        m_periodCount = 0;
        m_sampleCount = 0;
        m_minSample = 1023;
        m_maxSample = 0;
    }
}

void SensorTrafoAC::update(uint32_t nowMs)
{
    bool consumedWindow = false;

    // Consume completed windows (at most a few).
    while (m_qCount)
    {
        // Atomically claim one window from the queue.
        uint32_t sum = 0;
        uint64_t sumSq = 0;
        uint16_t qmin = 1023;
        uint16_t qmax = 0;
        uint16_t qn = 0;
        noInterrupts();
        if (m_qCount) {
            const uint8_t r = m_qR;
            sum   = m_qSum[r];
            sumSq = m_qSumSq[r];
            qmin  = m_qMin[r];
            qmax  = m_qMax[r];
            qn    = m_qN[r];
            m_qR = (uint8_t)((r + 1u) % WIN_Q);
            m_qCount--;
        }
        interrupts();

        m_lastWindowSamples = qn;

        if (qn == 0u) {
            continue;
        }

        consumedWindow = true;
        m_lastGoodWindowMs = nowMs;

        // True RMS in ADC domain via variance:
        // var = E[x^2] - E[x]^2
        const float n = (float)qn;
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
 #if defined(MEGA2_DEBUG_TRAFO_RAW)
        Serial.print(F("[TRAW]["));
        Serial.print(m_debugLabel ? m_debugLabel : "?");
        Serial.print(F("] t="));
        Serial.print(millis());

        Serial.print(F(" adc="));
        Serial.print(vrmsAdc, 5);

        Serial.print(F(" trafo="));
        Serial.print(vrmsTrafo, 5);

        Serial.print(F(" min="));
        Serial.print(qmin);

        Serial.print(F(" max="));
        Serial.print(qmax);

        Serial.print(F(" n="));
        Serial.println(qn);
#endif

        // Median + asymmetrische Glättung:
        // - im Normalbetrieb wieder ruhiger/stabiler
        // - bei deutlich fallender Spannung weiterhin schneller nach unten
        // - nahe 0V weiterhin sehr schneller Rücklauf
        m_lastRms[m_lastRmsIdx] = vrmsTrafo;
        if (m_lastRmsCount < MEDIAN_N) m_lastRmsCount++;
        m_lastRmsIdx = (uint8_t)((m_lastRmsIdx + 1u) % MEDIAN_N);

        float tmp[MEDIAN_N] = {0};
        for (uint8_t i = 0; i < m_lastRmsCount; i++) tmp[i] = m_lastRms[i];
        const float med = medianOfUpTo3(tmp, m_lastRmsCount);

        const float oldTrafo = m_rmsFilteredTrafo;
        const float oldAdc   = m_rmsFilteredAdc;
        
        // ADC-domain median approximated from trafo median to keep both domains aligned.
        const float medAdc = (m_scale > 0.0001f) ? (med / m_scale) : vrmsAdc;

        // Adaptive post-filter:
        // - tiny deltas => calmer display
        // - medium deltas => moderate reaction
        // - large deltas / near zero => fast reaction
        const float deltaTrafo = fabsf(med - oldTrafo);
        const float deltaAdc   = fabsf(medAdc - oldAdc);

        auto chooseAlpha = [](float value, float delta, float snapZero, float smallDelta) -> float
        {
            if (value <= snapZero) return ADAPT_ALPHA_FAST;
            if (delta <= smallDelta) return ADAPT_ALPHA_SLOW;
            if (delta <= (smallDelta * 3.0f)) return ADAPT_ALPHA_MID;
            return ADAPT_ALPHA_FAST;
        };

        const float alphaTrafo = chooseAlpha(med,    deltaTrafo, ADAPT_SNAP_ZERO_TRAFO, ADAPT_SMALL_DELTA_TRAFO);
        const float alphaAdc   = chooseAlpha(medAdc, deltaAdc,   ADAPT_SNAP_ZERO_ADC,   ADAPT_SMALL_DELTA_ADC);

        float nextTrafo = oldTrafo + alphaTrafo * (med    - oldTrafo);
        float nextAdc   = oldAdc   + alphaAdc   * (medAdc - oldAdc);

        // Clean snap-to-zero after filtering so near-off regions do not "float".
        if (med <= ADAPT_SNAP_ZERO_TRAFO && nextTrafo < ADAPT_SNAP_ZERO_TRAFO) {
            nextTrafo = 0.0f;
        }
        if (medAdc <= ADAPT_SNAP_ZERO_ADC && nextAdc < ADAPT_SNAP_ZERO_ADC) {
            nextAdc = 0.0f;
        }

        // Slew limiter for NORMAL operation only:
        // limit visible step size per completed RMS window.
        // Keep large real changes responsive enough, but suppress visible jumps.
        // 2.5 V per 0.5 s => ~5 V/s, much more responsive than before.
        const float MAX_STEP_TRAFO = 2.50f; // V per window (~0.5 s)
        const float MAX_STEP_ADC   = (m_scale > 0.0001f) ? (MAX_STEP_TRAFO / m_scale) : 0.01f;

        const bool limitTrafo = (med > ADAPT_SNAP_ZERO_TRAFO) && (deltaTrafo <= (ADAPT_SMALL_DELTA_TRAFO * 6.0f));
        const bool limitAdc   = (medAdc > ADAPT_SNAP_ZERO_ADC) && (deltaAdc <= (ADAPT_SMALL_DELTA_ADC * 6.0f));

        if (limitTrafo) {
            const float d = nextTrafo - oldTrafo;
            if (d >  MAX_STEP_TRAFO) nextTrafo = oldTrafo + MAX_STEP_TRAFO;
            if (d < -MAX_STEP_TRAFO) nextTrafo = oldTrafo - MAX_STEP_TRAFO;
        }
        if (limitAdc) {
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

    // If no fresh RMS window arrives for a while, the AC measurement has
    // effectively gone inactive (e.g. trafo turned off and no more clean
    // zero-cross based windows complete). In that case, actively decay the
    // displayed value toward zero instead of holding the last value forever.
    //
    // 500 ms is long enough to avoid fighting the normal window cadence,
    // but short enough that the UI no longer appears "stuck on".
    if (!consumedWindow) {
        const uint32_t staleMs = (m_lastGoodWindowMs == 0u) ? 0u : (uint32_t)(nowMs - m_lastGoodWindowMs);
        if (m_lastGoodWindowMs != 0u && staleMs >= 500u) {
            const float DECAY = 0.55f;   // fairly quick visible drop
            const float SNAP_TO_ZERO_TRAFO = 0.15f;
            const float SNAP_TO_ZERO_ADC   = 0.01f;

            m_rmsFilteredTrafo *= (1.0f - DECAY);
            m_rmsFilteredAdc   *= (1.0f - DECAY);

            if (m_rmsFilteredTrafo < SNAP_TO_ZERO_TRAFO) m_rmsFilteredTrafo = 0.0f;
            if (m_rmsFilteredAdc   < SNAP_TO_ZERO_ADC)   m_rmsFilteredAdc   = 0.0f;

            m_rmsFilteredTrafo_cV = (uint16_t)lroundf(m_rmsFilteredTrafo * 100.0f);
            m_rmsFilteredAdc_mV   = (uint16_t)lroundf(m_rmsFilteredAdc   * 1000.0f);

            // Reset median history once we are stale, so the next power-up
            // starts clean and is not biased by old non-zero windows.
            if (m_rmsFilteredTrafo == 0.0f && m_rmsFilteredAdc == 0.0f) {
                memset(m_lastRms, 0, sizeof(m_lastRms));
                m_lastRmsCount = 0;
                m_lastRmsIdx = 0;
            }
        }
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