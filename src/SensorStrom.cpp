#include "SensorStrom.h"
#include <math.h>
#include <Arduino.h>

SensorStrom::SensorStrom(uint8_t pin,
                         uint16_t thresholdCounts,
                         uint16_t mvPerAmp,
                         uint16_t vref_mV,
                         uint16_t adcMax)
    : m_pin(pin),
      m_thresholdCounts(thresholdCounts),
      m_mvPerAmp(mvPerAmp),
      m_vref_mV(vref_mV),
      m_adcMax(adcMax),
      m_threshold_mA(0),
      m_scaleNum(0),
      m_scaleDen(0)
{}

void SensorStrom::begin()
{
    pinMode(m_pin, INPUT);

    // Initiale Offset-Kalibrierung: Mittelwert über kurze Zeit.
    uint32_t sum = 0;
    const uint16_t N = 64;
    for (uint16_t i = 0; i < N; i++)
    {
        sum += analogRead(m_pin);
        delayMicroseconds(200);
    }

    m_raw = (uint16_t)(sum / N);
    m_offset = m_raw;
    m_absDev = 0;
    m_rms2 = 0;
    m_hasInit = true;
}

void SensorStrom::update()
{
    const uint16_t v = analogRead(m_pin);
    m_raw = v;

    if (!m_hasInit)
    {
        // Falls begin() vergessen wurde: sanft initialisieren.
        m_offset = v;
        m_hasInit = true;
    }

    // Offset sehr langsam nachführen (Drift / Vcc Änderungen):
    // alpha_offset ~ 1/256
    m_offset = (uint16_t)((uint32_t(m_offset) * 255u + v) / 256u);

    // Abweichung um Offset (Betrag)
    int16_t d = int16_t(v) - int16_t(m_offset);
    uint16_t ad = (d < 0) ? uint16_t(-d) : uint16_t(d);

    // absDev glätten (etwas schneller)
    m_absDev = (uint16_t)((uint32_t(m_absDev) * 7u + ad) / 8u);

    // RMS der Abweichung (EMA über d^2)
    // rms2 = (15/16)*rms2 + (1/16)*(d^2)
    const uint32_t d2 = uint32_t(d) * uint32_t(d);
    m_rms2 = (m_rms2 * 15u + d2) / 16u;
}

uint16_t SensorStrom::raw() const { return m_raw; }
uint16_t SensorStrom::offset() const { return m_offset; }
uint16_t SensorStrom::absDevCounts() const { return m_absDev; }

uint16_t SensorStrom::rmsCounts() const
{
    // sqrt(m_rms2) – hier bewusst double, damit AVR-lib passt
    return (uint16_t)sqrt((double)m_rms2);
}

bool SensorStrom::overThreshold() const
{
    // Prefer mA threshold if configured AND scaling is available
    if (m_threshold_mA > 0) {
        const uint16_t ma = rms_mA();
        if (ma > 0) return ma >= m_threshold_mA;
        // If scaling not available yet, fall back to counts threshold
    }
    return rmsCounts() >= m_thresholdCounts;
}

void SensorStrom::setThresholdCounts(uint16_t t)
{
    m_thresholdCounts = t;
}

void SensorStrom::setThreshold_mA(uint16_t t)
{
    m_threshold_mA = t;
}

void SensorStrom::setScaleCountsToMA(uint16_t num, uint16_t den)
{
    if (den == 0) { m_scaleNum = 0; m_scaleDen = 0; return; }
    m_scaleNum = num;
    m_scaleDen = den;
}

uint16_t SensorStrom::rms_mA() const
{
    // Preferred: fixed-point counts->mA scaling (configured via setScaleCountsToMA()).
    if (m_scaleDen != 0)
    {
        const uint32_t mA = (uint32_t(rmsCounts()) * uint32_t(m_scaleNum) + (uint32_t(m_scaleDen) / 2u)) / uint32_t(m_scaleDen);
        return (uint16_t)((mA > 65535u) ? 65535u : mA);
    }

    // Fallback: derive mA from mvPerAmp (if configured).
    if (m_mvPerAmp == 0) return 0;

    // Counts -> mV (RMS-Abweichung)
    const uint32_t mv = (uint32_t)rmsCounts() * m_vref_mV / m_adcMax;

    // mV / (mV/A) = A
    const uint32_t mA = (mv * 1000u) / m_mvPerAmp;

    return (uint16_t)((mA > 65535u) ? 65535u : mA);
}

uint16_t SensorStrom::measureRmsCountsBlocking(uint16_t freqHz, uint8_t periods) const
{
    if (freqHz == 0) freqHz = 50;
    if (periods == 0) periods = 1;

    const uint32_t periodUs = 1000000UL / (uint32_t)freqHz;
    const uint32_t totalUs  = periodUs * (uint32_t)periods;

    // --- Phase 1: Zeropoint (DC offset) as mean over totalUs
    uint32_t sum = 0;
    uint32_t n   = 0;

    const uint32_t t0 = micros();
    while ((uint32_t)(micros() - t0) < totalUs) {
        sum += (uint16_t)analogRead(m_pin);
        n++;
    }
    if (n == 0) return 0;
    const uint16_t zero = (uint16_t)(sum / n);

    // --- Phase 2: Mean square over totalUs (deviation around zero)
    uint32_t sumSq = 0;
    uint32_t n2    = 0;

    const uint32_t t1 = micros();
    while ((uint32_t)(micros() - t1) < totalUs) {
        const int32_t d = (int32_t)(uint16_t)analogRead(m_pin) - (int32_t)zero;
        sumSq += (uint32_t)(d * d);
        n2++;
    }
    if (n2 == 0) return 0;

    // RMS = sqrt(mean(d^2))
    const uint32_t meanSq = sumSq / n2;
    return (uint16_t)sqrt((double)meanSq);
}