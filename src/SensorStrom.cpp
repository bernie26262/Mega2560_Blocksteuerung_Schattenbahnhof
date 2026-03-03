#include "SensorStrom.h"
#include <math.h>
#include <Arduino.h>
#include "AdcScheduler.h"

SensorStrom::SensorStrom(uint8_t pin,
                         uint16_t thresholdCounts,
                         uint16_t mvPerAmp,
                         uint16_t vref_mV,
                         uint16_t adcMax)
    : m_pin(pin)
    , m_thresholdCounts(thresholdCounts)
    , m_threshold_mA(0)
    , m_mvPerAmp(mvPerAmp)
    , m_vref_mV(vref_mV)
    , m_adcMax(adcMax)
{}

void SensorStrom::begin()
{
    pinMode(m_pin, INPUT);

    // ADC is sampled by AdcScheduler (timer/ISR). Do not call analogRead() here.
    // Initialize lazily on first received sample.
    m_raw = 0;
    m_offset = 0;
    m_absDev = 0;
    m_rms2 = 0;
    m_hasInit = false;
}

void SensorStrom::update()
{
    // Consume queued samples for this channel.
    // Note: if loop() is slow, the ring may drop samples (counted in adcSchedDropped()).
    uint16_t v = 0;
    while (adcSchedPop(m_pin, v))
    {
        m_raw = v;

        if (!m_hasInit)
        {
            // Lazy init (first sample)
            m_offset = v;
            m_hasInit = true;
        }

        // Offset very slowly tracks drift (alpha ~ 1/256)
        m_offset = (uint16_t)((uint32_t(m_offset) * 255u + v) / 256u);

        // Deviation around offset (absolute)
        int16_t d = int16_t(v) - int16_t(m_offset);
        uint16_t ad = (d < 0) ? uint16_t(-d) : uint16_t(d);

        // absDev smoothing
        m_absDev = (uint16_t)((uint32_t(m_absDev) * 7u + ad) / 8u);

        // RMS of deviation (EMA over d^2)
        // rms2 = (15/16)*rms2 + (1/16)*(d^2)
        const int32_t di = (int32_t)d;
        const uint32_t d2 = (uint32_t)(di * di);
        m_rms2 = (m_rms2 * 15u + d2) / 16u;
    }
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