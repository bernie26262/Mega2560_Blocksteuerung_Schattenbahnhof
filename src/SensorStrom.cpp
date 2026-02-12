#include "SensorStrom.h"
#include <math.h>

SensorStrom::SensorStrom(uint8_t pin,
                         uint16_t thresholdCounts,
                         uint16_t mvPerAmp,
                         uint16_t vref_mV,
                         uint16_t adcMax)
    : m_pin(pin),
      m_thresholdCounts(thresholdCounts),
      m_mvPerAmp(mvPerAmp),
      m_vref_mV(vref_mV),
      m_adcMax(adcMax)
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
    return rmsCounts() >= m_thresholdCounts;
}

void SensorStrom::setThresholdCounts(uint16_t t)
{
    m_thresholdCounts = t;
}

uint16_t SensorStrom::rms_mA() const
{
    if (m_mvPerAmp == 0) return 0;

    // Counts -> mV (RMS-Abweichung)
    const uint32_t mv = (uint32_t)rmsCounts() * m_vref_mV / m_adcMax;

    // mV / (mV/A) = A
    const uint32_t mA = (mv * 1000u) / m_mvPerAmp;

    return (uint16_t)((mA > 65535u) ? 65535u : mA);
}