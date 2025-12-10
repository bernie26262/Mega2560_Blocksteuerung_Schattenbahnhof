#include "SensorTrafoAC.h"

SensorTrafoAC::SensorTrafoAC(uint8_t pin)
    : m_pin(pin)
{
}

void SensorTrafoAC::begin()
{
    pinMode(m_pin, INPUT);
    m_lastCalc = millis();
}

void SensorTrafoAC::update(uint32_t now)
{
    int raw = analogRead(m_pin);

    if (raw < m_minSample) m_minSample = raw;
    if (raw > m_maxSample) m_maxSample = raw;
    m_sampleCount++;

    if (m_sampleCount < SAMPLE_WINDOW)
        return;

    // Peak-to-peak im ADC-Bereich
    int vpp_adc = m_maxSample - m_minSample;
    float vpp_volt = (vpp_adc * 5.0f) / 1023.0f;

    // Vrms ≈ Vpp / (2 * sqrt(2)) ≈ Vpp * 0.35355
    float vrms = vpp_volt * 0.35355f;

    // Exponentieller gleitender Mittelwert
    m_rmsFiltered = (m_rmsFiltered * 0.90f) + (vrms * 0.10f);

    m_minSample = 1023;
    m_maxSample = 0;
    m_sampleCount = 0;
    m_lastCalc = now;
}

void SensorTrafoAC::printDebug(const char* label) const
{
    Serial.print(label);
    Serial.print(" Vrms=");
    Serial.print(m_rmsFiltered, 2);
    Serial.print("V  powered=");
    Serial.println(isPowered() ? "YES" : "NO");
}
