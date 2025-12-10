#pragma once
#include <Arduino.h>

/*
   SensorTrafoAC – AC-Spannungsmessung mit ZMPT101B

   - misst AC-Spannung 0–30V (Märklin)
   - nutzt Peak-to-Peak (ADC) → RMS-Annäherung
   - gleitende Filterung ohne blocking
   - liefert:
       * rms()        → gefilterte Spannung
       * isPowered()  → Trafo EIN/AUS (über Threshold)
*/

class SensorTrafoAC {
public:
    explicit SensorTrafoAC(uint8_t pin);

    void begin();
    void update(uint32_t now);

    float rms() const { return m_rmsFiltered; }
    bool isPowered() const { return m_rmsFiltered > m_powerThreshold; }

    void setPowerThreshold(float v) { m_powerThreshold = v; }

    void printDebug(const char* label) const;

private:
    uint8_t m_pin;

    int16_t  m_minSample   = 1023;
    int16_t  m_maxSample   = 0;
    uint16_t m_sampleCount = 0;

    uint32_t m_lastCalc = 0;

    float m_rmsFiltered   = 0.0f;
    float m_powerThreshold = 2.0f; // ca. 2V RMS = "Trafo EIN"

    static constexpr uint16_t SAMPLE_WINDOW     = 200;
    static constexpr uint16_t CALC_INTERVAL_MS  = 20;
};
