#pragma once
#include <Arduino.h>

class SensorStrom {
public:
    // thresholdCounts bezieht sich auf RMS der Abweichung vom Offset (nicht auf raw ADC).
    // mvPerAmp optional (z.B. ACS712: 185/100/66 mV/A je nach Typ). 0 => keine Umrechnung.
    // Für ZMCT103C (5A CT) sind kleine Signalpegel typisch -> Default niedriger.
    explicit SensorStrom(uint8_t pin,
                         uint16_t thresholdCounts = 8,
                         uint16_t mvPerAmp = 0,
                         uint16_t vref_mV = 5000,
                         uint16_t adcMax = 1023);

    void begin();
    void update();

    uint16_t raw() const;
    uint16_t offset() const;
    uint16_t absDevCounts() const;
    uint16_t rmsCounts() const;

    bool overThreshold() const;

    void setThresholdCounts(uint16_t t);
    uint16_t rms_mA() const;

private:
    uint8_t  m_pin;
    uint16_t m_thresholdCounts;

    // Optional scaling
    uint16_t m_mvPerAmp;
    uint16_t m_vref_mV;
    uint16_t m_adcMax;

    // State
    bool     m_hasInit = false;
    uint16_t m_raw = 0;
    uint16_t m_offset = 0;
    uint16_t m_absDev = 0;
    uint32_t m_rms2 = 0; // EMA von d^2
};
