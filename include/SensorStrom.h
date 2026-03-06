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

    // Provide power context (per-trafo). Used to improve settling / avoid stale values
    // when the rail is unpowered or has just changed state.
    void setPowered(bool powered);

    uint16_t raw() const;
    uint16_t offset() const;
    uint16_t absDev() const { return m_absDev; }
    uint16_t absDevCounts() const;
    uint16_t rmsCounts() const;
    uint16_t rmsCountsRaw() const; // debug (no noise-floor subtraction)
    bool overThreshold() const;

    void setThresholdCounts(uint16_t t);
    void setThreshold_mA(uint16_t t);

    // Fixed-point scaling: i_mA = (rmsCounts * num + den/2) / den
    void setScaleCountsToMA(uint16_t num, uint16_t den);

    uint16_t rms_mA() const;

private:
    uint8_t  m_pin;
    uint16_t m_thresholdCounts;
    uint16_t m_threshold_mA = 0;

    // Optional scaling
    uint16_t m_mvPerAmp;
    uint16_t m_vref_mV;
    uint16_t m_adcMax;

    // Fixed-point counts->mA scaling (preferred). If m_scaleDen==0 => disabled.
    uint16_t m_scaleNum = 0;
    uint16_t m_scaleDen = 0;

    // State
    bool     m_powered = true;
    bool     m_lastPowered = true;
    bool     m_hasInit = false;
    uint16_t m_raw = 0;
    
    // Baseline (DC) estimate and instantaneous deviation
    uint16_t m_offset = 0;   // interpreted as DC bias (mean of last window)
    uint16_t m_absDev = 0;   // smoothed abs(sample-offset)

    // Windowed true-RMS (AC component) in ADC counts (not squared).
    uint16_t m_rmsRaw = 0;        // latest computed (per window)
    uint16_t m_rmsFiltered = 0;   // display/threshold (EMA)

    // Window accumulators (non-overlapping windows, like TrueRMS)
    uint16_t m_winCount = 0;
    uint32_t m_winSum = 0;        // sum(samples)
    uint64_t m_winSumSq = 0;      // sum(samples^2)

    // Learned noise floor (counts). Updated only in idle.
    uint16_t m_noiseFloor = 0;
    uint16_t m_idleMs = 0;
    uint32_t m_lastMs = 0;
};