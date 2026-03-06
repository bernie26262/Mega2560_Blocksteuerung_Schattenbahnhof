#include "SensorStrom.h"
#include <math.h>
#include <Arduino.h>
#include "AdcScheduler.h"

// local min helper (avoid std::min / Arduino min macro template confusion)
static inline uint32_t eeMinU32(uint32_t a, uint32_t b) { return (a < b) ? a : b; }

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
    m_rmsRaw = 0;
    m_rmsFiltered = 0;
    m_winCount = 0;
    m_winSum = 0;
    m_winSumSq = 0;
    m_hasInit = false;
}

void SensorStrom::setPowered(bool powered)
{
    m_powered = powered;
}


void SensorStrom::update()
{
    // Consume queued samples for this channel (provided by AdcScheduler ISR).
    // Note: if loop() is slow, the ring may drop samples (counted in adcSchedDropped()).

    bool gotSample = false;
    uint16_t v = 0;

    // Timing (used for idle detection / decay stability)
    const uint32_t nowMs = millis();
    const uint16_t dtMs = (m_lastMs == 0) ? 0 : (uint16_t)eeMinU32(1000u, nowMs - m_lastMs);
    m_lastMs = nowMs;

    // Window size in samples (non-overlapping windows like TrueRMS).
    // With 500 Hz sampling, 64 samples ~= 128 ms update rate (~7.8 Hz).
    static const uint16_t WIN_SAMPLES = 64;

    // Idle detection thresholds (counts)
    static const uint16_t IDLE_ABSDEV_MAX = 12;   // ~noise band
    static const uint16_t IDLE_RMS_MAX    = 18;   // ~noise band
    static const uint16_t IDLE_HOLD_MS = 100;  // must be quiet this long to be considered idle

    // Detect power edges to help the filter re-settle without "hard forcing" values.
    if (m_powered && !m_lastPowered) {
        // Rising edge: rail may jump. Restart windowing and baseline.
        m_hasInit = false;
        m_winCount = 0;
        m_winSum = 0;
        m_winSumSq = 0;
    }

    if (!m_powered && m_lastPowered) {
        // Falling edge: stop trusting previous AC energy; restart windowing.
        // Do NOT write artificial zeros; let filter decay naturally.
        m_winCount = 0;
        m_winSum = 0;
        m_winSumSq = 0;
    }

    // local helper: integer sqrt for 64-bit
    auto isqrt64 = [](uint64_t x) -> uint32_t {
        uint64_t op = x;
        uint64_t res = 0;
        uint64_t one = 1ULL << 62; // second-to-top bit
        while (one > op) one >>= 2;
        while (one != 0) {
            if (op >= res + one) {
                op -= res + one;
                res = (res >> 1) + one;
            } else {
                res >>= 1;
            }
            one >>= 2;
        }
        return (uint32_t)res;
    };

    while (adcSchedPop(m_pin, v))
    {
        gotSample = true;
        m_raw = v;

        if (!m_hasInit)
        {
            // Lazy init (first sample after (re)start)
            m_offset = v;
            m_absDev = 0;
            m_rmsRaw = 0;
            m_rmsFiltered = 0;
            m_noiseFloor = 0;
            m_idleMs = 0;
            m_winCount = 0;
            m_winSum = 0;
            m_winSumSq = 0;
            m_hasInit = true;
        }

        // Track abs deviation around last known offset (DC estimate).
        int16_t d = int16_t(v) - int16_t(m_offset);
        uint16_t ad = (d < 0) ? uint16_t(-d) : uint16_t(d);
        m_absDev = (uint16_t)((uint32_t(m_absDev) * 7u + ad) / 8u);

        // Accumulate window stats
        m_winCount++;
        m_winSum += (uint32_t)v;
        m_winSumSq += (uint64_t)v * (uint64_t)v;

        if (m_winCount >= WIN_SAMPLES)
        {
            const uint16_t n = m_winCount;
            const uint32_t mean = (uint32_t)((m_winSum + (n / 2u)) / n); // rounded
            const uint64_t ex2  = (m_winSumSq + (uint64_t)(n / 2u)) / (uint64_t)n;

            uint64_t mean2 = (uint64_t)mean * (uint64_t)mean;
            uint64_t var = (ex2 > mean2) ? (ex2 - mean2) : 0; // clamp
            uint16_t rms = (uint16_t)eeMinU32(65535u, isqrt64(var));

            m_offset = (uint16_t)eeMinU32(1023u, mean);
            m_rmsRaw = rms;

            // Idle detection / counters
            const bool idleCandidate = (m_powered && (m_absDev <= IDLE_ABSDEV_MAX) && (rms <= IDLE_RMS_MAX));
            if (idleCandidate && dtMs) {
                if (m_idleMs < 60000u) m_idleMs = (uint16_t)eeMinU32(60000u, (uint32_t)m_idleMs + dtMs);
            } else {
                m_idleMs = 0;
            }

            // Filter: in idle we want to converge faster back to baseline/noise.
            if (!m_powered) {
                // unpowered: very fast decay / fast settle to new low value
                m_rmsFiltered = (uint16_t)((uint32_t(m_rmsFiltered) + rms) / 3u);
            } else if (m_idleMs >= IDLE_HOLD_MS) {
                // powered + idle: faster decay back to near-zero
                m_rmsFiltered = (uint16_t)((uint32_t(m_rmsFiltered) + rms) / 3u);
            } else {
                // normal: still stable, but more responsive than before
                m_rmsFiltered = (uint16_t)((uint32_t(m_rmsFiltered) * 3u + rms) / 4u);
            }

            // Learn noise floor only in idle (slow IIR 1/32)
            if ((!m_powered) || (m_idleMs >= IDLE_HOLD_MS)) {
                m_noiseFloor = (uint16_t)((uint32_t(m_noiseFloor) * 3u + m_rmsFiltered) / 4u);
            }

            // Reset window (non-overlapping)
            m_winCount = 0;
            m_winSum = 0;
            m_winSumSq = 0;
        }
    }

    // If no new samples arrived this tick, ensure the value still relaxes.
    if (!gotSample)
    {
        if (!m_powered) {
            m_absDev = (uint16_t)((uint32_t(m_absDev)) / 3u);
            m_rmsFiltered = (uint16_t)((uint32_t(m_rmsFiltered)) / 3u);
        } else {
            if (m_idleMs >= IDLE_HOLD_MS) {
                m_absDev = (uint16_t)((uint32_t(m_absDev)) / 2u);
                m_rmsFiltered = (uint16_t)((uint32_t(m_rmsFiltered)) / 3u);
            } else {
                m_absDev = (uint16_t)((uint32_t(m_absDev) * 3u) / 4u);
                m_rmsFiltered = (uint16_t)((uint32_t(m_rmsFiltered) * 2u) / 3u);
            }
        }
    }

    m_lastPowered = m_powered;
}


uint16_t SensorStrom::raw() const { return m_raw; }
uint16_t SensorStrom::offset() const { return m_offset; }
uint16_t SensorStrom::absDevCounts() const { return m_absDev; }

uint16_t SensorStrom::rmsCounts() const
{
    // noise-floor subtracted filtered RMS counts
    const uint16_t v = m_rmsFiltered;
    if (v <= m_noiseFloor) return 0;
    return (uint16_t)(v - m_noiseFloor);
}

uint16_t SensorStrom::rmsCountsRaw() const
{
    return m_rmsRaw;
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