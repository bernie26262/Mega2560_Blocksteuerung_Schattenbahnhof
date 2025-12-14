#pragma once

#include <Arduino.h>

class BlockController;
class Weiche;

class ShadowYardController
{
public:
    enum class SBhfState : uint8_t {
        Idle,
        PrepareExit,
        SettingWeichen,
        WaitBlock6,
        ExitRunning,
        Error
    };

    enum class SbhfMode : uint8_t {
        Sequential,
        Random
    };

    enum class HardErrorReason : uint8_t {
        NONE = 0,
        WEICHE_SOLL_IST,
        WEICHE_TIMEOUT,
        NOTHALT_TRIGGERED,
        UNEXPECTED_EVENT
    };

    ShadowYardController(BlockController* bc);

    void begin();
    void update(uint32_t nowMs);

    // Events
    void onS11();
    void onS12();
    void onS13();
    void onS14();
    void onS15();
    void onS16();

    // 🔹 Expliziter Reset nach Hard-Error
    void resetError();

    // 🔹 Getter (Debug / Status / D3)
    SBhfState state() const { return m_state; }
    uint8_t ausfahrGleis() const { return m_currentGleis; }

    HardErrorReason getLastError() const { return m_lastError; }
    uint8_t getLastErrorWeiche() const { return m_lastErrorWeiche; }

private:
    uint8_t pickNextGleis();
    uint8_t pickRandomGleisNoRepeat(uint8_t last);

    void buildWeichenPlan(uint8_t gleis);

    void startWeichenSequence(uint32_t nowMs);
    void processWeichenSequence(uint32_t nowMs);

    void triggerHardError(HardErrorReason reason, uint8_t weichenId = 0);

private:
    BlockController* m_bc = nullptr;

    SBhfState m_state = SBhfState::Idle;
    SbhfMode  m_mode  = SbhfMode::Sequential;

    uint8_t m_currentGleis = 0;
    uint8_t m_nextGleis    = 0;

    static constexpr uint8_t MAX_WEICHEN = 6;
    Weiche* m_weichen[MAX_WEICHEN];
    bool    m_weichenSollAbzweig[MAX_WEICHEN];
    uint8_t m_weichenCount = 0;
    uint8_t m_weichenIndex = 0;

    uint32_t m_phaseStartMs = 0;

    bool m_errorActive   = false;
    bool m_exitPowerOn   = false;
    bool m_nothaltActive = false;
    bool m_entryArmed    = false;

    HardErrorReason m_lastError = HardErrorReason::NONE;
    uint8_t m_lastErrorWeiche  = 0;
};
