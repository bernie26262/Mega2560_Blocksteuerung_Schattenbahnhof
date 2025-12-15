#pragma once
#include <Arduino.h>

class BlockController;
class Weiche;

// ============================================================
// Schattenbahnhof – Zustände
// ============================================================
enum class SBhfState : uint8_t {
    Idle = 0,
    PrepareExit,
    SettingWeichen,
    WaitBlock6,
    ExitRunning,
    Error
};

enum class SbhfMode : uint8_t {
    Sequential = 0,
    Random
};

// ============================================================
// ShadowYardController
// ============================================================
class ShadowYardController
{
public:
    explicit ShadowYardController(BlockController* bc);

    void begin();
    void update(uint32_t nowMs);

    // Events (PulseSensor oder Debug)
    void onS11();
    void onS12();
    void onS13();
    void onS14();
    void onS15();   // NOT-AUS EIN
    void onS16();   // NOT-AUS AUS / Hard-Error

    // D3: Reset / Acknowledge
    void onResetAck();

    // Status (Debug / Proto)
    SBhfState state() const { return m_state; }
    uint8_t   ausfahrGleis() const { return m_currentGleis; }

    // Modus (vorbereitet für ESP)
    void setMode(SbhfMode m) { m_mode = m; }

    // --------------------------------------------------------
    // Weichen-Status für Proto (ersetzt g_weichen komplett)
    // Index 0..weichenCount()-1 entspricht der aktuellen Sequenz
    // --------------------------------------------------------
    uint8_t weichenCount() const { return m_weichenCount; }
    bool    weicheIst(uint8_t idx) const;
    bool    weicheSoll(uint8_t idx) const;

private:
    // ---------------- Gleiswahl ----------------
    uint8_t pickNextGleis();
    uint8_t pickRandomGleisNoRepeat(uint8_t last);

    // ---------------- Weichen ------------------
    void buildWeichenPlan(uint8_t gleis);
    void startWeichenSequence(uint32_t nowMs);
    void processWeichenSequence(uint32_t nowMs);

    // ---------------- Fehler / Reset -----------
    void triggerHardError();
    bool canReset() const;
    void resetError();

private:
    BlockController* m_bc;

    SBhfState m_state;
    SbhfMode  m_mode;

    uint8_t m_currentGleis;   // 1..3, 0 = none
    uint8_t m_nextGleis;      // serieller Modus

    // Weichen-Sequencer
    static constexpr uint8_t MAX_WEICHEN = 4; // Gleis 2 hat 4 Weichen (W12,W13,W14,W15)
    Weiche*  m_weichen[MAX_WEICHEN];
    bool     m_weichenSollAbzweig[MAX_WEICHEN];
    uint8_t  m_weichenCount;
    uint8_t  m_weichenIndex;

    enum class WPhase : uint8_t { Idle, Impuls, Check };
    WPhase   m_wphase;
    uint32_t m_phaseStartMs;

    // Flags
    bool m_errorActive;
    bool m_exitPowerOn;
};
