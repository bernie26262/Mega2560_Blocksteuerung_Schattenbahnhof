#pragma once
#include <Arduino.h>

// forward decls ...
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
    // Block 6 / Nothaltgleis-Logik:
    // S15: Nothaltgleis EIN (Freigabe), Zug darf passieren
    // S16: Nothaltgleis AUS (Nothalt aktiv), Stopzone wieder scharf
    // Hard-Error wird NICHT an S16 gekoppelt, sondern an Safety-Bedingung (EMERG_NOTHALT_SBHF).
    void onS15();   // NOT-AUS aus
    void onS16();   // NOT-AUS ein / Hard-Error

    // D3: Reset / Acknowledge
    void onResetAck();

    // Status (Debug / Proto)
    SBhfState state() const { return m_state; }
    uint8_t   ausfahrGleis() const { return m_currentGleis; }


// --------------------------------------------------------
// SBHF-Warnings / Restricted Mode
// --------------------------------------------------------
enum SbhfWarning : uint8_t
{
    SBHF_WARN_NONE           = 0x00,
    SBHF_WARN_RESTRICTED     = 0x01, // Betrieb eingeschränkt (allowedMask != 0b111)

    SBHF_WARN_W12_DEFECT     = 0x02,
    SBHF_WARN_W13_DEFECT     = 0x04,
    SBHF_WARN_W14_DEFECT     = 0x08,
    SBHF_WARN_W15_DEFECT     = 0x10,
};

// Bit0..2 => Gleis1..3 erlaubt
uint8_t allowedGleisMask() const { return m_allowedMask; }

// SBHF-Warnmaske (bits siehe SbhfWarning)
uint8_t warningMask() const { return m_warningMask; }

// --------------------------------------------------------
// Selbsttest (nach ACK bei SBHF-Weichenfehler)
// --------------------------------------------------------
// includeNonCritical=true => testet zusätzlich W14/W15 (Ergebnis erzeugt nur Warning)
bool startSelftest(bool includeNonCritical = true);

// Startup-Checklist Trigger: erlaubt Start auch aus sauberem Idle (z.B. Boot-ERR / SYS_ERROR_PRESENT),
// aber weiterhin NICHT bei aktivem HW-Notaus / Safety-Lock / wenn Selftest bereits läuft.
bool startSelftestStartup(bool includeNonCritical = true);

// UI Retry nach SBHF-Weichenfehler:
// Muss auch dann starten dürfen, wenn Safety gerade NOTAUS/LOCK aktiv hat,
// weil genau dieser Fehler sonst nicht auflösbar ist.
bool startSelftestRetry(bool includeNonCritical = true);

bool isSelftestActive() const { return m_selftestActive; }
bool isSelftestDone()   const { return m_selftestDone; }
void clearSelftestDone() { m_selftestDone = false; }

    // Wird aufgerufen, wenn Mega2 nach DIAG_TEST wieder in die Automatik geht.
    // Stellt im unkritischen Fall die Bereitschaftsroute für das nächste Gleis her.
    void onAutomationResumed(uint32_t nowMs);

    // Gate für Sensor-Dispatch (S11..S16):
    // true => keine Sensor-Events in die SBhf-Logik einspeisen (SafetyLock / Error / Selftest)
    bool isSafetyBlocked() const;

    // Modus (vorbereitet für ESP)
    void setMode(SbhfMode m) { m_mode = m; }


// ---------------- SBHF Restriction / Warning -------------
uint8_t m_allowedMask = 0x07;   // default: Gleis 1..3 erlaubt
uint8_t m_warningMask = 0x00;

// ---------------- Selftest -------------------------------
bool     m_selftestActive = false;
bool     m_selftestDone   = false;
bool     m_selftestIncludeNonCritical = true;
bool     m_selftestForcedPowerOff = false;
bool     m_stLoggedThisRun = false;

// Pipeline-Selftest: Impulse getaktet (PULSE_MS), Settling/Auswertung parallel
bool     m_stPulseActive   = false;
uint32_t m_stPulseStartMs  = 0;
uint8_t  m_stPulseIdx      = 0;   // welche Weiche pulst gerade (0..maxWeichen-1)


    // Pipeline-Selftest: Impulse sequentiell, Settling/Auswertung parallel
    enum SelftestPhase : uint8_t { ST_GERADE = 0, ST_ABBIEGEN = 1 };
    SelftestPhase m_selftestPhase = ST_GERADE;
    uint8_t  m_selftestNextIdx = 0; // welche Weiche als nächste einen Impuls bekommt (0..maxWeichen-1)

    struct SelftestTurnoutState {
        bool issuedGerade  = false;
        bool checkedGerade = false;
        bool okGerade      = false;
        uint32_t dueGeradeMs = 0;
        // expected RM bit at evaluation time: 1=Gerade, 0=Abbiegen
        uint8_t expGeradeBit = 0xFF;

        bool issuedAbbiegen  = false;
        bool checkedAbbiegen = false;
        bool okAbbiegen      = false;
        uint32_t dueAbbiegenMs = 0;
        uint8_t expAbbiegenBit = 0xFF;
    };
    SelftestTurnoutState m_stT[4];

// Ergebnisse pro Weiche (Index 0..3 => W12,W13,W14,W15)
bool     m_stOk[4]    = {false, false, false, false};
bool     m_stKnown[4] = {false, false, false, false};
uint8_t  m_stPos[4]   = {0,0,0,0};  // 0=GERADE, 1=ABBIEGEN (wenn known)
uint8_t  m_stChk[4]   = {0,0,0,0};  // bit0=Gerade ok, bit1=Abbiegen ok

void selftestUpdate(uint32_t nowMs);
void selftestFinish();

void setWarningForWeiche(uint8_t weicheId);
bool isCriticalWeiche(uint8_t weicheId) const;

// Shared implementation:
// allowFromCleanIdle=true => Idle-Start auch ohne Warnungen/Restriktionen (Startup-Checklist)
bool startSelftestImpl(bool includeNonCritical, bool allowFromCleanIdle, bool allowDuringEmergency);

    // --------------------------------------------------------
    // Weichen-Status für Proto (ersetzt g_weichen komplett)
    // Index 0..weichenCount()-1 entspricht der aktuellen Sequenz
    // --------------------------------------------------------
    uint8_t weichenCount() const { return m_weichenCount; }
    bool    weicheIst(uint8_t idx) const;
    bool    weicheSoll(uint8_t idx) const;

    // Reset-Guard für Safety/ACK: nur wenn SBHF in Error ist, Nothalt aus und keine aktive Ausfahrt
    bool canReset() const;
private:
    void applyReadyRouteForNextGleis(uint32_t nowMs);

    // ---------------- Gleiswahl ----------------
    uint8_t peekNextGleis() const;
    uint8_t pickNextGleis();
    uint8_t pickRandomGleisNoRepeat(uint8_t last);

    // ---------------- Weichen ------------------
    void buildWeichenPlan(uint8_t gleis);
    void startWeichenSequence(uint32_t nowMs);
    void processWeichenSequence(uint32_t nowMs);

    // ---------------- Fehler / Reset -----------
    void triggerHardError(uint8_t weicheId);
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

    // true => Weichensequenz nur für Bereitschaftsroute,
    // danach zurück nach Idle und KEINE Ausfahrt starten
    bool m_readyRouteOnly;

    // --------------------------------------------------------
    // Resume-Checkpoint (damit nach NOTAUS/ACK kein erneutes S11 nötig ist)
    // Wird bei HARD-ERROR gesetzt, wenn ein Run bereits gestartet war.
    // --------------------------------------------------------
    bool     m_resumePending;
    uint8_t  m_resumeGleis;
    SBhfState m_resumeState;
};
