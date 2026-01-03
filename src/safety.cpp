#include "safety.h"

#include <Arduino.h>

#include "BlockController.h"
#include "Mega2PowerControl.h"
#include "SensorTrafoAC.h"
#include "SensorKontakt.h"
#include "ShadowYardController.h"

#include "safety_error.h"
#include "mega2_debug.h"

// ============================================================================
// Externe Abhängigkeiten (aus main.cpp)
// ============================================================================
extern Mega2PowerControl g_power;
extern BlockController   g_bc;
extern SensorTrafoAC     g_trafoOben;
extern SensorTrafoAC     g_trafoUnten;

// Stopzone / Nothalt-Kontakt (Block 6 extra Kontaktgleis)
extern SensorKontakt     k_nothalt;

// SBHF Controller (für Weichenfehler-Reset/Retry)
extern ShadowYardController& shadowController;

// ============================================================================
// Konfiguration / Thresholds
// ============================================================================
static constexpr uint32_t SHORT_DETECT_MS      = 200;
static constexpr uint16_t SHORT_THRESHOLD_MA   = 1800;
static constexpr uint16_t NO_CURRENT_MA        = 100;

// Doppelte Blockbelegung (heuristisch via Strom)
static constexpr uint16_t DOUBLE_OCC_THRESHOLD_MA = 1200; // TODO: kalibrieren
static constexpr uint32_t DOUBLE_OCC_DETECT_MS    = 600;
static constexpr uint32_t POWER_STABLE_MS         = 400;  // nach SSR-Schaltvorgängen keine Fehltrigger

// Safety-Controller Fault (Watchdog / Invariants)
static constexpr uint32_t SAFETY_TICK_MAX_GAP_MS  = 1000;

// SBHF Weichenfehler: ACK->Retry Timeout
static constexpr uint32_t WEICHE_RETRY_TIMEOUT_MS = 4000;
static constexpr uint32_t SSR_STUCK_DELAY_MS   = 250;

// ============================================================================
// Status
// ============================================================================
static bool              s_emergencyActive = false;
static bool              s_lock            = true; // Boot-Lock bis ACK
static SafetyBlockReason s_blockReason     = SAFETY_BLOCK_BOOT;

static uint32_t          s_lastPowerOnMs   = 0;
static uint32_t          s_lastSsrBOffMs   = 0;

static uint32_t          s_lastPowerSwitchMs = 0;
static uint32_t          s_lastSafetyUpdateMs = 0;
static uint32_t          s_doubleOccStartMs[16] = {0};

// SBHF Weichenfehler: ACK->Retry
static bool              s_weicheRetryPending = false;
static uint32_t          s_weicheRetryStartMs = 0;

// Reverse-Entry / Nothalt-Kontakt-Fall (SBHF soll weiterlaufen, Reset nur Safety)
static bool              s_emergNothaltSbhfLatched = false;

// SIM helper
#if MEGA2_SIM_MODE
static bool s_forceTrafoUntenPowered = false;
#endif

// ============================================================================
// Public HW-API (muss mit safety.h konsistent sein)
// ============================================================================
void safetySetSSR(SafetySSR ssr, bool enable)
{
    // Track SSR switching to avoid false-trigger safety heuristics right after power changes.
    const bool before = safetyIsSSR(ssr);
    if (before != enable)
        s_lastPowerSwitchMs = millis();

    switch (ssr)
    {
        case SSR_MAIN_ENABLE: g_power.setMainPower(enable);  break;
        case SSR_TRAFO_A:     g_power.setSsrTrafoA(enable);  break;
        case SSR_TRAFO_B:     g_power.setSsrTrafoB(enable);  break;
        default: break;
    }
}

bool safetyIsSSR(SafetySSR ssr)
{
    switch (ssr)
    {
        case SSR_MAIN_ENABLE: return g_power.isMainPowerOn();
        case SSR_TRAFO_A:     return g_power.isSsrTrafoA();
        case SSR_TRAFO_B:     return g_power.isSsrTrafoB();
        default: return false;
    }
}

bool safetyIsPowerOn()
{
    return g_power.isMainPowerOn();
}

static inline bool isTrafoObenPowered()
{
    // AC-Spannung oben wird aktuell nur für Diagnose genutzt
    return g_trafoOben.isPowered();
}

static inline bool isTrafoUntenPowered()
{
#if MEGA2_SIM_MODE
    if (s_forceTrafoUntenPowered) return true;
#endif
    return g_trafoUnten.isPowered();
}

// ============================================================================
// Public API
// ============================================================================
void safetyBegin()
{
    safetyErrorClear();

    s_emergencyActive = false;
    s_lock            = true;
    s_blockReason     = SAFETY_BLOCK_BOOT;

    s_lastPowerOnMs   = 0;
    s_lastSsrBOffMs   = 0;
    s_emergNothaltSbhfLatched = false;
}

bool safetyIsEmergencyActive() { return s_emergencyActive; }
bool safetyIsLocked()          { return s_lock; }
uint8_t safetyGetBlockReason() { return static_cast<uint8_t>(s_blockReason); }

// Debug (SIM)
#if MEGA2_SIM_MODE
void safetyDebugForceTrafoUntenPowered(bool on) { s_forceTrafoUntenPowered = on; }
#else
void safetyDebugForceTrafoUntenPowered(bool) {}
#endif

void safetySetEmergency(bool on)
{
    if (on == s_emergencyActive) return;

    s_emergencyActive = on;

    if (on)
    {
        // Generischer Emergency-Einstieg: Details (Reason/Index) kommen über safetyErrorSet()
        // von den jeweiligen Triggern. Nur wenn kein Fehler gesetzt ist, wird NOTAUS angenommen.
        if (s_blockReason == SAFETY_BLOCK_NONE || s_blockReason == SAFETY_BLOCK_BOOT)
            s_blockReason = SAFETY_BLOCK_EMERGENCY;

        s_lock = true;

        // Alles aus
        safetySetSSR(SSR_MAIN_ENABLE, false);
        safetySetSSR(SSR_TRAFO_A,     false);
        safetySetSSR(SSR_TRAFO_B,     false);

        if (!safetyErrorActive())
            safetyErrorSet(SAFETY_ERR_NOTAUS, 0);
    }
    else
    {
        // Reset erfolgt via safetyResetEmergency()
    }
}

bool safetyPowerOn()
{
    // blockiert wenn Safety-Lock aktiv
    if (s_lock)
        return false;

    safetySetSSR(SSR_MAIN_ENABLE, true);
    s_lastPowerOnMs = millis();
    return true;
}

bool safetyResetEmergency()
{
    if (!s_lock) return true;

    const SafetyErrorInfo& err = safetyErrorGet();

    // ------------------------------------------------------------
    // 1) Block-Short: ACK nur wenn Strom wieder 0 ist
    // ------------------------------------------------------------
    if (err.type == SAFETY_ERR_BLOCK_SHORT)
    {
        const uint8_t b = (err.index >= 1 && err.index <= g_bc.count()) ? err.index : 6;
        const uint16_t i = g_bc.stromFiltered(b);
        if (i > NO_CURRENT_MA)
        {
            DBG_PRINTF("[SAFETY] ACK blocked (SHORT): B%d current=%umA
", b, i);
            return false;
        }

        safetyErrorClear();
        s_emergencyActive = false;
        s_lock            = false;
        s_blockReason     = SAFETY_BLOCK_NONE;
        return true;
    }

    // ------------------------------------------------------------
    // 2) SSR-Stuck: ACK nur wenn Trafo unten wirklich "aus" ist
    // ------------------------------------------------------------
    if (err.type == SAFETY_ERR_SSR_STUCK)
    {
        if (isTrafoUntenPowered())
        {
            DBG_PRINTLN("[SAFETY] ACK blocked (SSR_STUCK): Trafo unten still powered");
            return false;
        }

        safetyErrorClear();
        s_emergencyActive = false;
        s_lock            = false;
        s_blockReason     = SAFETY_BLOCK_NONE;
        return true;
    }

    // ------------------------------------------------------------
    // 3) NOTAUS (Reverse-Entry / Stopzone Kontakt)
    //    ACK nur wenn Kontaktgleis frei ist.
    //    (SBHF-Reset NICHT automatisch!)
    // ------------------------------------------------------------
    if (err.type == SAFETY_ERR_NOTAUS && s_emergNothaltSbhfLatched)
    {
        if (k_nothalt.raw())
        {
            DBG_PRINTLN("[SAFETY] ACK blocked (NOTAUS): Stopzone contact still occupied");
            return false;
        }

        s_emergNothaltSbhfLatched = false;

        safetyErrorClear();
        s_emergencyActive = false;
        s_lock            = false;
        s_blockReason     = SAFETY_BLOCK_NONE;
        return true;
    }

    // ------------------------------------------------------------
    // 4) SBHF Weichenfehler:
    //    ACK startet Retry über SBHF-Reset, bleibt aber solange im Emergency,
    //    bis der SBHF wieder stabil läuft (Freigabe in safetyUpdate()).
    // ------------------------------------------------------------
    if (err.type == SAFETY_ERR_SBH_WEICHE)
    {
        if (!shadowController.canReset())
        {
            DBG_PRINTLN("[SAFETY] ACK blocked (SBH_WEICHE): SBHF reset conditions not met");
            return false;
        }

        shadowController.onResetAck();
        s_weicheRetryPending = true;
        s_weicheRetryStartMs = millis();

        DBG_PRINTLN("[SAFETY] ACK accepted (SBH_WEICHE): retry started");
        return true;
    }

    // ------------------------------------------------------------
    // 5) Doppelte Blockbelegung: ACK nur wenn Strom wieder 0 ist
    // ------------------------------------------------------------
    if (err.type == SAFETY_ERR_DOUBLE_OCCUPANCY)
    {
        const uint8_t b = (err.index >= 1 && err.index <= g_bc.count()) ? err.index : 0;
        const uint16_t i = (b ? g_bc.stromFiltered(b) : 0);

        if (b && i > NO_CURRENT_MA)
        {
            DBG_PRINTF("[SAFETY] ACK blocked (DOUBLE_OCC): B%d current=%umA
", b, i);
            return false;
        }

        safetyErrorClear();
        s_emergencyActive = false;
        s_lock            = false;
        s_blockReason     = SAFETY_BLOCK_NONE;
        return true;
    }

    // ------------------------------------------------------------
    // 6) Safety-Controller Fault: Reinit Safety (Boot-Lock)
    // ------------------------------------------------------------
    if (err.type == SAFETY_ERR_CONTROLLER_FAULT)
    {
        DBG_PRINTLN("[SAFETY] ACK accepted (CTRL_FAULT): reinit safety subsystem (boot-lock)");

        safetyErrorClear();
        s_emergencyActive = false;
        s_lock            = true;
        s_blockReason     = SAFETY_BLOCK_BOOT;

        s_emergNothaltSbhfLatched = false;
        s_weicheRetryPending      = false;
        return true;
    }

    // ------------------------------------------------------------
    // Default: Boot-Lock / unbekannt -> freigeben
    // ------------------------------------------------------------
    safetyErrorClear();
    s_emergencyActive = false;
    s_lock            = false;
    s_blockReason     = SAFETY_BLOCK_NONE;
    return true;
}

void safetyTriggerBlockShort(uint8_t block)
{
    (void)block;

    // Kurzschluss: SSR_A/B aus, Lock
    safetySetSSR(SSR_MAIN_ENABLE, false);
    safetySetSSR(SSR_TRAFO_A,     false);
    safetySetSSR(SSR_TRAFO_B,     false);

    s_emergencyActive = true;
    s_lock            = true;
    s_blockReason     = SAFETY_BLOCK_SHORT;

    safetyErrorSet(SAFETY_ERR_BLOCK_SHORT, block);

    DBG_PRINT(F("[SAFETY] EMERG_BLOCK_SHORT(B"));
    DBG_PRINT(block);
    DBG_PRINTLN(F(") -> SSR_A/B OFF, LOCK"));
}

void safetyUpdate()
{
    const uint32_t now = millis();

    // ------------------------------------------------------------
    // Safety-Controller Fault: Watchdog / Invariants
    // ------------------------------------------------------------
    if (!s_emergencyActive)
    {
        if (s_lastSafetyUpdateMs != 0 && (now - s_lastSafetyUpdateMs) > SAFETY_TICK_MAX_GAP_MS)
        {
            safetyErrorSet(SAFETY_ERR_CONTROLLER_FAULT, 1); // tick gap
            safetySetEmergency(true);
            DBG_PRINTLN("[SAFETY] EMERG_CONTROLLER_FAULT(tick-gap) -> ALL OFF, LOCK");
            s_lastSafetyUpdateMs = now;
            return;
        }

        // einfache Invariants (nur Beispiele; erweitert man bei Bedarf)
        if (s_lock && s_blockReason == SAFETY_BLOCK_NONE)
        {
            safetyErrorSet(SAFETY_ERR_CONTROLLER_FAULT, 2); // inkonsistent
            safetySetEmergency(true);
            DBG_PRINTLN("[SAFETY] EMERG_CONTROLLER_FAULT(invariant) -> ALL OFF, LOCK");
            s_lastSafetyUpdateMs = now;
            return;
        }
    }
    s_lastSafetyUpdateMs = now;

    // ------------------------------------------------------------
    // SBHF Weichenfehler: Retry-Freigabe nach ACK
    // ------------------------------------------------------------
    if (s_weicheRetryPending && s_emergencyActive && safetyErrorGet().type == SAFETY_ERR_SBH_WEICHE)
    {
        const SBhfState st = shadowController.state();

        // Erfolg: Weichen-Setup abgeschlossen und SBHF läuft wieder
        if (st == SBhfState::WaitBlock6 || st == SBhfState::ExitRunning || st == SBhfState::Idle)
        {
            s_weicheRetryPending = false;

            safetyErrorClear();
            s_emergencyActive = false;
            s_lock            = false;
            s_blockReason     = SAFETY_BLOCK_NONE;

            DBG_PRINTLN("[SAFETY] SBHF weiche retry OK -> UNLOCK");
        }
        else if (st == SBhfState::Error)
        {
            // erneuter Fehler -> bleibt emergency
            s_weicheRetryPending = false;
            DBG_PRINTLN("[SAFETY] SBHF weiche retry FAILED -> still locked");
        }
        else if ((now - s_weicheRetryStartMs) > WEICHE_RETRY_TIMEOUT_MS)
        {
            s_weicheRetryPending = false;
            DBG_PRINTLN("[SAFETY] SBHF weiche retry TIMEOUT -> still locked");
        }
    }

    // ------------------------------------------------------------
    // Reverse-Entry / Stopzone / NOTAUS:
    // Trigger wenn:
    // 1) Nothaltgleis ist AUS
    // 2) Trafo unten ist powered (Fahrspannung an)
    // 3) Stopzone-Kontakt (k_nothalt) ist belegt
    // 4) Block 6 Strom ~ 0 (Kontaktgleis ohne Fahrstrom)
    // ------------------------------------------------------------
    if (!s_emergencyActive)
    {
        // Wie vereinbart: "Stopzone aktiv" == Nothaltgleis AUS == isNothaltActive()==true
        const bool stopzoneActive    = g_power.isNothaltActive();
        const bool trafoUntenPowered = isTrafoUntenPowered();
        const bool nothaltKontaktOcc = k_nothalt.raw();

        const uint16_t i6            = g_bc.stromFiltered(6);
        const bool block6NoCurrent   = (i6 <= NO_CURRENT_MA);

        if (stopzoneActive && trafoUntenPowered && nothaltKontaktOcc && block6NoCurrent)
        {
            // Safety lock + Trafo unten aus (Trafo oben aus lassen wir ebenfalls aus für maximale Sicherheit)
            safetySetSSR(SSR_MAIN_ENABLE, false);
            safetySetSSR(SSR_TRAFO_A,     false);
            safetySetSSR(SSR_TRAFO_B,     false);

            s_emergencyActive         = true;
            s_lock                    = true;
            s_blockReason             = SAFETY_BLOCK_EMERGENCY;
            s_emergNothaltSbhfLatched = true;

            safetyErrorSet(SAFETY_ERR_NOTAUS, 6);

            DBG_PRINTLN("[SAFETY] EMERG_REVERSE_ENTRY(Stopzone) -> SSR_A/B OFF, LOCK");
            return;
        }
    }

    // ------------------------------------------------------------
    // Doppelte Blockbelegung (heuristisch via Strom-Anstieg)
    // Trigger wenn:
    // - Block x ist bereits belegt (entryBlocked)
    // - Strom >= DOUBLE_OCC_THRESHOLD_MA
    // - Keine SSR-Schaltaktion in den letzten POWER_STABLE_MS
    // - für DOUBLE_OCC_DETECT_MS stabil
    // ------------------------------------------------------------
    if (!s_emergencyActive)
    {
        const bool powerStable = (now - s_lastPowerSwitchMs) >= POWER_STABLE_MS;
        if (!powerStable)
        {
            for (uint8_t b = 0; b < 16; b++) s_doubleOccStartMs[b] = 0;
        }
        else
        {
            for (uint8_t b = 1; b <= g_bc.count() && b < 16; b++)
            {
                const uint16_t i = g_bc.stromFiltered(b);
                const bool blocked = g_bc.isOccupied(b); // entryBlocked(x)

                if (blocked && i >= DOUBLE_OCC_THRESHOLD_MA)
                {
                    if (s_doubleOccStartMs[b] == 0) s_doubleOccStartMs[b] = now;
                    if ((now - s_doubleOccStartMs[b]) >= DOUBLE_OCC_DETECT_MS)
                    {
                        safetyErrorSet(SAFETY_ERR_DOUBLE_OCCUPANCY, b);
                        safetySetEmergency(true);

                        DBG_PRINTF("[SAFETY] EMERG_DOUBLE_OCCUPANCY(B%d) -> ALL OFF, LOCK
", b);
                        return;
                    }
                }
                else
                {
                    s_doubleOccStartMs[b] = 0;
                }
            }
        }
    }

    // ------------------------------------------------------------
    // Optional: "Short" detect aus Strom (SIM: 2500mA)
    // ------------------------------------------------------------
    if (!s_emergencyActive)
    {
        static uint32_t overSinceMs = 0;

        const uint16_t i6 = g_bc.stromFiltered(6);
        const bool over = (i6 >= SHORT_THRESHOLD_MA);

        if (over)
        {
            if (overSinceMs == 0) overSinceMs = now;
            if ((now - overSinceMs) >= SHORT_DETECT_MS)
            {
                safetyTriggerBlockShort(6);
                overSinceMs = 0;
            }
        }
        else
        {
            overSinceMs = 0;
        }
    }
}
