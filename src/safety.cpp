#include "safety.h"

#include <Arduino.h>

#include "BlockController.h"
#include "Mega2PowerControl.h"
#include "SensorTrafoAC.h"
#include "SensorKontakt.h"
#include "ShadowYardController.h"

#include "safety_error.h"
#include "mega2_debug.h"

#include "proto_mega2.h"
#include "threshold_values.h"

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
static constexpr uint32_t SHORT_DETECT_MS      = THR_SHORT_DETECT_MS;
static constexpr uint16_t SHORT_THRESHOLD_MA   = THR_SHORT_MA_EFF;
static constexpr uint16_t NO_CURRENT_MA        = THR_NO_CURRENT_MA;

// Doppelte Blockbelegung (heuristisch via Strom)
static constexpr uint16_t DOUBLE_OCC_THRESHOLD_MA = THR_DOUBLE_OCC_MA_EFF;
static constexpr uint32_t DOUBLE_OCC_DETECT_MS    = THR_DOUBLE_OCC_DETECT_MS;
static constexpr uint32_t POWER_STABLE_MS         = THR_POWER_STABLE_MS;  // nach SSR-Schaltvorgängen keine Fehltrigger
// Adaptive Double-Occ (zusätzlich zum Hard-Trigger)
static constexpr uint32_t DOUBLE_OCC_TAU_MS        = THR_DOUBLE_OCC_TAU_MS;        // ~2s EMA
static constexpr uint16_t DOUBLE_OCC_FACTOR_NUM    = THR_DOUBLE_OCC_FACTOR_NUM;    // 1.5x
static constexpr uint16_t DOUBLE_OCC_FACTOR_DEN    = THR_DOUBLE_OCC_FACTOR_DEN;
static constexpr uint16_t DOUBLE_OCC_DELTA_MIN_MA  = THR_DOUBLE_OCC_DELTA_MIN_MA;  // absoluter Sprung
static constexpr uint16_t DOUBLE_OCC_BASE_MIN_MA   = THR_DOUBLE_OCC_BASE_MIN_MA;   // optional: Basis muss "echt" sein


// Safety-Controller Fault (Watchdog / Invariants)
static constexpr uint32_t SAFETY_TICK_MAX_GAP_MS  = 1000;

// SBHF Weichenfehler: ACK->Retry Timeout
static constexpr uint32_t SSR_STUCK_DELAY_MS   = 250;
static constexpr uint32_t SBHF_SELFTEST_TIMEOUT_MS = 15000;

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
static uint32_t          s_doubleOccStartMs[MEGA2_MAX_BLOCKS] = {0};

// Adaptive double occupancy: EMA baseline (mA) per block
static uint16_t s_doubleOccBaseMa[MEGA2_MAX_BLOCKS]    = {0};
static uint32_t s_doubleOccBaseLastMs[MEGA2_MAX_BLOCKS]= {0};

// SIM/Debug: forced block current (mA). 0 => disabled.
static uint16_t          s_forceBlockCurrentMa[MEGA2_MAX_BLOCKS] = {0};

// SBHF Weichenfehler: ACK -> Selftest (ShadowYardController)
static bool              s_sbhfSelftestPending = false;
static uint32_t          s_sbhfSelftestStartMs = 0;

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
    s_sbhfSelftestPending = false;
    s_sbhfSelftestStartMs = 0;
}

bool safetyIsEmergencyActive() { return s_emergencyActive; }
bool safetyIsLocked()          { return s_lock; }
uint8_t safetyGetBlockReason() { return static_cast<uint8_t>(s_blockReason); }

// Debug (SIM)
#if MEGA2_SIM_MODE
void safetyDebugForceTrafoUntenPowered(bool on) { s_forceTrafoUntenPowered = on; }
bool safetyDebugIsTrafoUntenForced() { return s_forceTrafoUntenPowered; }
#else
void safetyDebugForceTrafoUntenPowered(bool) {}
bool safetyDebugIsTrafoUntenForced() { return false; }
#endif

// ------------------------------------------------------------
// SIM/Debug: force synthetic block current (mA)
// ------------------------------------------------------------
void safetyDebugForceBlockCurrentMa(uint8_t block, uint16_t ma)
{
    if (block >= MEGA2_MAX_BLOCKS) return;
    s_forceBlockCurrentMa[block] = ma;
}

uint16_t safetyDebugGetForcedBlockCurrentMa(uint8_t block)
{
    if (block >= MEGA2_MAX_BLOCKS) return 0;
    return s_forceBlockCurrentMa[block];
}

static inline uint16_t safetyBlockCurrentMa(uint8_t block)
{
    const uint16_t forced = (block < MEGA2_MAX_BLOCKS) ? s_forceBlockCurrentMa[block] : 0;
    if (forced) return forced;
    return g_bc.stromFiltered(block);
}


static inline uint16_t emaUpdateMa(uint16_t base, uint16_t sample, uint32_t dtMs, uint32_t tauMs)
{
    if (dtMs == 0 || tauMs == 0) return base;
    const uint32_t alphaNum = dtMs;
    const uint32_t alphaDen = tauMs + dtMs;
    const int32_t diff = (int32_t)sample - (int32_t)base;
    const int32_t delta = (int32_t)(((int64_t)diff * alphaNum) / alphaDen);
    int32_t out = (int32_t)base + delta;
    if (out < 0) out = 0;
    if (out > 65535) out = 65535;
    return (uint16_t)out;
}


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
    // Wenn nicht gelockt: optionaler manueller Selftest-Retry (z.B. nach Fix im eingeschränkten Betrieb)
    if (!s_lock)
    {
        if (!shadowController.isSelftestActive())
        {
            if (shadowController.startSelftest(true /*include W14/W15*/))
            {
                DBG_PRINTLN(F("[SAFETY] Selftest retry started (unlocked)"));
            }
        }
        return true;
    }

    const SafetyErrorInfo& err = safetyErrorGet();

    // ------------------------------------------------------------
    // 1) Block-Short: ACK nur wenn Strom wieder 0 ist
    // ------------------------------------------------------------
    if (err.type == SAFETY_ERR_BLOCK_SHORT)
    {
        const uint8_t b = (err.index >= 1 && err.index <= g_bc.count()) ? err.index : 6;
        const uint16_t i = safetyBlockCurrentMa(b);
        if (i > NO_CURRENT_MA)
        {
            DBG_PRINTF("[SAFETY] ACK blocked (SHORT): B%d current=%umA\n", b, i);
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
            DBG_PRINTLN(F("[SAFETY] ACK blocked (SSR_STUCK): Trafo unten still powered"));
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
            DBG_PRINTLN(F("[SAFETY] ACK blocked (NOTAUS): Stopzone contact still occupied"));
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
    // 4) SBHF Weichenfehler (W12/W13 kritisch):
    //    ACK startet IMMER einen Selbsttest auf Mega2.
    //    SafetyLock bleibt während des Selbsttests aktiv.
    //    Nach Selftest: entweder Normalbetrieb (kein Warn),
    //    oder eingeschränkter Betrieb (Warn + allowedMask),
    //    oder kein sicherer Pfad (Lock bleibt).
    // ------------------------------------------------------------
    if (err.type == SAFETY_ERR_SBH_WEICHE)
    {
        // Wenn Selftest bereits läuft (z.B. via UI-Button), nicht blocken,
        // sondern nur pending setzen und warten bis selftestDone.
        if (shadowController.isSelftestActive())
        {
            s_sbhfSelftestPending = true;
            s_sbhfSelftestStartMs = millis();
            DBG_PRINTLN(F("[SAFETY] ACK accepted (SBH_WEICHE): selftest already running"));
            return true;
        }

        if (!shadowController.startSelftest(true /*include W14/W15*/))
        {
            DBG_PRINTLN(F("[SAFETY] ACK blocked (SBH_WEICHE): selftest could not start"));
            return false;
        }

        s_sbhfSelftestPending = true;
        s_sbhfSelftestStartMs = millis();

        DBG_PRINTLN(F("[SAFETY] ACK accepted (SBH_WEICHE): selftest started"));
        return true;
    }

    // ------------------------------------------------------------
    // 5) Doppelte Blockbelegung: ACK nur wenn Strom wieder 0 ist
    // ------------------------------------------------------------
    if (err.type == SAFETY_ERR_DOUBLE_OCCUPANCY)
    {
        const uint8_t b = (err.index >= 1 && err.index <= g_bc.count()) ? err.index : 0;
        const uint16_t i = (b ? safetyBlockCurrentMa(b) : 0);

        if (b && i > NO_CURRENT_MA)
        {
            DBG_PRINTF("[SAFETY] ACK blocked (DOUBLE_OCC): B%d current=%umA\n", b, i);
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
        DBG_PRINTLN(F("[SAFETY] ACK accepted (CTRL_FAULT): reinit safety subsystem (boot-lock)"));

        safetyErrorClear();
        s_emergencyActive = false;
        s_lock            = true;
        s_blockReason     = SAFETY_BLOCK_BOOT;

        s_emergNothaltSbhfLatched = false;
        s_sbhfSelftestPending = false;
        s_sbhfSelftestStartMs = 0;
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


void safetyNotifySbhfSelftestStarted()
{
    // Nur sinnvoll im Kontext SBHF-Weichenfehler
    if (safetyErrorGet().type != SAFETY_ERR_SBH_WEICHE)
        return;

    s_sbhfSelftestPending = true;
    s_sbhfSelftestStartMs = millis();

    DBG_PRINTLN(F("[SAFETY] SBHF selftest pending set (external start)"));
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
            DBG_PRINTLN(F("[SAFETY] EMERG_CONTROLLER_FAULT(tick-gap) -> ALL OFF, LOCK"));
            s_lastSafetyUpdateMs = now;
            return;
        }

        // einfache Invariants (nur Beispiele; erweitert man bei Bedarf)
        if (s_lock && s_blockReason == SAFETY_BLOCK_NONE)
        {
            safetyErrorSet(SAFETY_ERR_CONTROLLER_FAULT, 2); // inkonsistent
            safetySetEmergency(true);
            DBG_PRINTLN(F("[SAFETY] EMERG_CONTROLLER_FAULT(invariant) -> ALL OFF, LOCK"));
            s_lastSafetyUpdateMs = now;
            return;
        }
    }
    s_lastSafetyUpdateMs = now;

    // ------------------------------------------------------------
    // SBHF Weichenfehler: Selftest-Auswertung nach ACK
    // ------------------------------------------------------------
    // SBHF Weichenfehler: Selftest-Auswertung nach (extern/ACK) gestartetem Selftest.
    // Nicht zusätzlich an s_emergencyActive koppeln, sonst kann ein Deadlock entstehen,
    // wenn Emergency/Lock-Zustände zwischendrin anders gesetzt/geresettet wurden.
    if (s_sbhfSelftestPending && safetyErrorGet().type == SAFETY_ERR_SBH_WEICHE)
    {
        if (shadowController.isSelftestDone())
        {
            s_sbhfSelftestPending = false;
            shadowController.clearSelftestDone();

            const uint8_t allowed = shadowController.allowedGleisMask();
            const uint8_t warn    = shadowController.warningMask();

            if (allowed == 0)
            {
                // Kein sicherer Pfad ableitbar -> bleibt LOCKED
                DBG_PRINTLN(F("[SAFETY] SBHF selftest FAIL(no-safe) -> still locked"));
            }
            else
            {
                // SBHF Error-Reset + Fortsetzen (wenn Resume-Gleis erlaubt)
                if (shadowController.canReset())
                    shadowController.onResetAck();

                safetyErrorClear();
                s_emergencyActive = false;
                s_lock            = false;
                s_blockReason     = SAFETY_BLOCK_NONE;

                DBG_PRINTF("[SAFETY] SBHF selftest OK -> UNLOCK (allowed=0x%02X warn=0x%02X)\n", allowed, warn);
            }
        }
        else if ((now - s_sbhfSelftestStartMs) > SBHF_SELFTEST_TIMEOUT_MS)
        {
            s_sbhfSelftestPending = false;
            DBG_PRINTLN(F("[SAFETY] SBHF selftest TIMEOUT -> still locked"));
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
        // Stopzone aktiv / scharf nur dann, wenn das Nothaltgleis AUS ist.
        // Nach der korrigierten S15/S16-Semantik bedeutet das:
        // isNothaltActive()==false  -> Stopzone scharf
        const bool stopzoneActive    = !g_power.isNothaltActive();
        const bool trafoUntenPowered = isTrafoUntenPowered();
        const bool nothaltKontaktOcc = k_nothalt.raw();

        const uint16_t i6            = safetyBlockCurrentMa(6);
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

            DBG_PRINTLN(F("[SAFETY] EMERG_REVERSE_ENTRY(Stopzone) -> SSR_A/B OFF, LOCK"));
            return;
        }
    }

    // ------------------------------------------------------------
    // Doppelte Blockbelegung (heuristisch via Strom-Anstieg)
    // Trigger wenn:
    // - Block x ist bereits belegt (isOccupied)
    // - Strom >= DOUBLE_OCC_THRESHOLD_MA
    // - Keine SSR-Schaltaktion in den letzten POWER_STABLE_MS
    // - für DOUBLE_OCC_DETECT_MS stabil
    // ------------------------------------------------------------
    if (!s_emergencyActive)
    {
        const bool powerStable = (now - s_lastPowerSwitchMs) >= POWER_STABLE_MS;
        if (!powerStable)
        {
            for (uint8_t b = 0; b < MEGA2_MAX_BLOCKS; b++)
            {
                s_doubleOccStartMs[b] = 0;
                s_doubleOccBaseMa[b] = 0;
                s_doubleOccBaseLastMs[b] = 0;
            }
        }
        else
        {
            for (uint8_t b = 1; b <= g_bc.count() && b < MEGA2_MAX_BLOCKS; b++)
            {
                const uint16_t iNow = safetyBlockCurrentMa(b);
                const bool occupied = g_bc.isOccupied(b);

                // --- Update EMA baseline only when occupied ---
                if (occupied)
                {
                    const uint32_t last = s_doubleOccBaseLastMs[b];
                    const uint32_t dt   = (last == 0) ? 0 : (now - last);
                    s_doubleOccBaseLastMs[b] = now;

                    if (s_doubleOccBaseMa[b] == 0)
                    {
                        s_doubleOccBaseMa[b] = iNow; // init
                    }
                    else if (dt > 0 && dt < 5000)
                    {
                        s_doubleOccBaseMa[b] = emaUpdateMa(s_doubleOccBaseMa[b], iNow, dt, DOUBLE_OCC_TAU_MS);
                    }
                }
                else
                {
                    s_doubleOccBaseMa[b] = 0;
                    s_doubleOccBaseLastMs[b] = 0;
                }

                // --- compute adaptive threshold ---
                const uint16_t iBase = s_doubleOccBaseMa[b];
                const uint16_t thrRel = (uint16_t)((uint32_t)iBase * DOUBLE_OCC_FACTOR_NUM / DOUBLE_OCC_FACTOR_DEN);
                const uint16_t thrAbs = (uint16_t)(iBase + DOUBLE_OCC_DELTA_MIN_MA);
                const uint16_t thrAdaptive = (thrRel > thrAbs) ? thrRel : thrAbs;

                // --- candidate condition: Hard OR Adaptive ---
                const bool inRangeForDoubleOcc = (iNow < SHORT_THRESHOLD_MA);

                const bool hardTrip = (iNow >= DOUBLE_OCC_THRESHOLD_MA);
                const bool adaptiveTrip = (iBase >= DOUBLE_OCC_BASE_MIN_MA) && (iNow >= thrAdaptive);

                const bool doubleOccCandidate =
                    occupied &&
                    inRangeForDoubleOcc &&
                    (hardTrip || adaptiveTrip);

                if (doubleOccCandidate)
                {
                    if (s_doubleOccStartMs[b] == 0) s_doubleOccStartMs[b] = now;
                    if ((now - s_doubleOccStartMs[b]) >= DOUBLE_OCC_DETECT_MS)
                    {
                        safetyErrorSet(SAFETY_ERR_DOUBLE_OCCUPANCY, b);
                        safetySetEmergency(true);

                        DBG_PRINTF("[SAFETY] EMERG_DOUBLE_OCCUPANCY(B%d) iNow=%umA iBase=%umA thr=%umA -> ALL OFF, LOCK\n",
                                b, iNow, iBase, (hardTrip ? DOUBLE_OCC_THRESHOLD_MA : thrAdaptive));
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

        const uint16_t i6 = safetyBlockCurrentMa(6);
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