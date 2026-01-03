#include "safety.h"

#include <Arduino.h>

#include "BlockController.h"
#include "Mega2PowerControl.h"
#include "SensorTrafoAC.h"
#include "SensorKontakt.h"

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

// ============================================================================
// Konfiguration / Thresholds
// ============================================================================
static constexpr uint32_t SHORT_DETECT_MS      = 200;
static constexpr uint16_t SHORT_THRESHOLD_MA   = 1800;
static constexpr uint16_t NO_CURRENT_MA        = 100;
static constexpr uint32_t SSR_STUCK_DELAY_MS   = 250;

// ============================================================================
// Status
// ============================================================================
static bool              s_emergencyActive = false;
static bool              s_lock            = true; // Boot-Lock bis ACK
static SafetyBlockReason s_blockReason     = SAFETY_BLOCK_BOOT;

static uint32_t          s_lastPowerOnMs   = 0;
static uint32_t          s_lastSsrBOffMs   = 0;

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
        s_blockReason = SAFETY_BLOCK_NOTAUS;
        s_lock        = true;

        // Alles aus
        safetySetSSR(SSR_MAIN_ENABLE, false);
        safetySetSSR(SSR_TRAFO_A,     false);
        safetySetSSR(SSR_TRAFO_B,     false);

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
    // ACK darf nur wirken, wenn Lock aktiv ist (Boot-Lock oder Emergency)
    if (!s_lock)
        return true;

    // ------------------------------------------------------------
    // 1) BLOCK_SHORT: ACK nur wenn Strom weg (Block6)
    // ------------------------------------------------------------
    if (s_blockReason == SAFETY_BLOCK_SHORT)
    {
        const uint16_t i6 = g_bc.stromFiltered(6);
        const bool     block6NoCurrent = (i6 <= NO_CURRENT_MA);

        if (!block6NoCurrent)
        {
            DBG_PRINTLN("[SAFETY] ACK blocked (BLOCK_SHORT): Block6 still has current");
            return false;
        }

        // ok -> unlock
        safetyErrorClear();
        s_emergencyActive = false;
        s_lock            = false;
        s_blockReason     = SAFETY_BLOCK_NONE;
        return true;
    }

    // ------------------------------------------------------------
    // 2) SSR_STUCK: ACK nur wenn Trafo unten wirklich aus ist
    // ------------------------------------------------------------
    if (s_blockReason == SAFETY_BLOCK_SSR_STUCK)
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
    if (s_blockReason == SAFETY_BLOCK_NOTAUS && s_emergNothaltSbhfLatched)
    {
        const bool nothaltKontaktFree = !k_nothalt.raw();
        if (!nothaltKontaktFree)
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
    // 4) Boot-Lock: einfach freigeben
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
    // SSR stuck detection (Trafo unten bleibt powered obwohl SSR_B OFF)
    // ------------------------------------------------------------
    if (!s_emergencyActive)
    {
        if (!safetyIsSSR(SSR_TRAFO_B))
        {
            if (s_lastSsrBOffMs == 0)
                s_lastSsrBOffMs = now;

            if ((now - s_lastSsrBOffMs) > SSR_STUCK_DELAY_MS && isTrafoUntenPowered())
            {
                // SSR_B "stuck"
                safetySetSSR(SSR_MAIN_ENABLE, false);
                safetySetSSR(SSR_TRAFO_A,     false);
                safetySetSSR(SSR_TRAFO_B,     false);

                s_emergencyActive = true;
                s_lock            = true;
                s_blockReason     = SAFETY_BLOCK_SSR_STUCK;

                safetyErrorSet(SAFETY_ERR_SSR_STUCK, static_cast<uint8_t>(SSR_TRAFO_B));

                DBG_PRINTLN("[SAFETY] EMERG_SSR_STUCK(B) -> SSR_A/B OFF, LOCK");
            }
        }
        else
        {
            s_lastSsrBOffMs = 0;
        }
    }

    // ------------------------------------------------------------
    // Reverse-Entry / Stopzone / NOTAUS:
    // Trigger wenn:
    // 1) Nothaltgleis ist AUS
    // 2) Trafo unten ist powered (Fahrspannung an)
    // 3) Stopzone-Kontakt (k_nothalt) ist belegt
    // 4) Block 6 Strom = 0
    // ------------------------------------------------------------
    if (!s_emergencyActive)
    {
        // Wie vereinbart: "Stopzone aktiv" == Nothaltgleis AUS == isNothaltActive()==true
        const bool stopzoneActive       = g_power.isNothaltActive();
        const bool trafoUntenPowered    = isTrafoUntenPowered();
        const bool nothaltKontaktOcc    = k_nothalt.raw();

        const uint16_t i6               = g_bc.stromFiltered(6);
        const bool block6NoCurrent      = (i6 <= NO_CURRENT_MA);

        if (stopzoneActive && trafoUntenPowered && nothaltKontaktOcc && block6NoCurrent)
        {
            // Safety lock + Trafo unten aus (Trafo oben aus lassen wir ebenfalls aus für maximale Sicherheit)
            safetySetSSR(SSR_MAIN_ENABLE, false);
            safetySetSSR(SSR_TRAFO_A,     false);
            safetySetSSR(SSR_TRAFO_B,     false);

            s_emergencyActive = true;
            s_lock            = true;
            s_blockReason     = SAFETY_BLOCK_NOTAUS;
            s_emergNothaltSbhfLatched = true;

            safetyErrorSet(SAFETY_ERR_NOTAUS, 6);

            DBG_PRINTLN("[SAFETY] EMERG_REVERSE_ENTRY(Stopzone) -> SSR_A/B OFF, LOCK");
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
