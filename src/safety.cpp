#include "safety.h"
#include "mega2_pins.h"
#include <Arduino.h>
#include "safety_error.h"
#include "BlockController.h"
#include "Mega2PowerControl.h"
#include "SensorTrafoAC.h"

// Externe Objekte (definiert in main.cpp)
extern BlockController   g_bc;
extern Mega2PowerControl g_power;
extern SensorTrafoAC     g_trafoUnten;
extern SensorTrafoAC     g_trafoOben;

// --------------------------------------------------
// Interner Zustand
// --------------------------------------------------

// Akuter Not-Aus (Taste gedrückt / sofortige Abschaltung)

static bool s_emergencyActive = false;

// Latenter Safety-Lock: nach einem Safety-Ereignis muss quittiert werden,
// bevor wieder eingeschaltet werden darf.


// interner Merker für SSR-Zustand
static bool s_ssrState[2] = { false, false };


static SafetyBlockReason s_blockReason = SAFETY_BLOCK_BOOT;

// --------------------------------------------------
// SBHF-Servicefall: Reverse-Entry im Nothaltgleis (Block 6)
// --------------------------------------------------
static bool     s_emergNothaltSbhfLatched = false;
static uint32_t s_nothaltCondSinceMs      = 0;

// Debug (SIM): Trafo unten "powered" erzwingen
static bool s_dbgTrafoUntenForced = false;

static bool isTrafoUntenPowered()
{
#if MEGA2_SIM_MODE
    if (s_dbgTrafoUntenForced)
        return true;
#endif
    return g_trafoUnten.isPowered();
}


// --------------------------------------------------
// Initialisierung
// --------------------------------------------------

void safetyBegin()
{
    pinMode(PIN_RELAY_TRAFO_OBEN_CUT, OUTPUT);
    pinMode(PIN_RELAY_TRAFO_UNTEN_CUT, OUTPUT);

    // Sicherer Start: beide Trafos AUS
    safetySetSSR(SafetySSR::SSR_TRAFO_A, false);
    safetySetSSR(SafetySSR::SSR_TRAFO_B, false);

    s_emergencyActive = false;

    s_emergNothaltSbhfLatched = false;
    s_nothaltCondSinceMs      = 0;
    s_dbgTrafoUntenForced     = false;

    // Boot-Zustand: Start gesperrt, aber kein Not-Aus
    s_blockReason = SAFETY_BLOCK_BOOT;
}

// --------------------------------------------------
// Zyklisches Update
// --------------------------------------------------

void safetyUpdate()
{
    // Wenn bereits gelockt oder Not-Aus aktiv: nichts Neues auswerten
    if (s_blockReason != SAFETY_BLOCK_NONE)
        return;

    const uint32_t now = millis();

    // --------------------------------------------------
    // EMERG_NOTHALT_SBHF (Servicefall Reverse-Entry Block 6)
    //
    // Trigger (Contract):
    //  - Trafo unten powered
    //  - Nothaltgleis scharf (S16)
    //  - Block 6 belegt
    //  - kein Stromfluss in Block 6
    // --------------------------------------------------
    const bool trafoPowered = isTrafoUntenPowered();
    const bool nothaltOn    = g_power.isNothaltActive();
    const bool b6occ        = g_bc.isOccupied(6);
    const bool b6noI        = (g_bc.stromFiltered(6) == 0);

    const bool cond = trafoPowered && nothaltOn && b6occ && b6noI;

    if (cond)
    {
        if (s_nothaltCondSinceMs == 0)
            s_nothaltCondSinceMs = now;

        // kleine Bestätigungszeit gegen Flattern
        if ((now - s_nothaltCondSinceMs) >= 200)
        {
            // Lock setzen
            s_blockReason = SAFETY_BLOCK_EMERGENCY;
            s_emergNothaltSbhfLatched = true;

            // Unteren Trafo abschalten (SSR_B)
            safetySetSSR(SafetySSR::SSR_TRAFO_B, false);

            // Error-Info setzen (nutzen vorhandenen Code NOTAUS, Context=6)
            safetyErrorSet(SAFETY_ERR_NOTAUS, 6);

#if MEGA2_DEBUG
            Serial.println(F("[SAFETY] EMERG_NOTHALT_SBHF -> SSR_B OFF, LOCK"));
#endif
        }
    }
    else
    {
        s_nothaltCondSinceMs = 0;
    }
}

// --------------------------------------------------
// Status
// --------------------------------------------------

bool safetyIsEmergencyActive()
{
    return s_blockReason == SAFETY_BLOCK_EMERGENCY;
}

bool safetyIsSSR(SafetySSR ssr)
{
    return s_ssrState[static_cast<uint8_t>(ssr)];
}

bool safetyIsPowerOn()
{
    return safetyIsSSR(SafetySSR::SSR_TRAFO_A)
        && safetyIsSSR(SafetySSR::SSR_TRAFO_B);
}

bool safetyIsLocked()
{
    return s_blockReason != SAFETY_BLOCK_NONE;
}

uint8_t safetyGetBlockReason()
{
    return static_cast<uint8_t>(s_blockReason);
}




void safetyDebugForceTrafoUntenPowered(bool on)
{
    s_dbgTrafoUntenForced = on;
}

bool safetyDebugIsTrafoUntenForced()
{
    return s_dbgTrafoUntenForced;
}

// Optional (nur falls du es im Status/Debug später anzeigen willst)
// bool safetyIsLocked() { return s_safetyLocked; }

// --------------------------------------------------
// Aktionen
// --------------------------------------------------

void safetySetEmergency(bool active)
{
    if (active)
    {
        // Echter Not-Aus hat Vorrang, SBHF-Service-Merker zurücksetzen
        s_emergNothaltSbhfLatched = false;
        s_nothaltCondSinceMs      = 0;
        s_emergencyActive = true;
        s_blockReason     = SAFETY_BLOCK_EMERGENCY;

        safetySetSSR(SafetySSR::SSR_TRAFO_A, false);
        safetySetSSR(SafetySSR::SSR_TRAFO_B, false);

        safetyErrorSet(SAFETY_ERR_NOTAUS, 0);
        return;
    }

    s_emergencyActive = false;
}

// --------------------------------------------------
// Quittierung / Reset
// --------------------------------------------------

bool safetyResetEmergency()
{
    // Spezieller Servicefall: Reverse-Entry Block 6
    if (s_emergNothaltSbhfLatched)
    {
        const bool block6Free  = !g_bc.isOccupied(6);
        const bool nothaltFree = !g_power.isNothaltActive();

        if (!block6Free || !nothaltFree)
        {
#if MEGA2_DEBUG
            Serial.print(F("[SAFETY] ACK blocked (EMERG_NOTHALT_SBHF): "));
            if (!block6Free)  Serial.print(F("Block6 still occupied "));
            if (!nothaltFree) Serial.print(F("Nothalt still active "));
            Serial.println();
#endif
            return false;
        }

        // Bedingungen erfüllt -> Merker entfernen
        s_emergNothaltSbhfLatched = false;
        s_nothaltCondSinceMs      = 0;
    }

    s_emergencyActive = false;
    s_blockReason     = SAFETY_BLOCK_NONE;

    safetyErrorClear();
    return true;
}

// --------------------------------------------------
// SSR schalten
// --------------------------------------------------

void safetySetSSR(SafetySSR ssr, bool enable)
{
    s_ssrState[static_cast<uint8_t>(ssr)] = enable;

    switch (ssr)
    {
        case SafetySSR::SSR_TRAFO_A:
            // aktiv-low Cut-Relais: LOW = durchschalten, HIGH = cut
            digitalWrite(PIN_RELAY_TRAFO_OBEN_CUT, enable ? LOW : HIGH);
            break;

        case SafetySSR::SSR_TRAFO_B:
            digitalWrite(PIN_RELAY_TRAFO_UNTEN_CUT, enable ? LOW : HIGH);
            break;

        default:
            break;
    }
}

// --------------------------------------------------
// Explizites Wiedereinschalten der Leistung
// --------------------------------------------------

bool safetyPowerOn()
{
    // echter Not-Aus blockiert immer
    if (s_emergencyActive)
        return false;

    // jegliche Blockade (Boot oder Emergency) blockiert PowerOn
    if (s_blockReason != SAFETY_BLOCK_NONE)
        return false;

    safetySetSSR(SafetySSR::SSR_TRAFO_A, true);
    safetySetSSR(SafetySSR::SSR_TRAFO_B, true);

    return true;
}
