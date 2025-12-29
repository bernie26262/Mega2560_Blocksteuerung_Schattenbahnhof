#include "safety.h"
#include "mega2_pins.h"
#include <Arduino.h>
#include "safety_error.h"

#include "BlockController.h"
#include "Mega2PowerControl.h"
#include "SensorTrafoAC.h"

extern BlockController   g_bc;
extern Mega2PowerControl g_power;
extern SensorTrafoAC     g_trafoUnten; // Trafo B (unten) Spannungssensor

// --------------------------------------------------
// Interner Zustand
// --------------------------------------------------

// Akuter Not-Aus (Taste gedrückt / sofortige Abschaltung)

static bool s_emergencyActive = false;

// Latenter Safety-Lock: nach einem Safety-Ereignis muss quittiert werden,
// bevor wieder eingeschaltet werden darf.


// interner Merker für SSR-Zustand
static bool s_ssrState[2] = { false, false };
// Merker: EMERG_NOTHALT_SBHF wurde ausgelöst (Servicefall)
static bool s_emergNothaltSbhfLatched = false;
// Debug: Trafo unten "powered" erzwingen (ohne Hardware)
static bool s_dbgForceTrafoUntenPowered = false;




static SafetyBlockReason s_blockReason = SAFETY_BLOCK_BOOT;

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

    // Boot-Zustand: Start gesperrt, aber kein Not-Aus
    s_blockReason = SAFETY_BLOCK_BOOT;
    s_emergNothaltSbhfLatched = false;
    s_dbgForceTrafoUntenPowered = false;
}

// --------------------------------------------------
// Zyklisches Update
// --------------------------------------------------

void safetyUpdate()
{
    const uint32_t now = millis();

    // --------------------------------------------------
    // EMERG_NOTHALT_SBHF (Servicefall: Zug falschherum im Nothaltgleis)
    //
    // Trigger (Contract):
    //  I_block_6 == 0
    //  AND Kontaktgleis_NOTHALT_SBHF == belegt
    //  AND V_traf_B > V_THRESHOLD
    //
    // Umsetzung:
    //  - Block6: g_bc.isOccupied(6) + g_bc.stromFiltered(6)
    //  - Trafo B Spannung: g_trafoUnten.isPowered()
    //  - Nothaltgleis muss AUS sein (Stopzone scharf): g_power.isNothaltActive()
    // Aktion:
    //  - SSR Trafo B OFF
    //  - lock = true (SAFETY_BLOCK_EMERGENCY)
    //  - ACK erst erlaubt, wenn Block6 frei UND Nothalt frei
    // --------------------------------------------------

    if (s_blockReason == SAFETY_BLOCK_NONE)
    {
        const bool trafoBPowered  = g_trafoUnten.isPowered() || s_dbgForceTrafoUntenPowered;
        const bool nothaltActive  = g_power.isNothaltActive();     // Stopzone: Gleis AUS
        const bool block6Occupied = g_bc.isOccupied(6);
        const bool block6NoCurrent = (g_bc.stromFiltered(6) == 0);

        const bool cond = trafoBPowered && nothaltActive && block6Occupied && block6NoCurrent;

        static uint32_t sinceMs = 0;

        if (cond)
        {
            if (sinceMs == 0) sinceMs = now;

            // 200ms stabil -> Emergency latch
            if (now - sinceMs >= 200)
            {
                s_blockReason = SAFETY_BLOCK_EMERGENCY;
                s_emergNothaltSbhfLatched = true;

                // Nur Trafo B abschalten (Contract)
                safetySetSSR(SafetySSR::SSR_TRAFO_B, false);

                // Fehlercode (vorerst NOTAUS mit Kontext=6, bis es ein eigenes Enum gibt)
                safetyErrorSet(SAFETY_ERR_NOTAUS, 6);

            #if MEGA2_DEBUG
                Serial.println(F("[SAFETY] EMERG_NOTHALT_SBHF -> SSR_B OFF, LOCK"));
            #endif
            }
        }
        else
        {
            sinceMs = 0;
        }
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


// --------------------------------------------------
// Debug-API: Trafo unten "powered" erzwingen
// --------------------------------------------------

void safetyDebugForceTrafoUntenPowered(bool on)
{
    s_dbgForceTrafoUntenPowered = on;
}

bool safetyDebugIsTrafoUntenForced()
{
    return s_dbgForceTrafoUntenPowered;
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
    // Spezieller Servicefall: EMERG_NOTHALT_SBHF darf erst quittiert werden,
    // wenn Block 6 wieder frei ist UND der Bediener das Nothaltgleis wieder freigegeben hat.
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

        s_emergNothaltSbhfLatched = false;
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

