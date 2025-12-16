#include "safety.h"
#include "mega2_pins.h"
#include <Arduino.h>

// --------------------------------------------------
// Interner Zustand
// --------------------------------------------------

// Akuter Not-Aus (Taste gedrückt / sofortige Abschaltung)
static bool s_emergencyActive = false;

// Latenter Safety-Lock: nach einem Safety-Ereignis muss quittiert werden,
// bevor wieder eingeschaltet werden darf.
static bool s_safetyLocked = true; // sicherer Default: nach Boot erst quittieren

// interner Merker für SSR-Zustand
static bool s_ssrState[2] = { false, false };

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
    s_safetyLocked    = true;   // Boot -> erst ACK, dann PowerOn (sicher)
}

// --------------------------------------------------
// Zyklisches Update
// --------------------------------------------------

void safetyUpdate()
{
    // aktuell keine externe Quelle
}

// --------------------------------------------------
// Status
// --------------------------------------------------

bool safetyIsEmergencyActive()
{
    return s_emergencyActive;
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

// Optional (nur falls du es im Status/Debug später anzeigen willst)
// bool safetyIsLocked() { return s_safetyLocked; }

// --------------------------------------------------
// Aktionen
// --------------------------------------------------

void safetySetEmergency(bool active)
{
    // Nur beim Aktivieren: Lock setzen und hart abschalten
    if (active)
    {
        s_emergencyActive = true;
        s_safetyLocked    = true; // 🔒 Lock bleibt bis ACK

        // Harte Abschaltung beider Trafos
        safetySetSSR(SafetySSR::SSR_TRAFO_A, false);
        safetySetSSR(SafetySSR::SSR_TRAFO_B, false);
        return;
    }

    // Deaktivieren (z.B. Not-Aus losgelassen):
    // Emergency kann weg sein, aber Lock bleibt!
    s_emergencyActive = false;
}

// --------------------------------------------------
// Quittierung / Reset
// --------------------------------------------------

bool safetyResetEmergency()
{
    // ACK quittiert den latenten Lock, unabhängig davon,
    // ob emergencyActive gerade noch true ist.
    s_emergencyActive = false;
    s_safetyLocked    = false;  // 🔑 Lock lösen

    // WICHTIG: KEIN automatisches Wiedereinschalten der SSR!
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
    // Darf nur erfolgen, wenn KEIN Not-Aus aktiv ist
    if (s_emergencyActive)
        return false;

    // Darf nur erfolgen, wenn Safety quittiert wurde
    if (s_safetyLocked)
        return false;

    safetySetSSR(SafetySSR::SSR_TRAFO_A, true);
    safetySetSSR(SafetySSR::SSR_TRAFO_B, true);

    return true;
}
