#include "safety.h"
#include "mega2_pins.h"
#include <Arduino.h>

// --------------------------------------------------
// Interner Zustand
// --------------------------------------------------

static bool s_emergencyActive = false;

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
}

// --------------------------------------------------
// Zyklisches Update
// --------------------------------------------------

void safetyUpdate()
{
    // aktuell keine externe Quelle
    // Not-Aus wird ausschließlich über safetySetEmergency(...)
    // oder safetyResetEmergency() beeinflusst
}

// --------------------------------------------------
// Status
// --------------------------------------------------

bool safetyIsEmergencyActive()
{
    return s_emergencyActive;
}

// --------------------------------------------------
// Aktionen
// --------------------------------------------------

void safetySetEmergency(bool active)
{
    s_emergencyActive = active;

    if (s_emergencyActive)
    {
        // Harte Abschaltung beider Trafos
        safetySetSSR(SafetySSR::SSR_TRAFO_A, false);
        safetySetSSR(SafetySSR::SSR_TRAFO_B, false);
    }
}

// --------------------------------------------------
// B3.1 – Quittierung / Reset
// --------------------------------------------------

bool safetyResetEmergency()
{
    if (!s_emergencyActive)
        return true;   // nichts zu tun

    // Hier später Bedingungen möglich:
    // - alle Controller Idle
    // - kein Block besetzt
    // - UI-Bestätigung etc.

    s_emergencyActive = false;

    // WICHTIG:
    // KEIN automatisches Wiedereinschalten der SSR!
    return true;
}

// --------------------------------------------------
// SSR schalten
// --------------------------------------------------

void safetySetSSR(SafetySSR ssr, bool enable)
{
    switch (ssr)
    {
        case SafetySSR::SSR_TRAFO_A:
            digitalWrite(PIN_RELAY_TRAFO_OBEN_CUT, enable ? LOW : HIGH);
            break;

        case SafetySSR::SSR_TRAFO_B:
            digitalWrite(PIN_RELAY_TRAFO_UNTEN_CUT, enable ? LOW : HIGH);
            break;

        default:
            break;
    }
}
