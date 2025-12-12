#include "safety.h"
#include "mega2_pins.h"
#include <Arduino.h>

// --------------------------------------------------
// Interner Zustand
// --------------------------------------------------

// Not-Aus-Latch
static bool s_emergencyActive = false;

// --------------------------------------------------
// Initialisierung
// --------------------------------------------------

void safetyBegin()
{
    pinMode(PIN_RELAY_TRAFO_OBEN_CUT, OUTPUT);
    pinMode(PIN_RELAY_TRAFO_UNTEN_CUT, OUTPUT);

    // Sicherer Startzustand: beide Trafos AUS
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
    // ausgelöst (z. B. durch Proto / UI)
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

void safetySetSSR(SafetySSR ssr, bool enable)
{
    switch (ssr)
    {
        case SafetySSR::SSR_TRAFO_A:
            // Annahme: LOW = EIN, HIGH = AUS
            digitalWrite(PIN_RELAY_TRAFO_OBEN_CUT, enable ? LOW : HIGH);
            break;

        case SafetySSR::SSR_TRAFO_B:
            digitalWrite(PIN_RELAY_TRAFO_UNTEN_CUT, enable ? LOW : HIGH);
            break;

        default:
            // andere SSRs (z. B. MAIN_ENABLE) aktuell nicht verwendet
            break;
    }
}
