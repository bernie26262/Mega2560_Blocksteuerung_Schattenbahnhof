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
// SSR-Stuck-Detektion: SSR ist AUS, aber Trafo-Spannung bleibt anliegen
static uint32_t s_ssrStuckCondSinceMs[2] = { 0, 0 };

static constexpr uint32_t SSR_STUCK_DETECT_MS = 500;



static SafetyBlockReason s_blockReason = SAFETY_BLOCK_BOOT;


// -----------------------------------------------------------------------------
// Kurzschluss-Erkennung (automatisch)
// -----------------------------------------------------------------------------
// Blöcke 1..9 sind die "realen" Strommess-Blöcke (SBhf-Gleise 1..3 sind 7..9).
static constexpr uint8_t  SHORT_BLOCK_MIN   = 1;
static constexpr uint8_t  SHORT_BLOCK_MAX   = 9;
static constexpr uint16_t SHORT_I_MA        = 1800;  // ab hier "Kurzschluss vermuten"
static constexpr uint16_t SHORT_CLEAR_MA    = 100;   // darunter gilt Strom als "weg"
static constexpr uint16_t SHORT_CONFIRM_MS  = 200;   // so lange muss es anstehen

// Für die "falsche Fahrtrichtung in Block 6"-Erkennung: Strom muss praktisch 0 sein.
static constexpr uint16_t NO_CURRENT_MA     = 100;

static uint32_t s_shortCondSinceMs[SHORT_BLOCK_MAX + 1] = {0};


// --------------------------------------------------
// SBHF-Servicefall: Reverse-Entry im Nothaltgleis (Block 6)
// --------------------------------------------------
static bool     s_emergNothaltSbhfLatched = false;
static uint32_t s_nothaltCondSinceMs      = 0;

// Debug (SIM): Trafo unten "powered" erzwingen
static bool s_dbgTrafoUntenForced = false;

static bool isTrafoObenPowered()
{
    return g_trafoOben.isPowered();
}


static bool isTrafoUntenPowered()
{
#if MEGA2_SIM_MODE
    if (s_dbgTrafoUntenForced)
        return true;
#endif
    return g_trafoUnten.isPowered();
}


static void checkSsrStuck(uint32_t now)
{
    // Wenn ein anderer Fehler aktiv ist (z.B. Block-Short), lassen wir ihn latches.
    if (safetyErrorActive() && safetyErrorGet().type != SAFETY_ERR_SSR_STUCK)
        return;

    // Für beide SSRs prüfen: SSR AUS, aber Spannung bleibt anliegen
    for (uint8_t idx = 0; idx < 2; idx++)
    {        // SSR EIN => keine Stuck-Prüfung
        if (s_ssrState[idx])
        {
            s_ssrStuckCondSinceMs[idx] = 0;
            continue;
        }

        const bool powered = (idx == 0) ? isTrafoObenPowered() : isTrafoUntenPowered();
        if (!powered)
        {
            s_ssrStuckCondSinceMs[idx] = 0;
            continue;
        }

        if (s_ssrStuckCondSinceMs[idx] == 0)
            s_ssrStuckCondSinceMs[idx] = now;

        if ((now - s_ssrStuckCondSinceMs[idx]) >= SSR_STUCK_DETECT_MS)
        {
            // Safety-Lock setzen
            s_blockReason = SAFETY_BLOCK_EMERGENCY;

            // Beide SSRs hart AUS (failsafe)
            safetySetSSR(SafetySSR::SSR_TRAFO_A, false);
            safetySetSSR(SafetySSR::SSR_TRAFO_B, false);

    s_ssrStuckCondSinceMs[0] = 0;
    s_ssrStuckCondSinceMs[1] = 0;

            safetyErrorSet(SAFETY_ERR_SSR_STUCK, idx);

#if MEGA2_DEBUG
            Serial.print(F("[SAFETY] EMERG_SSR_STUCK("));
            Serial.print(idx == 0 ? F("A") : F("B"));
            Serial.println(F(") -> SSR_A/B OFF, LOCK"));
#endif
            return;
        }
    }
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

    s_ssrStuckCondSinceMs[0] = 0;
    s_ssrStuckCondSinceMs[1] = 0;

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


    // ---------------------------------------------------------
    // Kurzschluss automatisch erkennen (nur wenn "Power ON")
    // ---------------------------------------------------------
    if (!safetyIsPowerOn())
    {
        for (uint8_t b = SHORT_BLOCK_MIN; b <= SHORT_BLOCK_MAX; b++)
            s_shortCondSinceMs[b] = 0;
    }
    else
    {
        // Wenn ein anderer Fehler aktiv ist (z.B. SSR stuck), lassen wir ihn latches.
        if (safetyErrorActive() && safetyErrorGet().type != SAFETY_ERR_BLOCK_SHORT)
        {
            for (uint8_t b = SHORT_BLOCK_MIN; b <= SHORT_BLOCK_MAX; b++)
                s_shortCondSinceMs[b] = 0;
        }
        else
        {
            for (uint8_t b = SHORT_BLOCK_MIN; b <= SHORT_BLOCK_MAX; b++)
            {
                const uint16_t iMa = g_bc.stromFiltered(b);

                if (iMa <= SHORT_CLEAR_MA)
                {
                    s_shortCondSinceMs[b] = 0;
                    continue;
                }

                if (iMa >= SHORT_I_MA)
                {
                    if (s_shortCondSinceMs[b] == 0)
                        s_shortCondSinceMs[b] = now;

                    if ((now - s_shortCondSinceMs[b]) >= SHORT_CONFIRM_MS)
                    {
                        safetyTriggerBlockShort(b);
                        for (uint8_t bb = SHORT_BLOCK_MIN; bb <= SHORT_BLOCK_MAX; bb++)
                            s_shortCondSinceMs[bb] = 0;
                        return;
                    }
                }
                else
                {
                    // "irgendwo Strom", aber noch kein Kurzschluss – Timer nicht laufen lassen
                    s_shortCondSinceMs[b] = 0;
                }
            }
        }
    }

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
    const bool b6noI        = (g_bc.stromFiltered(6) <= NO_CURRENT_MA);

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

    s_ssrStuckCondSinceMs[0] = 0;
    s_ssrStuckCondSinceMs[1] = 0;

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

    checkSsrStuck(now);
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

    s_ssrStuckCondSinceMs[0] = 0;
    s_ssrStuckCondSinceMs[1] = 0;

        safetyErrorSet(SAFETY_ERR_NOTAUS, 0);
        return;
    }

    s_emergencyActive = false;
}

void safetyTriggerBlockShort(uint8_t block)
{
    // Kurzschluss: Alles AUS + Safety-Lock
    safetySetSSR(SafetySSR::SSR_TRAFO_A, false);
    safetySetSSR(SafetySSR::SSR_TRAFO_B, false);

    s_ssrStuckCondSinceMs[0] = 0;
    s_ssrStuckCondSinceMs[1] = 0;

    s_blockReason = SAFETY_BLOCK_EMERGENCY;
    safetyErrorSet(SAFETY_ERR_BLOCK_SHORT, block);

#if MEGA2_DEBUG
    Serial.print(F("[SAFETY] EMERG_BLOCK_SHORT(B"));
    Serial.print(block);
    Serial.println(F(") -> SSR_A/B OFF, LOCK"));
#endif
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

        if (!block6Free)
        {
#if MEGA2_DEBUG
            Serial.print(F("[SAFETY] ACK blocked (EMERG_NOTHALT_SBHF): "));
            if (!block6Free)  Serial.print(F("Block6 still occupied "));
            Serial.println();
#endif
            return false;
        }

        // Bedingungen erfüllt -> Merker entfernen
        s_emergNothaltSbhfLatched = false;
        s_nothaltCondSinceMs      = 0;
    }

    // --------------------------------------------------
    // Block-Short: ACK erst wenn kein Strom mehr anliegt
    // --------------------------------------------------
    if (safetyErrorActive())
    {
        const auto e = safetyErrorGet();
        if (e.type == SAFETY_ERR_BLOCK_SHORT)
        {
            const uint8_t b = e.index;
            if (g_bc.stromFiltered(b) > SHORT_CLEAR_MA)
            {
#if MEGA2_DEBUG
                Serial.print(F("[SAFETY] ACK blocked (BLOCK_SHORT): Block"));
                Serial.print(b);
                Serial.println(F(" still has current"));
#endif
                return false;
            }
        }
        else if (e.type == SAFETY_ERR_SSR_STUCK)
        {
            const bool obenOk  = !isTrafoObenPowered();
            const bool untenOk = !isTrafoUntenPowered();

            if (!obenOk || !untenOk)
            {
#if MEGA2_DEBUG
                Serial.print(F("[SAFETY] ACK blocked (SSR_STUCK): "));
                if (!obenOk)  Serial.print(F("Trafo oben still powered "));
                if (!untenOk) Serial.print(F("Trafo unten still powered "));
                Serial.println();
#endif
                return false;
            }
        }

    }


    s_emergencyActive = false;
    s_blockReason     = SAFETY_BLOCK_NONE;

    safetyErrorClear();

    s_ssrStuckCondSinceMs[0] = 0;
    s_ssrStuckCondSinceMs[1] = 0;
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
