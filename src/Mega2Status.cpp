#include "proto_mega2.h"
#include "BlockController.h"
#include "ShadowYardController.h"
#include "Mega2PowerControl.h"

// --------------------------------------------------
// SAFETY (D3.1 – final, Mega2-only)
// --------------------------------------------------
void buildMega2SafetyStatus(Mega2SafetyStatus& out)
{
    out.notausActive = 0;
    out.ssrMask     = 0;
    out.errorFlags  = 0;

    extern ShadowYardController g_sbhf;
    extern Mega2PowerControl    g_power;

    // Hard-Error = sicherheitsrelevant
    if (g_sbhf.state() == ShadowYardController::SBhfState::Error)
    {
        out.notausActive = 1;
        out.errorFlags |= 0x01;   // Bit 0 = Hard-Error aktiv
    }

    // SSR-Zustände (Ist = zuletzt gesetzt)
    if (g_power.isTrafoAEnabled())
        out.ssrMask |= (1 << 1);

    if (g_power.isTrafoBEnabled())
        out.ssrMask |= (1 << 2);
}

// --------------------------------------------------
// BLOCKS (unverändert)
// --------------------------------------------------
void buildMega2BlockStatus(BlockStatus* out, const BlockController& ctrl)
{
    for (uint8_t i = 0; i < ctrl.count(); i++)
    {
        out[i].besetzt  = ctrl.isOccupied(i);
        out[i].stromRaw = ctrl.stromFiltered(i);
    }
}

// --------------------------------------------------
// SHADOW YARD (D3.1 – Anzeige, ehrlich)
// --------------------------------------------------
void buildMega2ShadowStatus(ShadowYardStatus& out,
                            const ShadowYardController& sy)
{
    out.gleisBesetztMask = 0;
    out.kontaktMask      = 0;     // aktuell nicht ableitbar
    out.stromMask        = 0;     // aktuell nicht ableitbar

    out.einfahrGleis     = 0xFF;  // unbekannt
    out.ausfahrGleis     = 0xFF;

    out.modus            = 0;     // Default / unbekannt
    out.state            = static_cast<uint8_t>(sy.state());

    extern BlockController g_bc;

    // Bit 0 → SBhf-Gleis 1 (Block 7)
    if (g_bc.isOccupied(7))
        out.gleisBesetztMask |= (1 << 0);

    // Bit 1 → SBhf-Gleis 2 (Block 8)
    if (g_bc.isOccupied(8))
        out.gleisBesetztMask |= (1 << 1);

    // Bit 2 → SBhf-Gleis 3 (Block 9)
    if (g_bc.isOccupied(9))
        out.gleisBesetztMask |= (1 << 2);

    uint8_t g = sy.ausfahrGleis();
    if (g >= 1 && g <= 3)
        out.ausfahrGleis = g - 1;   // 0..2 gemäß Proto
}
