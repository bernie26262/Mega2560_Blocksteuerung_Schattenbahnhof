#include "Mega2Status.h"

#include <Arduino.h>

#include "BlockController.h"
#include "ShadowYardController.h"
#include "safety.h"
#include "safety_error.h"

// globale Controller
extern BlockController      g_bc;
extern ShadowYardController g_sbhf;
extern uint16_t             g_bootId;

// --------------------------------------------------
// SAFETY
// --------------------------------------------------
void buildMega2SafetyStatus(Mega2SafetyStatus& out)
{
    out.notausActive = safetyIsEmergencyActive();
    out.ssrMask      = 0;   // aktuell ungenutzt
    out.errorFlags   = 0;
}

// --------------------------------------------------
// BLOCKS
// --------------------------------------------------
void buildMega2BlockStatus(BlockStatus* out,
                           const BlockController& bc)
{
    for (uint8_t i = 0; i < bc.count(); i++)
    {
        out[i].besetzt  = bc.isOccupied(i);
        out[i].stromRaw = bc.stromFiltered(i);
    }
}

// --------------------------------------------------
// SHADOW YARD
// --------------------------------------------------
void buildMega2ShadowStatus(ShadowYardStatus& out,
                            const ShadowYardController& sy)
{
    out.state        = static_cast<uint8_t>(sy.state());
    out.ausfahrGleis = sy.ausfahrGleis();
}

// --------------------------------------------------
// SYSTEM STATUS (neu)
// --------------------------------------------------
void buildMega2SystemStatus(SystemStatus& out)
{
    out.version = SYSTEM_STATUS_VERSION;
    out.nodeId  = NODE_MEGA2;
    out.size    = sizeof(SystemStatus);

    out.uptimeMs = millis();
    out.bootId   = g_bootId;

    // -----------------------------
    // FLAGS
    // -----------------------------
    out.flags = SYS_OK;

    if (safetyIsEmergencyActive())
        out.flags |= SYS_NOTAUS_ACTIVE;

    if (safetyIsLocked())
        out.flags |= SYS_ERROR_PRESENT;

    if (safetyIsPowerOn())
        out.flags |= SYS_POWER_ON;

    // -----------------------------
    // SAFETY ERROR DETAILS (NEU)
    // -----------------------------
    const SafetyErrorInfo& err = safetyErrorGet();
    out.safetyErrorType  = static_cast<uint8_t>(err.type);
    out.safetyErrorIndex = err.index;

    // -----------------------------
    // BLOCKS
    // -----------------------------
    out.blockOccupiedMask = 0;
    for (uint8_t i = 1; i <= 9; i++)
        if (g_bc.isOccupied(i))
            out.blockOccupiedMask |= (1 << (i - 1));

    // -----------------------------
    // SCHATTENBAHNHOF
    // -----------------------------
    out.sbhfState = static_cast<uint8_t>(g_sbhf.state());

    out.sbhfOccupiedMask = 0;
    for (uint8_t i = 0; i < 3; i++)
        if (g_bc.isOccupied(7 + i))
            out.sbhfOccupiedMask |= (1 << i);

    out.reserved = 0;
}
