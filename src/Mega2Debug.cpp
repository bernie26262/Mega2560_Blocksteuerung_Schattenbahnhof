#include "Mega2Debug.h"
#include "mega2_debug.h"
#include "BlockController.h"
#include "ShadowYardController.h"
#include "safety.h"

extern BlockController& blockController;
extern ShadowYardController& shadowController;

void mega2DebugDump()
{
    DBG_PRINTLN(F("=== Mega2 Diagnose ==="));

    // -------- SAFETY --------
    DBG_PRINT(F("Safety: "));
    DBG_PRINTLN(safetyIsEmergencyActive() ? F("NOT-AUS") : F("OK"));

    // -------- BLOCKS --------
    DBG_PRINTLN(F("Blocks:"));
    for (uint8_t i = 1; i <= blockController.count(); i++)
    {
        DBG_PRINT(F("  B"));
        DBG_PRINT(i);
        DBG_PRINT(F(": occ="));
        DBG_PRINT(blockController.isOccupied(i));
        DBG_PRINT(F(" I="));
        DBG_PRINT(blockController.stromFiltered(i));
        DBG_PRINTLN(F("mA"));
    }

    // -------- SHADOW YARD --------
    DBG_PRINT(F("SBHF state="));
    DBG_PRINT((uint8_t)shadowController.state());
    DBG_PRINT(F(" ausfahrGleis="));
    DBG_PRINTLN(shadowController.ausfahrGleis());

    DBG_PRINTLN(F("======================"));
}
