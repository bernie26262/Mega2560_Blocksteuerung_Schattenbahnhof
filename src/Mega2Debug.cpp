#include "Mega2Debug.h"
#include "mega2_debug.h"
#include "BlockController.h"
#include "ShadowYardController.h"
#include "safety.h"
#include "safety_error.h"

#include "Mega2PowerControl.h"
#include "SensorKontakt.h"

extern BlockController& blockController;
extern ShadowYardController& shadowController;
extern Mega2PowerControl g_power;
extern SensorKontakt k_nothalt;

void mega2DebugDump()
{
    DBG_PRINTLN(F("=== Mega2 Diagnose ==="));

    // -------- SAFETY --------
    DBG_PRINT(F("Safety: "));
    DBG_PRINTLN(safetyIsEmergencyActive() ? F("NOT-AUS") : F("OK"));

    const SafetyErrorInfo& err = safetyErrorGet();
    DBG_PRINT(F("SafetyErr type="));
    DBG_PRINT((uint8_t)err.type);
    DBG_PRINT(F(" idx="));
    DBG_PRINTLN(err.index);

    // -------- STOPZONE / REVERSE-ENTRY DEBUG --------
    DBG_PRINT(F("StopzoneActive="));
    DBG_PRINT(g_power.isNothaltActive());
    DBG_PRINT(F(" k_nothalt(raw)="));
    DBG_PRINT(k_nothalt.raw());
    DBG_PRINT(F(" I6="));
    DBG_PRINT(blockController.stromFiltered(6));
    DBG_PRINTLN(F("mA"));
    DBG_PRINTF(" allowedMask=0x%02X warningMask=0x%02X\n",
               shadowController.allowedGleisMask(),
               shadowController.warningMask());

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

    DBG_PRINTF("SBHF allowedMask=0x%02X warnMask=0x%02X\n", shadowController.allowedGleisMask(), shadowController.warningMask());

    DBG_PRINTLN(F("======================"));
}
