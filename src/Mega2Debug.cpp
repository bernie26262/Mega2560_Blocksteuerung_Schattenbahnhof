#include "Mega2Debug.h"
#include "mega2_debug.h"
#include "BlockController.h"
#include "ShadowYardController.h"
#include "safety.h"
#include "safety_error.h"

#include "Mega2PowerControl.h"
#include "SensorKontakt.h"
#include "SensorStrom.h"
#include "SensorTrafoAC.h"
// ------------------------------------------------------------
// Optional: 1x/s Analog-Tick (Strom/Trafo) – temporär fürs Bring-up
// Aktivieren per build_flag: -DMEGA2_DEBUG_ANALOG_TICK=1
// ------------------------------------------------------------
#ifndef MEGA2_DEBUG_ANALOG_TICK
#define MEGA2_DEBUG_ANALOG_TICK 0
#endif

extern BlockController& blockController;
extern ShadowYardController& shadowController;
extern Mega2PowerControl g_power;
extern SensorKontakt k_nothalt;

#if MEGA2_DEBUG_ANALOG_TICK
extern SensorStrom strom1;
extern SensorStrom stromSbhf1;
extern SensorTrafoAC g_trafoOben;
extern SensorTrafoAC g_trafoUnten;
static uint32_t s_lastAnalogTickMs = 0;

void mega2DebugAnalogTick(uint32_t nowMs)
{
    if (nowMs - s_lastAnalogTickMs < 1000u) return;
    s_lastAnalogTickMs = nowMs;

    DBG_PRINT(F("[AN] I1 raw="));
    DBG_PRINT(strom1.raw());
    DBG_PRINT(F(" off="));
    DBG_PRINT(strom1.offset());
    DBG_PRINT(F(" rms="));
    DBG_PRINT(strom1.rmsCounts());
    DBG_PRINT(F(" act="));
    DBG_PRINT(strom1.overThreshold());

    DBG_PRINT(F(" | ISB1 raw="));
    DBG_PRINT(stromSbhf1.raw());
    DBG_PRINT(F(" off="));
    DBG_PRINT(stromSbhf1.offset());
    DBG_PRINT(F(" rms="));
    DBG_PRINT(stromSbhf1.rmsCounts());
    DBG_PRINT(F(" act="));
    DBG_PRINT(stromSbhf1.overThreshold());

    DBG_PRINT(F(" | TOben="));
    DBG_PRINT(g_trafoOben.rms());
    DBG_PRINT(F("V pow="));
    DBG_PRINT(g_trafoOben.isPowered());
    DBG_PRINT(F(" | TUnten="));
    DBG_PRINT(g_trafoUnten.rms());
    DBG_PRINT(F("V pow="));
    DBG_PRINTLN(g_trafoUnten.isPowered());
}
#endif


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

#if MEGA2_DEBUG_ANALOG
    // -------- ANALOG (1x Snapshot) --------
    // Strom: nur exemplarisch B1 (strom1). Weitere Kanäle bei Bedarf ergänzen.
    DBG_PRINT(F("Analog: I1 raw="));
    DBG_PRINT(strom1.raw());
    DBG_PRINT(F(" off="));
    DBG_PRINT(strom1.offset());
    DBG_PRINT(F(" rms="));
    DBG_PRINT(strom1.rmsCounts());
    DBG_PRINT(F(" thr="));
    // Kein Getter vorhanden -> hier nur Status ausgeben
    DBG_PRINT(F(" act="));
    DBG_PRINT(strom1.overThreshold());
    DBG_PRINT(F(" | TrafoOben Vrms="));
    DBG_PRINT(g_trafoOben.rms());
    DBG_PRINT(F(" pow="));
    DBG_PRINT(g_trafoOben.isPowered());
    DBG_PRINT(F(" | TrafoUnten Vrms="));
    DBG_PRINT(g_trafoUnten.rms());
    DBG_PRINT(F(" pow="));
    DBG_PRINTLN(g_trafoUnten.isPowered());
#endif


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

#if MEGA2_DEBUG_ANALOG_TICK
void mega2DebugAnalogTick(uint32_t nowMs);
#endif
