#include "Mega2Debug.h"
#include "mega2_debug.h"
#include "BlockController.h"
#include <math.h>
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

    // Keep this tick lightweight to minimize loop stalls:
    // - no float printing (AVR float->string is expensive)
    // - avoid sqrt in this tick (use absDev as cheap proxy)
    // - split output across 4 seconds (phase 0..3)

    static uint8_t s_phase = 0;
    s_phase = (uint8_t)((s_phase + 1u) & 0x03u);

    switch (s_phase) {
        case 0:
            // I1
            DBG_PRINT(F("[AN] I1 raw=")); DBG_PRINT(strom1.raw());
            DBG_PRINT(F(" off="));       DBG_PRINT(strom1.offset());
            DBG_PRINT(F(" abs="));       DBG_PRINT(strom1.absDev());
            DBG_PRINT(F(" act="));       DBG_PRINTLN(strom1.overThreshold());
            break;

        case 1:
            // ISB1
            DBG_PRINT(F("[AN] ISB1 raw=")); DBG_PRINT(stromSbhf1.raw());
            DBG_PRINT(F(" off="));         DBG_PRINT(stromSbhf1.offset());
            DBG_PRINT(F(" abs="));         DBG_PRINT(stromSbhf1.absDev());
            DBG_PRINT(F(" act="));         DBG_PRINTLN(stromSbhf1.overThreshold());
            break;

        case 2:
            // TOben (cached ints from SensorTrafoAC::update())
            DBG_PRINT(F("[AN] TOben=")); DBG_PRINT(g_trafoOben.rmsTrafo_cV()); DBG_PRINT(F("cV"));
            DBG_PRINT(F(" adc="));       DBG_PRINT(g_trafoOben.rmsAdc_mV());   DBG_PRINT(F("mV"));
            DBG_PRINT(F(" pow="));       DBG_PRINTLN(g_trafoOben.isPowered());
            break;

        default:
            // TUnten (cached ints from SensorTrafoAC::update())
            DBG_PRINT(F("[AN] TUnten=")); DBG_PRINT(g_trafoUnten.rmsTrafo_cV()); DBG_PRINT(F("cV"));
            DBG_PRINT(F(" adc="));        DBG_PRINT(g_trafoUnten.rmsAdc_mV());   DBG_PRINT(F("mV"));
            DBG_PRINT(F(" pow="));        DBG_PRINTLN(g_trafoUnten.isPowered());
            break;
    }
}

#endif

// ------------------------------------------------------------
// Optional: 1x/s Loop-Timing – temporär fürs Bring-up
// Aktivieren per build_flag: -DDEBUG_LOOP_PERFORMANCE=1
 // ------------------------------------------------------------
#ifndef DEBUG_LOOP_PERFORMANCE
#define DEBUG_LOOP_PERFORMANCE 0
#endif

#ifndef DEBUG_LOOP_PERFORMANCE_LOG
#define DEBUG_LOOP_PERFORMANCE_LOG 0
#endif

#if DEBUG_LOOP_PERFORMANCE
static uint32_t s_loopTickLastMs = 0;
static uint32_t s_loopCount = 0;
static uint32_t s_loopMaxGapUs = 0;

void mega2DebugLoopTick(uint32_t nowMs, uint32_t loopDtUs)
{
    s_loopCount++;
    if (loopDtUs > s_loopMaxGapUs) s_loopMaxGapUs = loopDtUs;

    if (nowMs - s_loopTickLastMs < 1000u) return;
    const uint32_t dt = nowMs - s_loopTickLastMs;
    s_loopTickLastMs = nowMs;

    const uint32_t hz = (dt > 0) ? (uint32_t)((s_loopCount * 1000UL) / dt) : 0;
    
#if DEBUG_LOOP_PERFORMANCE_LOG
    DBG_PRINTF("[LOOP] hz=%lu maxGapUs=%lu\n", (unsigned long)hz, (unsigned long)s_loopMaxGapUs);
#endif

    s_loopCount = 0;
    s_loopMaxGapUs = 0;
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
