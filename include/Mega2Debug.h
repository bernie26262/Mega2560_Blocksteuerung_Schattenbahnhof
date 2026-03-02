#pragma once
#include <Arduino.h>  // for uint32_t

#ifndef DEBUG_LOOP_PERFORMANCE
#define DEBUG_LOOP_PERFORMANCE 0
#endif
#ifndef DEBUG_LOOP_PERFORMANCE_LOG
#define DEBUG_LOOP_PERFORMANCE_LOG 0
#endif

// Diagnose-Ausgabe
void mega2DebugDump();

// Optional: 1x/s Analog-Log (Strom/Trafo)
#if MEGA2_DEBUG_ANALOG_TICK
void mega2DebugAnalogTick(uint32_t nowMs);
#endif
// Optional: 1x/s Loop-Timing (Hz + max gap)
#if DEBUG_LOOP_PERFORMANCE
void mega2DebugLoopTick(uint32_t nowMs, uint32_t loopDtUs);
#endif
