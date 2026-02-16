#pragma once
#include <Arduino.h>  // for uint32_t
// Diagnose-Ausgabe
void mega2DebugDump();

// Optional: 1x/s Analog-Log (Strom/Trafo)
#if MEGA2_DEBUG_ANALOG_TICK
void mega2DebugAnalogTick(uint32_t nowMs);
#endif
