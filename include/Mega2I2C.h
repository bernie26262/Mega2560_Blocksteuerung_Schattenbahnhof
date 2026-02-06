#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "proto_mega2.h"

void megaI2C_begin();
void megaI2C_update();

// DataReady helper: mark specific digital payloads as pending (see proto_common.h M2_PEND_*)
// Keeps DRDY active LOW until pendingMask==0.
void megaI2C_setPending(uint16_t bits);

// Backward compatible helper: mark all digital payloads as "changed" (DRDY active LOW)
void megaI2C_markDataReady();

// Returns true if the master recently requested diag sensor snapshots.
// Used to avoid latching DRDY for diag-only signals when no diag client is active.
bool megaI2C_diagIsActive();
