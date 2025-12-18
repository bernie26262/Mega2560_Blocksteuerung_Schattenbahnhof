#pragma once

#include "proto_mega2.h"
#include "system/status_system.h"

class BlockController;
class ShadowYardController;

// Bestehende Builder (von Mega2I2C genutzt)
void buildMega2SafetyStatus(Mega2SafetyStatus& out);
void buildMega2BlockStatus(BlockStatus* out, const BlockController& bc);
void buildMega2ShadowStatus(ShadowYardStatus& out, const ShadowYardController& sy);

// Neuer Builder (Systemstatus)
void buildMega2SystemStatus(SystemStatus& out);

