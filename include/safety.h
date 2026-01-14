#pragma once

#include <Arduino.h>

// 0 = Hardware, 1 = Simulation (Debug ohne Anlage)
#ifndef MEGA2_SIM_MODE
#define MEGA2_SIM_MODE 0
#endif
#include "proto_common.h"

// Initialisierung
void safetyBegin();

// Zyklisches Update
void safetyUpdate();

// Status
bool safetyIsEmergencyActive();

// Aktionen
void safetySetEmergency(bool active);

// B3.1: explizite Quittierung
bool safetyResetEmergency();

// Hardware-nahe Aktion
void safetySetSSR(SafetySSR ssr, bool enable);

// Explizites Wiedereinschalten der Leistung (SSR)
// Liefert false, wenn Safety-Bedingungen nicht erfüllt sind
bool safetyPowerOn();

bool safetyIsSSR(SafetySSR ssr);

bool safetyIsPowerOn();

bool safetyIsLocked();

uint8_t safetyGetBlockReason();

void safetyDebugForceTrafoUntenPowered(bool on);
bool safetyDebugIsTrafoUntenForced();

// ------------------------------------------------------------
// SIM/Debug: Force synthetic block current (mA) for safety tests.
// ma==0 disables forcing for that block.
// Only meaningful/active in MEGA2_SIM_MODE builds.
// ------------------------------------------------------------
void safetyDebugForceBlockCurrentMa(uint8_t block, uint16_t ma);
uint16_t safetyDebugGetForcedBlockCurrentMa(uint8_t block);

// Debug/Test: Kurzschluss in Block manuell triggern
void safetyTriggerBlockShort(uint8_t block);
