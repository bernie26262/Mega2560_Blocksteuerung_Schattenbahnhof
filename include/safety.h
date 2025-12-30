#pragma once

#include <Arduino.h>
#include "proto_common.h"

// Initialisierung
void safetyBegin();

// Zyklisches Update
void safetyUpdate();

// Status
bool safetyIsEmergencyActive();

// Aktionen
void safetySetEmergency(bool active);

// Test/Detektor: Block-Kurzschluss (latch + SSR OFF + Lock)
void safetyTriggerBlockShort(uint8_t block);

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
