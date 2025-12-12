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

// B3.1: explizite Quittierung
bool safetyResetEmergency();

// Hardware-nahe Aktion
void safetySetSSR(SafetySSR ssr, bool enable);
