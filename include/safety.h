#pragma once

#include <Arduino.h>
#include "proto_common.h"

void safetyBegin();
void safetyUpdate();

bool safetyIsEmergencyActive();

void safetySetEmergency(bool active);
void safetySetSSR(SafetySSR ssr, bool enable);
 