#pragma once
#include <stdint.h>

enum SafetyErrorType : uint8_t
{
    SAFETY_ERR_NONE        = 0,
    SAFETY_ERR_NOTAUS      = 1,
    SAFETY_ERR_BLOCK_SHORT = 2,
    SAFETY_ERR_SBH_WEICHE  = 3,
    SAFETY_ERR_SSR_STUCK   = 4
};

struct SafetyErrorInfo
{
    SafetyErrorType type;
    uint8_t index;   // Block / Weiche / 0 = nicht relevant
};

// Globale Fehler-API
void safetyErrorSet(SafetyErrorType type, uint8_t index = 0);
void safetyErrorClear();
const SafetyErrorInfo& safetyErrorGet();
bool safetyErrorActive();
