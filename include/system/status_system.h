#pragma once
#include <stdint.h>
#include "proto_mega2.h"

// Wire-format version.
// IMPORTANT: Mega1 and Mega2 must share the same version/size.
// Selftest runtime information is encoded in sbhfOccupiedMask META bits (see Mega2Status.cpp).
static constexpr uint8_t SYSTEM_STATUS_VERSION = 4;

enum SystemNodeId : uint8_t
{
    NODE_NONE  = 0,
    NODE_MEGA1 = 1,
    NODE_MEGA2 = 2,
};

enum SystemStatusFlags : uint16_t
{
    SYS_OK               = 0,
    SYS_NOTAUS_ACTIVE    = 1 << 0,
    SYS_POWER_ON         = 1 << 1,
    SYS_ERROR_PRESENT    = 1 << 2,
    SYS_CONTROLLER_RESET = 1 << 3,
    SYS_WARNING_PRESENT  = 1 << 4,
    // Node is in diagnose/service test mode (automation paused).
    // (Mega1 may leave this 0; Mega2 sets it when DIAG_TEST is active.)
    SYS_MODE_DIAG        = 1 << 5,
};

// v4 (kompakt) — PACKED für stabile I2C-Übertragung
struct __attribute__((packed)) SystemStatus
{
    uint8_t  version;
    uint8_t  nodeId;
    uint16_t size;

    uint32_t uptimeMs;
    uint16_t bootId;

    uint16_t flags;

    uint8_t  errorCause;
    uint8_t  errorIndex;
    uint8_t  errorDetailCode;
    uint8_t  reservedErr;

    uint16_t blockOccupiedMask;

    uint8_t  sbhfState;
    uint8_t  sbhfOccupiedMask;

    uint8_t  sbhfCurrentGleis; // 0=none, 1..3
    uint8_t  sbhfFlags;
    // sbhfFlags bits:
    // bit0 = block5ToSbhfActive
    // other bits reserved

    uint16_t turnoutSollMask;  // Bit0=W12..Bit3=W15
    uint16_t turnoutIstMask;

    uint16_t reserved;         // Variant A: allowedMask<<8 | warningMask
};

static_assert(sizeof(SystemStatus) == 28, "SystemStatus must be 28 bytes (packed)");
