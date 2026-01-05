#pragma once
#include <stdint.h>
#include "proto_mega2.h"

static constexpr uint8_t SYSTEM_STATUS_VERSION = 3;

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
};

// v3 (kompakt, <=32 Bytes) — PACKED für stabile I2C-Übertragung
struct __attribute__((packed)) SystemStatus
{
    uint8_t  version;
    uint8_t  nodeId;
    uint16_t size;

    uint32_t uptimeMs;
    uint16_t bootId;

    uint16_t flags;

    uint8_t  safetyErrorType;
    uint8_t  safetyErrorIndex;

    uint16_t blockOccupiedMask;

    uint8_t  sbhfState;
    uint8_t  sbhfOccupiedMask;

    uint8_t  sbhfCurrentGleis; // 0=none, 1..3
    uint8_t  _pad0;

    uint16_t turnoutSollMask;  // Bit0=W12..Bit3=W15
    uint16_t turnoutIstMask;

    uint16_t reserved;         // Variant A: allowedMask<<8 | warningMask
};

static_assert(sizeof(SystemStatus) == 26, "SystemStatus must be 26 bytes (packed)");
