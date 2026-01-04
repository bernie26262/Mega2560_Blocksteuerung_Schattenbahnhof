#pragma once
#include <stdint.h>

// =====================================================
// Versionierung
// =====================================================
static constexpr uint8_t SYSTEM_STATUS_VERSION = 2;

// =====================================================
// Controller-ID
// =====================================================
enum SystemNodeId : uint8_t
{
    NODE_NONE  = 0,
    NODE_MEGA1 = 1,
    NODE_MEGA2 = 2,
};

// =====================================================
// Statusflags
// =====================================================
enum SystemStatusFlags : uint16_t
{
    SYS_OK               = 0,
    SYS_NOTAUS_ACTIVE    = 1 << 0,
    SYS_POWER_ON         = 1 << 1,
    SYS_ERROR_PRESENT    = 1 << 2,
    SYS_CONTROLLER_RESET = 1 << 3,
    SYS_WARNING_PRESENT  = 1 << 4,
};

// =====================================================
// Gemeinsames Systemstatus-Struct
// =====================================================
struct SystemStatus
{
    uint8_t  version;
    uint8_t  nodeId;
    uint16_t size;

    uint32_t uptimeMs;
    uint16_t bootId;

    uint16_t flags;

    // -----------------------------
    // SAFETY ERROR DETAILS
    // -----------------------------
    uint8_t  safetyErrorType;
    uint8_t  safetyErrorIndex;

    uint16_t blockOccupiedMask;

    uint8_t  sbhfState;
    uint8_t  sbhfOccupiedMask;

    // reserved: Variant A
    //   upper byte : SBHF allowedMask (bits 0..2 => Gleis1..3)
    //   lower byte : SBHF warningMask (implementation-specific)
    uint16_t reserved;
};
