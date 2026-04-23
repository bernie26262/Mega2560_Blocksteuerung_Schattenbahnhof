#pragma once
#include <stdint.h>

enum ErrorCause : uint8_t
{
    ERR_CAUSE_NONE                 = 0,
    ERR_CAUSE_SBH_FALSE_ENTRY      = 1,
    ERR_CAUSE_SBH_EXIT_TIMEOUT     = 2,
    ERR_CAUSE_SBH_WEICHE           = 3,
    ERR_CAUSE_DOUBLE_OCCUPANCY     = 4,
    ERR_CAUSE_BLOCK_SHORT          = 5,
    ERR_CAUSE_CONTROLLER_FAULT     = 6,
    ERR_CAUSE_EXTERNAL_ESTOP       = 7,
    ERR_CAUSE_SBH_CONTROLLER_FAULT = 8,
    ERR_CAUSE_SBH_ENTRY_TIMEOUT    = 9,
    ERR_CAUSE_SBH_ENTRY_WRONG_TRACK = 10,
};

enum ErrorDetailCode : uint8_t
{
    ERR_DETAIL_NONE = 0,

    // SBHF false entry
    ERR_DETAIL_SBH_FALSE_ENTRY_STOPZONE = 1,

    // SBHF controller fault
    ERR_DETAIL_SBH_CTRL_TARGET_OCC_POWERED = 10,
    ERR_DETAIL_SBH_CTRL_INVALID_EXIT_GLEIS = 11,

    // Double occupancy
    ERR_DETAIL_DOUBLE_OCC_HARD_TRIP     = 20,
    ERR_DETAIL_DOUBLE_OCC_ADAPTIVE_TRIP = 21,

    // Controller fault
    ERR_DETAIL_CTRL_TICK_GAP  = 30,
    ERR_DETAIL_CTRL_INVARIANT = 31,

    // SBHF entry timeout
    ERR_DETAIL_SBH_ENTRY_TIMEOUT_WAIT_S  = 40,
    ERR_DETAIL_SBH_ENTRY_TIMEOUT_WAIT_GF = 41,

    // SBHF wrong-track marker (actual observed S-contact)
    ERR_DETAIL_SBH_ENTRY_WRONG_TRACK_S12 = 50,
    ERR_DETAIL_SBH_ENTRY_WRONG_TRACK_S13 = 51,
    ERR_DETAIL_SBH_ENTRY_WRONG_TRACK_S14 = 52,
};

struct SafetyErrorInfo
{
    ErrorCause cause;
    uint8_t index;      // Block / Gleis / Weiche / 0 = nicht relevant
    uint8_t detailCode; // Feindetail innerhalb der Ursache
};

// Globale Fehler-API
void safetyErrorSet(ErrorCause cause, uint8_t index = 0, uint8_t detailCode = ERR_DETAIL_NONE);
void safetyErrorClear();
const SafetyErrorInfo& safetyErrorGet();
bool safetyErrorActive();
