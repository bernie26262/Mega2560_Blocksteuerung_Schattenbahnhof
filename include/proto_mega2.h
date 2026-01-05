#pragma once
#include <Arduino.h>
#include "proto_common.h"

class BlockController;
class ShadowYardController;

// Real existierende/benutzte Blöcke (B1..B9 inkl. SBHF-Blocks)
static constexpr uint8_t MEGA2_NUM_BLOCKS = 9;

// Kapazität / Maskenbreite / reservierte Slots (z.B. 16-bit Masks)
static constexpr uint8_t MEGA2_MAX_BLOCKS = 16;
static const uint8_t MEGA2_MAX_SBH_GLEISE = 3;
static const uint8_t MEGA2_MAX_WEICHEN    = 4;

struct Mega2Payload {
    uint32_t timestamp;

    bool blockOccupied[MEGA2_MAX_BLOCKS];
    bool sbhfOccupied[MEGA2_MAX_SBH_GLEISE];

    bool weichenIst[MEGA2_MAX_WEICHEN];
    bool weichenSoll[MEGA2_MAX_WEICHEN];

    uint16_t blockStroeme_mA[MEGA2_MAX_BLOCKS];

    bool nothaltAktiv;
    uint8_t sbhfState;
    uint8_t sbhfExit;
    uint8_t sbhfTarget;

    bool valid;
};

void mega2_buildPayload(Mega2Payload& p);

