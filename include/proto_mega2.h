#pragma once
#include <Arduino.h>
#include "proto_common.h"

class BlockController;
class ShadowYardController;

static const uint8_t MEGA2_MAX_BLOCKS     = 16;
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
void buildMega2SafetyStatus(Mega2SafetyStatus& out);
void buildMega2BlockStatus(BlockStatus* out, const BlockController& bc);
void buildMega2ShadowStatus(ShadowYardStatus& out, const ShadowYardController& sy);
