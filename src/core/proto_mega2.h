#pragma once
#include <Arduino.h>
#include "proto_common.h"

// =============================================================================
// Mega2: Blockbelegung, Strom, Kontaktgleise, SBhf, Trafos, 4 Weichen (W12–W15)
// =============================================================================

static const uint8_t MEGA2_MAX_BLOCKS     = 16;
static const uint8_t MEGA2_MAX_SBH_GLEISE = 8;
static const uint8_t MEGA2_MAX_WEICHEN    = 4;   // W12–W15

enum class SbhfState : uint8_t {
    SBHF_IDLE     = 0,
    SBHF_EINFAHRT = 1,
    SBHF_AUSFAHRT = 2,
    SBHF_HOLD     = 3,
};

// Erweiterte Payload-Struktur -------------------------------------------------
struct Mega2StatusPayload
{
    uint16_t updateCounter;
    uint16_t bootId;
    uint32_t uptimeDiv100;

    uint8_t statusFlags;    // STATUS_FLAG_*
    uint8_t errorFlags;     // z.B. Überstrom
    uint8_t warnFlags;      // Warnungen
    uint8_t powerState;     // Booster/Trafo-Zustand

    // -----------------------------  
    // BLÖCKE
    // -----------------------------
    uint16_t blockBits;                              // Besetzt-Zustand (Strom)
    uint16_t blockCurrent_mA[MEGA2_MAX_BLOCKS];      // Strom pro Block

    // -----------------------------  
    // KONTAKTGLEISE JE BLOCK (NEU!)
    // -----------------------------
    uint16_t blockKontaktMainBits;   // Hauptkontakt je Block
    uint16_t blockKontaktBhf1Bits;   // Bahnhofskontakt #1 pro Block
    uint16_t blockKontaktBhf2Bits;   // Bahnhofskontakt #2 pro Block

    // -----------------------------  
    // SCHATTENBAHNHOF
    // -----------------------------
    uint16_t sbhfBits;               // SBhf-Gleise belegt
    SbhfState sbhfState;
    uint8_t sbhfSollGleis;
    uint8_t sbhfIstGleis;
    uint8_t reserved0;

    // -----------------------------  
    // SBHF-WEICHEN (W12–W15)
    // -----------------------------
    uint16_t weichenIstBits;
    uint16_t weichenSollBits;

    // -----------------------------  
    // TRAFOS
    // -----------------------------
    uint16_t trafoOben_mV;
    uint16_t trafoUnten_mV;
};

// Serializer
size_t buildMega2StatusFrame(uint8_t* outBuf, const Mega2StatusPayload& payload);
