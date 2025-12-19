#pragma once
#include <Arduino.h>

// =====================================================
//  Anlagen-Konstanten (fix)
// =====================================================
constexpr uint8_t M2_NUM_BLOCKS        = 9;  // 6 Strecke + 3 SBhf
constexpr uint8_t M2_NUM_SHADOW_TRACKS = 3;  // SBhf-Gleise

// =====================================================
//  I2C Commands (ESP -> Mega2)
// =====================================================
enum : uint8_t
{
    // --- Safety / Recovery ---
    M2_CMD_SET_NOTAUS         = 0x10, // [cmd, 0/1]
    M2_CMD_SET_SSR            = 0x11, // [cmd, ssrIndex, 0/1]
    M2_CMD_ACK_ERROR          = 0x12, // [cmd, mask]
    M2_CMD_POWER_ON           = 0x13,   // explizit: Leistung EIN

    // --- Status Abfragen (read-only) ---
    M2_CMD_GET_SAFETY_STATUS  = 0x20, // -> Mega2SafetyStatus
    M2_CMD_GET_BLOCK_STATUS   = 0x21, // -> BlockStatus[M2_NUM_BLOCKS]
    M2_CMD_GET_SHADOW_STATUS  = 0x22  // -> ShadowYardStatus
};

// =====================================================
//  Safety-SSR Indizes
// =====================================================
enum SafetySSR : uint8_t
{
    SSR_MAIN_ENABLE = 0,
    SSR_TRAFO_A     = 1,
    SSR_TRAFO_B     = 2,
    SSR__COUNT
};

// =====================================================
//  Safety-Status (global, klein)
// =====================================================

enum SafetyBlockReason : uint8_t
{
    SAFETY_BLOCK_NONE      = 0,
    SAFETY_BLOCK_BOOT      = 1,
    SAFETY_BLOCK_EMERGENCY = 2
};

struct Mega2SafetyStatus
{
    uint8_t notausActive;   // 0/1
    uint8_t ssrMask;        // Bit0=MAIN, Bit1=TRAFO_A, Bit2=TRAFO_B
    uint8_t errorFlags;     // global (z.B. Safety, I2C, Kurzschluss)

    uint8_t blockReason;    // NEU: siehe SafetyBlockReason
};

// =====================================================
//  Block-Status (pro Block identisch)
// =====================================================
//  Bits:
//   - kontakt     : Kontaktgleis aktiv
//   - stromEin    : Stromrelais EIN
//   - besetzt     : logisch berechnet
//   - kurzschluss : Stromfehler
//   - nothalt     : Safety wirkt auf Block
// =====================================================
struct BlockStatus
{
    uint8_t kontakt     : 1;
    uint8_t stromEin    : 1;
    uint8_t besetzt     : 1;
    uint8_t kurzschluss : 1;
    uint8_t nothalt     : 1;
    uint8_t reserved    : 3;

    uint16_t stromRaw;      // ADC-Wert Stromsensor
};

// =====================================================
//  Schattenbahnhof-Status (logischer Überblick)
// =====================================================
struct ShadowYardStatus
{
    uint8_t gleisBesetztMask;   // Bit 0..2
    uint8_t kontaktMask;        // Kontaktgleise SBhf
    uint8_t stromMask;          // Strom EIN pro Gleis

    uint8_t einfahrGleis;       // 0..2 oder 0xFF
    uint8_t ausfahrGleis;       // 0..2 oder 0xFF

    uint8_t modus;              // 0=seriell, 1=zufall
    uint8_t state;              // interner Automat (nur Anzeige)
};

enum Mega2Command : uint8_t {
    CMD_GET_M2_SAFETY = 0x20,
    CMD_GET_M2_BLOCKS = 0x21,
    CMD_GET_M2_SBH    = 0x22,
};

