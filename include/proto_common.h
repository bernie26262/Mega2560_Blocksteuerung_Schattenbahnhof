#pragma once
#include <Arduino.h>

// HINWEIS: Command-IDs müssen 1:1 mit Mega2Command übereinstimmen (siehe unten).
// =====================================================
//  Anlagen-Konstanten (fix)
// =====================================================
constexpr uint8_t M2_NUM_BLOCKS        = 9;  // 6 Strecke + 3 SBhf
constexpr uint8_t M2_NUM_SHADOW_TRACKS = 3;  // SBhf-Gleise

 
 // =====================================================
 //  Mega2 Analog Payload (I2C) – fixed point
 //  - vA10/vB10: 0.1V Schritte (V * 10)
 //  - i_mA[]   : Milliampere (0..~1500)
 //  Size: 24 bytes (Wire-safe)
 // =====================================================
 struct __attribute__((packed)) Mega2AnalogPayload
 {
     uint8_t  seq;          // increments per response
     uint8_t  flags;        // reserved (0 for now)
     uint16_t vA10;         // Trafo A voltage *10 (0.1V)
     uint16_t vB10;         // Trafo B voltage *10 (0.1V)
     uint16_t i_mA[M2_NUM_BLOCKS]; // B1..B9 currents in mA
 };
 static_assert(sizeof(Mega2AnalogPayload) == (2 + 2*2 + 2*M2_NUM_BLOCKS), "Mega2AnalogPayload size");

// =====================================================
//  I2C Commands (ESP -> Mega2)
// =====================================================
// Legacy-Namen (M2_CMD_*) – bitte keine neuen IDs hier ergänzen; stattdessen Mega2Command nutzen.
enum : uint8_t
{
    // --- Safety / Recovery ---
    M2_CMD_SET_NOTAUS         = 0x10, // [cmd, 0/1]
    M2_CMD_SET_SSR            = 0x11, // [cmd, ssrIndex, 0/1]
    M2_CMD_ACK_ERROR          = 0x12, // [cmd, mask]
    M2_CMD_POWER_ON           = 0x13,   // explizit: Leistung EIN
    M2_CMD_SBH_SELFTEST_RETRY  = 0x14, // [cmd] -> 0/1 (start SBHF selftest again)

    // --- Status Abfragen (read-only) ---
    M2_CMD_GET_SAFETY_STATUS  = 0x20, // -> Mega2SafetyStatus
    M2_CMD_GET_BLOCK_STATUS   = 0x21, // -> BlockStatus[M2_NUM_BLOCKS]
    M2_CMD_GET_SHADOW_STATUS  = 0x22, // -> ShadowYardStatus
    M2_CMD_GET_ENTRY_MATRIX  = 0x23,  // -> uint16_t[M2_NUM_BLOCKS] (FROM->TO)
    M2_CMD_GET_ENTRY_PREVIEW_MATRIX = 0x24, // -> uint16_t[M2_NUM_BLOCKS] (preview)
    M2_CMD_GET_ANALOG         = 0x25  // -> Mega2AnalogPayload (fixed point)
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
    // Historisch: "EMERGENCY" (2) – wird beibehalten, aber wir unterscheiden
    // künftig die Hauptursachen genauer.

    // 2 war historisch "EMERGENCY". Heute ist der präzise Grund: NOTAUS.
    SAFETY_BLOCK_NOTAUS    = 2,
    SAFETY_BLOCK_EMERGENCY = SAFETY_BLOCK_NOTAUS, // Alias nur für Kompatibilität
 
    SAFETY_BLOCK_SHORT     = 3,
    SAFETY_BLOCK_SSR_STUCK = 4
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


// =====================================================
//  Mega2 Pending-Mask (I2C) – digitale Änderungen (DRDY-getrieben)
//  - Bitmask tells the master which logical payloads have changed since last read.
//  - DRDY (active LOW) should stay asserted until pendingMask==0.
//  - Analog is intentionally NOT part of this mask (noise).
// =====================================================
enum : uint16_t {
    M2_PEND_SAFETY       = 1u << 0,
    M2_PEND_ENTRY        = 1u << 1,
    M2_PEND_ENTRY_PREV   = 1u << 2,
    M2_PEND_BLOCKS       = 1u << 3,
    M2_PEND_SHADOW       = 1u << 4,
    M2_PEND_TURNOUTS     = 1u << 5,  // reserved (future)

    M2_PEND_ALL_DIGITAL  = M2_PEND_SAFETY | M2_PEND_ENTRY | M2_PEND_ENTRY_PREV | M2_PEND_BLOCKS | M2_PEND_SHADOW | M2_PEND_TURNOUTS,
};

struct __attribute__((packed)) Mega2PendingMaskPayload
{
    uint8_t  seq;   // increments per response (debug / deglitch)
    uint8_t  rsv0;
    uint16_t mask;  // M2_PEND_* bits
};

struct __attribute__((packed)) Mega2TurnoutsPayload
{
    uint16_t sollMask;
    uint16_t istMask;
};



// Canonical command enum (Mega2 uses these symbols)
enum Mega2Command : uint8_t {
    CMD_GET_M2_SAFETY = 0x20,
    CMD_GET_M2_BLOCKS = 0x21,
    CMD_GET_M2_SBH    = 0x22,
    CMD_GET_M2_ENTRY  = 0x23,
    CMD_GET_M2_ENTRY_PREVIEW = 0x24,
    CMD_GET_M2_ANALOG = 0x25,
    CMD_GET_M2_PENDING_MASK = 0x26,
    CMD_GET_M2_TURNOUTS     = 0x27,
};

