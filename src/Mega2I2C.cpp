#include <Wire.h>
#include <Arduino.h>
#include <string.h>

#ifndef EE_DEBUG_DIAG_WRITE
#define EE_DEBUG_DIAG_WRITE 0
#endif
#if EE_DEBUG_DIAG_WRITE
  #define EE_DIAGW(fmt, ...) DBG_PRINTF("[DIAGW] " fmt "\n", ##__VA_ARGS__)
#else
  #define EE_DIAGW(...) do{}while(0)
#endif

extern uint16_t g_bootId;
#include "config.h"  // g_blocks[]
#include "Block.h"
#include "BlockController.h"
#include "ShadowYardController.h"

#include "SensorKontakt.h"
#include "PulseSensor.h"

#include "proto_common.h"
#include "proto_mega2.h"

#include "Mega2I2C.h"
#include "Mega2Status.h"
#include "system/status_system.h"
#include "safety.h"
#include "mega2_debug.h"
#include "Mega2RunMode.h"

#include "mega2_pins.h" // relay pin constants

// --- DIAG write response codes (1 byte) ---
// 0x01 = OK
// 0x02 = BUSY (pulse already active)
// 0x00 = FAIL/DENY (generic for now)
#ifndef M2_DIAG_RESP_OK
#define M2_DIAG_RESP_OK   0x01
#define M2_DIAG_RESP_BUSY 0x02
#define M2_DIAG_RESP_FAIL 0x00
#endif

 
// Analog payload (fixed point): see proto_common.h Mega2AnalogPayload
// ------------------------------------------------------------
// DataReady (Mega2 -> ESP): active LOW, latched until a digital read is served
// ------------------------------------------------------------
// constexpr uint8_t PIN_DATA_READY_M2 = 12; (ist in mega2_pins.h definiert)

// ------------------------------------------------------------
// Externe Controller aus main.cpp
// ------------------------------------------------------------
extern BlockController& blockController;
extern ShadowYardController& shadowController;

// ------------------------------------------------------------
// Externer Systemstatus (in main.cpp gebaut)
// ------------------------------------------------------------
extern SystemStatus g_systemStatus;

// ------------------------------------------------------------
// Externe Sensoren aus main.cpp (für Diag-Snapshot)
// ------------------------------------------------------------
extern SensorKontakt k_block1;
extern SensorKontakt k_block2;
extern SensorKontakt k_block3;
extern SensorKontakt k_block4;
extern SensorKontakt k_block5;
extern SensorKontakt k_block6;

extern SensorKontakt k_sbhf1;
extern SensorKontakt k_sbhf2;
extern SensorKontakt k_sbhf3;

extern SensorKontakt k_nothalt;

extern SensorKontakt k_bhf2a;
extern SensorKontakt k_bhf2b;
extern SensorKontakt k_bhf4a;
extern SensorKontakt k_bhf4b;

extern PulseSensor g_s11;
extern PulseSensor g_s12;
extern PulseSensor g_s13;
extern PulseSensor g_s14;
extern PulseSensor g_s15;
extern PulseSensor g_s16;
// ------------------------------------------------------------
// Interner Command-Response-Zustand
// ------------------------------------------------------------

// ------------------------------------------------------------
// Helper: Nachbarschaft (deine Topologie)
// 1->2, 2->3, 3->4, 4->1, 4->5, 5->7/8/9, 7/8/9->6, 6->4
// ------------------------------------------------------------
static bool isNeighbor(uint8_t fromBlock, uint8_t toBlock)
{
    if (fromBlock == 1 && toBlock == 2) return true;
    if (fromBlock == 2 && toBlock == 3) return true;
    if (fromBlock == 3 && toBlock == 4) return true;
    if (fromBlock == 4 && toBlock == 1) return true;
    if (fromBlock == 4 && toBlock == 5) return true;

    if (fromBlock == 5 && (toBlock == 7 || toBlock == 8 || toBlock == 9)) return true;
    if ((fromBlock == 7 || fromBlock == 8 || fromBlock == 9) && toBlock == 6) return true;

    if (fromBlock == 6 && toBlock == 4) return true;

    return false;
}

// ------------------------------------------------------------
// Helper: Entry matrices (shared by onRequest + DRDY change-scan)
// ------------------------------------------------------------
static void buildEntryMatrix(uint16_t entry[M2_NUM_BLOCKS])
{
    // Antwort: uint16_t[M2_NUM_BLOCKS] (FROM->TO bitmask)
    // Index: from-1; Bit(to-1)=1 => Einfahrt erlaubt
    for (uint8_t i = 0; i < M2_NUM_BLOCKS; i++) entry[i] = 0;

    for (uint8_t from = 1; from <= M2_NUM_BLOCKS; from++)
    {
        uint16_t mask = 0;
        for (uint8_t to = 1; to <= M2_NUM_BLOCKS; to++)
        {
            if (!isNeighbor(from, to))
                continue;

            if (blockController.canEnter(from, to))
                mask |= (1u << (to - 1));
        }
        entry[from - 1] = mask;
    }
}

static void buildEntryPreviewMatrix(uint16_t entry[M2_NUM_BLOCKS])
{
    // Antwort: uint16_t[M2_NUM_BLOCKS] (FROM->TO bitmask)
    // Semantik: "prinzipiell möglich" (Preview) – Topologie + Ziel frei + keine globale Safety-Sperre
    for (uint8_t i = 0; i < M2_NUM_BLOCKS; i++) entry[i] = 0;

    // Globaler Lock -> alles rot
    if (safetyIsLocked())
        return;

    for (uint8_t from = 1; from <= M2_NUM_BLOCKS; from++)
    {
        uint16_t mask = 0;
        for (uint8_t to = 1; to <= M2_NUM_BLOCKS; to++)
        {
            if (!isNeighbor(from, to))
                continue;

            // Preview ignoriert Speziallogik wie entryGranted() (z.B. Block4 Merge)
            // und fragt nur: "Zielblock frei?"
            if (!blockController.isOccupied(to))
                mask |= (1u << (to - 1));
        }
        entry[from - 1] = mask;
    }
}


static bool    s_cmdResponsePending = false;
static uint8_t s_cmdResponseOk      = 0;
static uint8_t s_pendingResponse    = 0;
static uint8_t s_analogSeq          = 0;
static bool    s_drdyActiveLow      = false;

// Pending mask for DRDY-driven digital payloads (see proto_common.h M2_PEND_*)
static volatile uint16_t s_pendingMask = 0;
static uint8_t           s_pendingSeq  = 0;
static uint32_t          s_lastDiagReadMs = 0;  // millis of last CMD_GET_M2_DIAG_SENSORS served

// Relay telemetry (diag-only, active-low pin levels)
static Mega2DiagRelaysPayload s_diagRelaysSnap{};
static uint32_t               s_lastDiagRelaysReadMs = 0; // millis of last CMD_GET_M2_DIAG_RELAYS served

// Diag snapshot (kontakt + schaltgleise)
static Mega2DiagSensorsPayload s_diagSnap{};
static bool     s_diagHasLast = false;
static uint16_t s_diagLastKontaktLevel = 0;
static uint8_t  s_diagLastSchaltLevel  = 0;
static uint8_t  s_diagLastSchaltRise[M2_DIAG_NUM_SCHALT]{};
static uint8_t  s_diagLastSchaltFall[M2_DIAG_NUM_SCHALT]{};

// Helpers: packed 4-bit counters in Mega2DiagSensorsPayload::kontaktRise4/kontaktFall4
static inline uint8_t getNibble(const uint8_t* a, uint8_t idx)
{
    const uint8_t b = a[idx >> 1];
    return (idx & 1) ? (uint8_t)(b >> 4) : (uint8_t)(b & 0x0F);
}
static inline void setNibble(uint8_t* a, uint8_t idx, uint8_t v)
{
    const uint8_t bi = (uint8_t)(idx >> 1);
    const uint8_t shift = (idx & 1) ? 4 : 0;
    const uint8_t mask = (uint8_t)(0x0F << shift);
    a[bi] = (uint8_t)((a[bi] & ~mask) | ((v & 0x0F) << shift));
}
static inline void incNibble(uint8_t* a, uint8_t idx)
{
    uint8_t v = getNibble(a, idx);
    v = (uint8_t)((v + 1) & 0x0F);
    setNibble(a, idx, v);
}

static inline uint32_t buildDiagRelaysMaskActiveLow()
{
    uint32_t m = 0;
    auto add = [&](uint8_t bit, uint8_t pin){
        if (digitalRead(pin) == LOW) m |= (1UL << bit);
    };

    uint8_t b = 0;
    add(b++, PIN_W12_GERADE);
    add(b++, PIN_W12_ABBIEGEN);
    add(b++, PIN_W13_GERADE);
    add(b++, PIN_W13_ABBIEGEN);
    add(b++, PIN_W14_GERADE);
    add(b++, PIN_W14_ABBIEGEN);
    add(b++, PIN_W15_GERADE);
    add(b++, PIN_W15_ABBIEGEN);

    add(b++, PIN_RELAY_BLOCK1_NACH2);
    add(b++, PIN_RELAY_BLOCK2_NACH3);
    add(b++, PIN_RELAY_BLOCK3_NACH4);
    add(b++, PIN_RELAY_BLOCK4_NACH1);
    add(b++, PIN_RELAY_BLOCK4_NACH5);
    add(b++, PIN_RELAY_BLOCK5_NACH_SBH);
    add(b++, PIN_RELAY_SBH_GL1_NACH6);
    add(b++, PIN_RELAY_SBH_GL2_NACH6);
    add(b++, PIN_RELAY_SBH_GL3_NACH6);
    add(b++, PIN_RELAY_BLOCK6_NACH4);

    add(b++, PIN_RELAY_NOTHALT);
    add(b++, PIN_RELAY_TRAFO_OBEN_CUT);
    add(b++, PIN_RELAY_TRAFO_UNTEN_CUT);

    return m;
}

// Map diag relay bit index -> physical pin (must match buildDiagRelaysMaskActiveLow order)
static inline bool diagRelayBitToPin(uint8_t bit, uint8_t& pinOut)
{
    switch (bit)
    {
        case 0:  pinOut = PIN_W12_GERADE; break;
        case 1:  pinOut = PIN_W12_ABBIEGEN; break;
        case 2:  pinOut = PIN_W13_GERADE; break;
        case 3:  pinOut = PIN_W13_ABBIEGEN; break;
        case 4:  pinOut = PIN_W14_GERADE; break;
        case 5:  pinOut = PIN_W14_ABBIEGEN; break;
        case 6:  pinOut = PIN_W15_GERADE; break;
        case 7:  pinOut = PIN_W15_ABBIEGEN; break;

        case 8:  pinOut = PIN_RELAY_BLOCK1_NACH2; break;
        case 9:  pinOut = PIN_RELAY_BLOCK2_NACH3; break;
        case 10: pinOut = PIN_RELAY_BLOCK3_NACH4; break;
        case 11: pinOut = PIN_RELAY_BLOCK4_NACH1; break;
        case 12: pinOut = PIN_RELAY_BLOCK4_NACH5; break;
        case 13: pinOut = PIN_RELAY_BLOCK5_NACH_SBH; break;
        case 14: pinOut = PIN_RELAY_SBH_GL1_NACH6; break;
        case 15: pinOut = PIN_RELAY_SBH_GL2_NACH6; break;
        case 16: pinOut = PIN_RELAY_SBH_GL3_NACH6; break;
        case 17: pinOut = PIN_RELAY_BLOCK6_NACH4; break;

        case 18: pinOut = PIN_RELAY_NOTHALT; break;
        case 19: pinOut = PIN_RELAY_TRAFO_OBEN_CUT; break;
        case 20: pinOut = PIN_RELAY_TRAFO_UNTEN_CUT; break;

        default: return false;
    }
    return true;
}

// Pulse state for turnout coils (bits 0..7)
static volatile bool     s_diagPulseReq = false;
static volatile uint8_t  s_diagPulseBit = 0;
static volatile uint16_t s_diagPulseMs  = 0;

static bool     s_diagPulseActive = false;
static uint8_t  s_diagPulsePin    = 0;
static uint32_t s_diagPulseEndMs  = 0;


// Selftest-Retry darf NICHT im I2C-Callback gestartet werden (kann onRequest verhungern lassen)
static volatile bool s_pendingSelftestRetry = false;
static volatile bool s_pendingSelftestStartup = false;

static inline void drdySetLow()
{
    if (!s_drdyActiveLow)
    {
        digitalWrite(PIN_DATA_READY_M2, LOW);
        s_drdyActiveLow = true;
    }
}

static inline void drdySetHigh()
{
    if (s_drdyActiveLow)
    {
        digitalWrite(PIN_DATA_READY_M2, HIGH);
        s_drdyActiveLow = false;
    }
}


// ------------------------------------------------------------
// Pending-mask helpers (digital-only; DRDY stays LOW while mask!=0)
// ------------------------------------------------------------
static inline void pendingSet(uint16_t bits)
{
    if (bits == 0) return;
    s_pendingMask |= bits;
    drdySetLow();
}

#if MEGA2_DEBUG
static inline uint8_t diffCountBytes(const void* a, const void* b, size_t n)
{
    const uint8_t* pa = (const uint8_t*)a;
    const uint8_t* pb = (const uint8_t*)b;
    uint8_t c = 0;
    for (size_t i=0;i<n;i++) if (pa[i] != pb[i]) c++;
    return c;
}

static inline void dbgPrintPendBits(uint16_t bits)
{
    DBG_PRINTF("[PEND] bits=0x%04X ", bits);
    DBG_PRINT(F(" ["));
    bool first=true;
    auto add=[&](const __FlashStringHelper* s){
        if (!first) DBG_PRINT(F("|"));
        DBG_PRINT(s); first=false;
    };
    if (bits & M2_PEND_SAFETY)     add(F("SAFETY"));
    if (bits & M2_PEND_ENTRY)      add(F("ENTRY"));
    if (bits & M2_PEND_ENTRY_PREV) add(F("ENTRY_PREV"));
    if (bits & M2_PEND_BLOCKS)     add(F("BLOCKS"));
    if (bits & M2_PEND_SHADOW)     add(F("SHADOW"));
    if (bits & M2_PEND_TURNOUTS)   add(F("TURNOUTS"));
    if (bits & M2_PEND_DIAG_SENSORS) add(F("DIAG_SENS"));
    if (first) DBG_PRINT(F("none"));
    DBG_PRINTLN(F("]"));
}
#endif

static inline void pendingClear(uint16_t bits)
{
    if (bits == 0) return;
    s_pendingMask &= (uint16_t)~bits;
    if (s_pendingMask == 0)
        drdySetHigh();
}

// Optional public hook for future use (keeps changes local to this module)
void megaI2C_setPending(uint16_t bits)
{
    pendingSet(bits);
}

bool megaI2C_diagIsActive()
{
    const uint32_t now = (uint32_t)millis();
    const uint32_t last = (s_lastDiagRelaysReadMs > s_lastDiagReadMs) ? s_lastDiagRelaysReadMs : s_lastDiagReadMs;
    return (uint32_t)(now - last) < 2000u;
}

// Backward compatible helper: mark all digital payloads as pending
void megaI2C_markDataReady()
{
    pendingSet(M2_PEND_ALL_DIGITAL);
}

// ------------------------------------------------------------
// I2C Receive (Master → Slave)
// ------------------------------------------------------------
void i2cOnReceive(int len)
{
    if (len <= 0) return;

    const uint8_t cmd = Wire.read();

    // --------------------------------------------------
    // SAFETY: Notaus setzen/löschen
    // Payload: [0/1]
    // --------------------------------------------------
    if (cmd == M2_CMD_SET_NOTAUS)
    {
        if (len < 1 + 1)
        {
            s_cmdResponseOk      = 0;
            s_cmdResponsePending = true;
            return;
        }

        const uint8_t on = Wire.read();
        safetySetEmergency(on != 0);

        // Digital state changed -> mark safety pending (DRDY active LOW)
        pendingSet(M2_PEND_SAFETY);

        s_cmdResponseOk      = 1;
        s_cmdResponsePending = true;
        return;
    }

    // --------------------------------------------------
    // SAFETY: Notaus quittieren (ACK)
    // --------------------------------------------------
    if (cmd == M2_CMD_ACK_ERROR)
    {
        // optional: mask wird aktuell ignoriert
        if (len >= 1 + 1) (void)Wire.read();

        bool ok = safetyResetEmergency();

        // Digital state may change -> mark safety pending (DRDY active LOW)
        pendingSet(M2_PEND_SAFETY);

        s_cmdResponseOk      = ok ? 1 : 0;
        s_cmdResponsePending = true;
        return;
    }

    // --------------------------------------------------
    // SBHF: Selftest retry (UI-triggered)
    // --------------------------------------------------
    if (cmd == M2_CMD_SBH_SELFTEST_RETRY)
    {
        // WICHTIG: NICHT hier startSelftest() aufrufen (I2C onReceive ist timingkritisch).
        // Wir quittieren sofort und starten den Selftest später im loop-Kontext.
        s_pendingSelftestRetry = true;
        
        // Digital state will change soon -> mark relevant payloads pending (DRDY active LOW)
        pendingSet(M2_PEND_SHADOW | M2_PEND_ENTRY | M2_PEND_ENTRY_PREV | M2_PEND_SAFETY);

        s_cmdResponseOk      = 1;
        s_cmdResponsePending = true;
        return;
    }
    
    // --------------------------------------------------
    // SBHF: Selftest startup (Startup-Checklist)
    // --------------------------------------------------
    if (cmd == M2_CMD_SBH_SELFTEST_STARTUP)
    {
        // NICHT im onReceive starten (timingkritisch) -> später im loop
        s_pendingSelftestStartup = true;
        pendingSet(M2_PEND_SHADOW | M2_PEND_ENTRY | M2_PEND_ENTRY_PREV | M2_PEND_SAFETY);
        s_cmdResponseOk      = 1;
        s_cmdResponsePending = true;
        return;
    }

    // --------------------------------------------------
    // MODE: Automatik <-> DIAG/TEST
    // Payload: [mode]
    //  mode: 0=AUTOMATIK, 1=DIAG_TEST
    // --------------------------------------------------
    if (cmd == M2_CMD_SET_RUNMODE)
    {
        if (len < 1 + 1)
        {
            s_cmdResponseOk      = 0;
            s_cmdResponsePending = true;
            return;
        }

        const uint8_t mode = Wire.read();

        bool ok = false;
        if (mode == 0)
        {
            mega2SetRunMode(Mega2RunMode::Automatik);
            ok = true;
        }
        else if (mode == 1)
        {
            mega2SetRunMode(Mega2RunMode::DiagTest);
            ok = true;
        }

        // Mode affects runtime behavior; nudge master to refresh digital payloads.
        if (ok)
            pendingSet(M2_PEND_ALL_DIGITAL | M2_PEND_TURNOUTS);

        s_cmdResponseOk      = ok ? 1 : 0;
        s_cmdResponsePending = true;
        return;
    }

    // --------------------------------------------------
    // SAFETY: SSR explizit schalten
    // Payload: [ssrIndex, enable]
    // --------------------------------------------------
    if (cmd == M2_CMD_SET_SSR)
    {
        // Erwartet genau 2 Bytes Payload
        if (len < 1 + 2)
        {
            s_cmdResponseOk      = 0;
            s_cmdResponsePending = true;
            return;
        }

        const uint8_t ssrIndex = Wire.read();
        const uint8_t enable   = Wire.read();

        bool ok = false;
        const bool en = (enable != 0);

        // Niemals einschalten, wenn Notaus aktiv oder Lock aktiv.
        // Ausschalten ist immer erlaubt.
        const bool allowEnable = (!en) || (!safetyIsEmergencyActive() && !safetyIsLocked());

        if (allowEnable)
        {
            if (ssrIndex == SSR_MAIN_ENABLE)
            {
                if (en) ok = safetyPowerOn();
                else { safetySetSSR(SSR_MAIN_ENABLE, false); ok = true; }
            }
            else if (ssrIndex == SSR_TRAFO_A)
            {
                safetySetSSR(SSR_TRAFO_A, en);
                ok = true;
            }
            else if (ssrIndex == SSR_TRAFO_B)
            {
                safetySetSSR(SSR_TRAFO_B, en);
                ok = true;
            }
        }


        s_cmdResponseOk      = ok ? 1 : 0;
        s_cmdResponsePending = true;
        return;
    }

    // --------------------------------------------------
    // SAFETY: Power-On (explizit)
    // --------------------------------------------------
    if (cmd == M2_CMD_POWER_ON)
    {
        const bool ok = safetyPowerOn();
        
        // Digital state may change -> mark safety pending (DRDY active LOW)
        pendingSet(M2_PEND_SAFETY);

        s_cmdResponseOk      = ok ? 1 : 0;
        s_cmdResponsePending = true;
        return;
    }


    // --------------------------------------------------
    // DIAG: Relay Set (pin-level, active-low)
    // Payload: Mega2DiagRelaySetPayload { bit, on }
    // Rules:
    //  - only in DIAG_TEST
    //  - enable (on=1 => drive LOW) blocked when Safety locked/emergency
    //  - disable always allowed
    // --------------------------------------------------
    if (cmd == CMD_SET_M2_DIAG_RELAY)
    {
        if (len < 1 + (int)sizeof(Mega2DiagRelaySetPayload))
        {
            s_cmdResponseOk      = 0;
            s_cmdResponsePending = true;
            return;
        }

        Mega2DiagRelaySetPayload pl{};
        pl.bit = Wire.read();
        pl.on  = Wire.read();

        uint8_t pin = 0;
        const bool on = (pl.on != 0);
        const bool mapOk = diagRelayBitToPin(pl.bit, pin);

        EE_DIAGW("rx SET_DIAG_RELAY bit=%u on=%u mapOk=%u pin=%u diagTest=%u safetyLock=%u emg=%u",
                (unsigned)pl.bit, (unsigned)(on?1:0),
                (unsigned)(mapOk?1:0), (unsigned)pin,
                (unsigned)(mega2IsDiagTest()?1:0),
                (unsigned)(safetyIsLocked()?1:0),
                (unsigned)(safetyIsEmergencyActive()?1:0));


        bool ok = false;
        if (mapOk && mega2IsDiagTest())
        {
            // DIAG policy: In DIAG_TEST we intentionally ignore SafetyLock (operator is in manual test mode).
            // Emergency (Not-Aus) stays hard blocking.
            const bool allowEnable = (!on) || (!safetyIsEmergencyActive());
            if (allowEnable)
            {
                EE_DIAGW("SET exec bit=%u on=%u -> pin=%u", (unsigned)pl.bit, (unsigned)(on?1:0), (unsigned)pin);
                digitalWrite(pin, on ? LOW : HIGH);
                ok = true;
                pendingSet(M2_PEND_DIAG_RELAYS);
            }
            else {
                EE_DIAGW("gate DENY SET_DIAG_RELAY bit=%u on=%u (emg)", (unsigned)pl.bit, (unsigned)(on?1:0));
            }
        }
        else {
            EE_DIAGW("gate DENY SET_DIAG_RELAY bit=%u (mapOk=%u diagTest=%u)", (unsigned)pl.bit,
                    (unsigned)(mapOk?1:0), (unsigned)(mega2IsDiagTest()?1:0));
        }

        EE_DIAGW("tx SET_DIAG_RELAY -> %s", ok?"OK":"FAIL");

        s_cmdResponseOk      = ok ? 1 : 0;
        s_cmdResponsePending = true;
        return;
    }

    // --------------------------------------------------
    // DIAG: Relay Pulse (turnout coils only, bits 0..7)
    // Payload: Mega2DiagRelayPulsePayload { bit, ms }
    // - pulse is executed in megaI2C_update() (not inside onReceive)
    // --------------------------------------------------
    if (cmd == CMD_PULSE_M2_DIAG_RELAY)
    {
        if (len < 1 + (int)sizeof(Mega2DiagRelayPulsePayload))
        {
            s_cmdResponseOk      = M2_DIAG_RESP_FAIL;
            s_cmdResponsePending = true;
            return;
        }

        Mega2DiagRelayPulsePayload pl{};
        pl.bit = Wire.read();
        const uint8_t lo = Wire.read();
        const uint8_t hi = Wire.read();
        pl.ms = (uint16_t)((uint16_t)lo | ((uint16_t)hi << 8));
        
        EE_DIAGW("rx PULSE_DIAG_RELAY bit=%u ms=%u diagTest=%u safetyLock=%u emg=%u",
                (unsigned)pl.bit, (unsigned)pl.ms,
                (unsigned)(mega2IsDiagTest()?1:0),
                (unsigned)(safetyIsLocked()?1:0),
                (unsigned)(safetyIsEmergencyActive()?1:0));

        uint8_t resp = M2_DIAG_RESP_FAIL;
        if (pl.bit <= 7 && mega2IsDiagTest())
        {
            uint16_t ms = pl.ms;
            if (ms < 50)   ms = 50;
            if (ms > 2000) ms = 2000;

            // DIAG policy: ignore SafetyLock in DIAG_TEST; Emergency remains hard blocking.
            const bool allow = (!safetyIsEmergencyActive());
            if (allow)
            {
                if (s_diagPulseActive)
                {
                    // Mega1-style: reject new pulse while one is active -> BUSY
                    EE_DIAGW("gate BUSY PULSE_DIAG_RELAY bit=%u (active)", (unsigned)pl.bit);
                    resp = M2_DIAG_RESP_BUSY;
                }
                else
                {
                    s_diagPulseBit = pl.bit;
                    s_diagPulseMs  = ms;
                    s_diagPulseReq = true;
                    resp = M2_DIAG_RESP_OK;
                }
            }
            else {
                EE_DIAGW("gate DENY PULSE_DIAG_RELAY bit=%u (emg)", (unsigned)pl.bit);
                resp = M2_DIAG_RESP_FAIL;
            }
        }
        else {
            EE_DIAGW("gate DENY PULSE_DIAG_RELAY bit=%u (diagTest=%u)", (unsigned)pl.bit, (unsigned)(mega2IsDiagTest()?1:0));
            resp = M2_DIAG_RESP_FAIL;
        }

        EE_DIAGW("tx PULSE_DIAG_RELAY -> %s resp=0x%02X",
                 (resp == M2_DIAG_RESP_OK) ? "OK" : "FAIL",
                 (unsigned)resp);

        s_cmdResponseOk      = resp;
        s_cmdResponsePending = true;
        return;
    }

    // --------------------------------------------------
    // Bestehende GET-Kommandos
    // --------------------------------------------------
    s_pendingResponse = cmd;
    
    // Keine DRDY-Aktion hier: GET ist nur "Abholen".
}

// ------------------------------------------------------------
// I2C Request (Slave → Master)
// ------------------------------------------------------------
void i2cOnRequest()
{
    // --------------------------------------------------
    // Priorität: Antwort auf Command (1 Byte OK/FAIL)
    // --------------------------------------------------
    if (s_cmdResponsePending)
    {
        Wire.write(&s_cmdResponseOk, 1);
        s_cmdResponsePending = false;

        // A command was just processed; the master has "seen" us.
        // We clear DRDY here to avoid a stuck-low line on pure command/ACK flows.
        // (Master will also do digital GETs shortly after.)
        if (s_pendingMask == 0) drdySetHigh();
        
        return;
    }

    // --------------------------------------------------
    // Default: SystemStatus (read-only)
    // IMPORTANT:
    // Do NOT rely on a periodically refreshed global struct here.
    // If the main loop fails to update g_systemStatus (or it is still zeroed
    // during boot), ESP will see ver/size/node as 0 and mark Mega2 offline.
    // Build the status on-demand to guarantee a valid v3/26B header.
    // --------------------------------------------------
    if (s_pendingResponse == 0)
    {
        SystemStatus st{};
        buildMega2SystemStatus(st);
        Wire.write(reinterpret_cast<uint8_t*>(&st), sizeof(st));

        // SystemStatus is a "digital snapshot" -> clear all pending digital bits after serving it
        pendingClear(M2_PEND_ALL_DIGITAL);
        
        return;
    }

    // --------------------------------------------------
    // GET-Kommandos
    // --------------------------------------------------
    switch (s_pendingResponse)
    {
        case CMD_GET_M2_SAFETY:
        {
            Mega2SafetyStatus st{};
            buildMega2SafetyStatus(st);
            Wire.write(reinterpret_cast<uint8_t*>(&st), sizeof(st));
            pendingClear(M2_PEND_SAFETY);
            break;
        }

        case CMD_GET_M2_BLOCKS:
        {
            BlockStatus blocks[M2_NUM_BLOCKS]{};
            buildMega2BlockStatus(blocks, blockController);
            Wire.write(reinterpret_cast<uint8_t*>(blocks), sizeof(blocks));
            pendingClear(M2_PEND_BLOCKS);
            break;
        }

        case CMD_GET_M2_TURNOUTS:
        {
            Mega2TurnoutsPayload t{};
            t.sollMask = g_systemStatus.turnoutSollMask;
            t.istMask  = g_systemStatus.turnoutIstMask;
            Wire.write(reinterpret_cast<uint8_t*>(&t), sizeof(t));
            pendingClear(M2_PEND_TURNOUTS);
            break;
        }

        case CMD_GET_M2_SBH:
        {
            ShadowYardStatus st{};
            buildMega2ShadowStatus(st, shadowController);
            Wire.write(reinterpret_cast<uint8_t*>(&st), sizeof(st));
            pendingClear(M2_PEND_SHADOW);   
            break;
        }
         
         case CMD_GET_M2_ANALOG:
         {
             Mega2AnalogPayload p{};
             p.seq = ++s_analogSeq;

             // Raw/Debug mode: keep payload wire-safe, but reinterpret fields.
             // flags bit4 (0x10): raw counts
             //   vA10/vB10 carry raw ADC counts (0..1023) from Trafo sensors (A9/A10)
             //   i_mA[]    carries RMS counts from current sensors (ZMCT103C front-end)
             p.flags = 0x10;

             // Trafo sensors: send instantaneous ADC counts for now (calibration later)
             p.vA10 = (uint16_t)analogRead(PIN_ADC_TRAFO_OBEN);   // A9
             p.vB10 = (uint16_t)analogRead(PIN_ADC_TRAFO_UNTEN);  // A10
 
             
             for (uint8_t i = 0; i < M2_NUM_BLOCKS; i++)
             {
                 // Blocks are 1-based (g_blocks[0] = nullptr)
                 Block* b = g_blocks[i + 1];
                 const uint16_t rmsCounts = b ? b->stromRmsCounts() : 0;
                 p.i_mA[i] = rmsCounts;
             }
 
             Wire.write(reinterpret_cast<uint8_t*>(&p), sizeof(p));

            // IMPORTANT: analog does NOT clear DRDY (digital-only signal)
             
             break;
         }
         case CMD_GET_M2_PENDING_MASK:
        {
            Mega2PendingMaskPayload p{};
            p.seq  = ++s_pendingSeq;
            p.rsv0 = 0;
            p.mask = (uint16_t)s_pendingMask;
            Wire.write(reinterpret_cast<uint8_t*>(&p), sizeof(p));

            // IMPORTANT: pending-mask read does NOT clear DRDY / pending bits.
            break;
        }

        case CMD_GET_M2_DIAG_SENSORS:
        {
            // Mark diag as active (so we start latching DRDY for diag-only changes)
            s_lastDiagReadMs = (uint32_t)millis();

            Wire.write(reinterpret_cast<uint8_t*>(&s_diagSnap), sizeof(s_diagSnap));

            // Clear pending bit after serving the snapshot.
            // IMPORTANT: we do NOT clear edge counters here (cumulative, like Mega1).
            pendingClear(M2_PEND_DIAG_SENSORS);
            break;
        }

        case CMD_GET_M2_DIAG_RELAYS:
        {
            s_lastDiagRelaysReadMs = (uint32_t)millis();
            // Always build a fresh snapshot to avoid stale/jittery UI updates.
            // This makes relay telemetry deterministic and "immediate" after writes.
            if (s_diagRelaysSnap.seq == 0) s_diagRelaysSnap.seq = 1;
            else s_diagRelaysSnap.seq++;
            s_diagRelaysSnap.levelMask = buildDiagRelaysMaskActiveLow();
            Wire.write(reinterpret_cast<uint8_t*>(&s_diagRelaysSnap), sizeof(s_diagRelaysSnap));
            pendingClear(M2_PEND_DIAG_RELAYS);
            break;
        }

        
        case CMD_GET_M2_ENTRY:
        {

            uint16_t entry[M2_NUM_BLOCKS]{};

            buildEntryMatrix(entry);
            Wire.write(reinterpret_cast<uint8_t*>(entry), sizeof(entry));
            pendingClear(M2_PEND_ENTRY);
            break;
        }

        case CMD_GET_M2_ENTRY_PREVIEW:
        {

            uint16_t entry[M2_NUM_BLOCKS]{};


            buildEntryPreviewMatrix(entry);
            Wire.write(reinterpret_cast<uint8_t*>(entry), sizeof(entry));
            pendingClear(M2_PEND_ENTRY_PREV);
            break;
        }

        default:
            // unbekannt → als Fallback SystemStatus senden (verhindert 0-Reads)
            {
                SystemStatus st{};
                buildMega2SystemStatus(st);
                Wire.write(reinterpret_cast<uint8_t*>(&st), sizeof(st));
                pendingClear(M2_PEND_ALL_DIGITAL);
            }
            break;
    }

    s_pendingResponse = 0;
}

// ------------------------------------------------------------
// I2C Initialisierung (Slave)
// ------------------------------------------------------------
void megaI2C_begin()
{
    Wire.begin(0x11);   // Mega2-Adresse
    Wire.onReceive(i2cOnReceive);
    Wire.onRequest(i2cOnRequest);

    // One-shot boot log: helps field-debug (no ISR logs)
    Serial.print(F("[M2I2C] ready addr=0x11 bootId="));
    Serial.println(g_bootId);

    // DRDY pin init: idle HIGH (not ready), active LOW (data ready)
    pinMode(PIN_DATA_READY_M2, OUTPUT);
    digitalWrite(PIN_DATA_READY_M2, HIGH);
    s_drdyActiveLow = false;
}

// ------------------------------------------------------------
// Zyklisches Update (derzeit leer)
// ------------------------------------------------------------
void megaI2C_update()
{
    // ------------------------------------------------------------
    // DIAG relay pulse sequencer (turnout coils W12..W15)
    // - executed in loop context (not in i2cOnReceive)
    // - active-low: pulse drives pin LOW, then releases to HIGH
    // ------------------------------------------------------------
    {
        const uint32_t nowMs = (uint32_t)millis();

        // Finish active pulse
        if (s_diagPulseActive && (int32_t)(nowMs - s_diagPulseEndMs) >= 0)
        {
            digitalWrite(s_diagPulsePin, HIGH);
            s_diagPulseActive = false;
            EE_DIAGW("pulse done bit=%u", (unsigned)s_diagPulseBit);
            pendingSet(M2_PEND_DIAG_RELAYS);
        }

        // Start new pulse request (one at a time)
        if (!s_diagPulseActive && s_diagPulseReq)
        {
            s_diagPulseReq = false;

            uint8_t pin = 0;
            if (diagRelayBitToPin(s_diagPulseBit, pin))
            {
                s_diagPulsePin    = pin;
                s_diagPulseEndMs  = nowMs + (uint32_t)s_diagPulseMs;
                s_diagPulseActive = true;
                EE_DIAGW("pulse start bit=%u ms=%u", (unsigned)s_diagPulseBit, (unsigned)s_diagPulseMs);
                digitalWrite(pin, LOW);
                pendingSet(M2_PEND_DIAG_RELAYS);
            }
        }
    }

    // Selftest-Retry aus UI asynchron starten
    if (s_pendingSelftestRetry)
    {
        s_pendingSelftestRetry = false;

        // UI Selftest-Retry (SBHF-Weichenfehler):
        // Muss auch unter Safety-Lock/NOTAUS starten dürfen, sonst Deadlock:
        // ACK blockt -> Selftest nötig, aber Selftest wäre sonst ebenfalls geblockt.
        const bool started = shadowController.startSelftestRetry(true);
        if (started)
        {
            DBG_PRINTLN(F("[I2C] SBHF selftest retry started"));
            safetyNotifySbhfSelftestStarted();
        }
        else         DBG_PRINTF("[I2C] SBHF selftest retry rejected: state=%u selftestActive=%u lock=%u notaus=%u warn=0x%02X allow=0x%02X\n",
                                (unsigned)shadowController.state(),
                                (unsigned)shadowController.isSelftestActive(),
                                (unsigned)safetyIsLocked(),
                                (unsigned)safetyIsEmergencyActive(),
                                (unsigned)shadowController.warningMask(),
                                (unsigned)shadowController.allowedGleisMask());
    }

    if (s_pendingSelftestStartup)
    {
        s_pendingSelftestStartup = false;

        // Startup-Checklist Selftest: darf auch bei lock=1 starten, aber NICHT bei aktivem HW-Notaus
        const bool started = shadowController.startSelftestStartup(true);
        if (started)
        {
            DBG_PRINTLN(F("[I2C] SBHF selftest startup started"));
            safetyNotifySbhfSelftestStarted();
        }
        else
            DBG_PRINTF("[I2C] SBHF selftest startup rejected: state=%u selftestActive=%u lock=%u notaus=%u warn=0x%02X allow=0x%02X\n",
                       (unsigned)shadowController.state(),
                       (unsigned)shadowController.isSelftestActive(),
                       (unsigned)safetyIsLocked(),
                       (unsigned)safetyIsEmergencyActive(),
                       (unsigned)shadowController.warningMask(),
                       (unsigned)shadowController.allowedGleisMask());
    }

    
    // ------------------------------------------------------------
    // DRDY change scan (digital payloads): if anything changed -> DRDY LOW
    // This makes updates event-driven without touching controller internals.
    // ------------------------------------------------------------
    static uint32_t s_scanMs = 0;
    const uint32_t now = millis();
    if ((uint32_t)(now - s_scanMs) < 50) return; // 20 Hz is enough
    s_scanMs = now;

    static bool s_hasLast = false;
    static Mega2SafetyStatus s_lastSafety{};
    static BlockStatus       s_lastBlocks[M2_NUM_BLOCKS]{};
    static uint8_t           s_lastBlockFlags[M2_NUM_BLOCKS]{}; // digital-only flags (no stromRaw noise)
    static ShadowYardStatus  s_lastSbh{};
    static uint16_t          s_lastEntry[M2_NUM_BLOCKS]{};
    static uint16_t          s_lastPreview[M2_NUM_BLOCKS]{};
    static uint16_t          s_lastOccMask = 0; // stable occupiedMask (digital)
    static uint16_t          s_lastTurnoutSoll = 0;
    static uint16_t          s_lastTurnoutIst  = 0;

    Mega2SafetyStatus curSafety{};
    BlockStatus       curBlocks[M2_NUM_BLOCKS]{};
    uint8_t           curBlockFlags[M2_NUM_BLOCKS]{}; // digital-only flags (no stromRaw noise)
    ShadowYardStatus  curSbh{};
    uint16_t          curEntry[M2_NUM_BLOCKS]{};
    uint16_t          curPreview[M2_NUM_BLOCKS]{};
    const uint16_t curOccMask      = g_systemStatus.blockOccupiedMask; // stable occupied mask
    const uint16_t curTurnoutSoll  = g_systemStatus.turnoutSollMask;
    const uint16_t curTurnoutIst   = g_systemStatus.turnoutIstMask;
    
    // ------------------------------------------------------------
    // Diag sensor snapshot (kontakt + schaltgleise)
    // - Snapshot update BEFORE pendingSet (no stale snapshot / "1 frame late")
    // - Only latch DRDY for diag when a diag client is active (recent reads)
    // ------------------------------------------------------------
    auto readKontakt = [&](uint8_t idx) -> bool {
        // fixed order, idx 0..13
        switch (idx) {
            case 0:  return k_block1.raw();
            case 1:  return k_block2.raw();
            case 2:  return k_block3.raw();
            case 3:  return k_block4.raw();
            case 4:  return k_block5.raw();
            case 5:  return k_block6.raw();
            case 6:  return k_sbhf1.raw();
            case 7:  return k_sbhf2.raw();
            case 8:  return k_sbhf3.raw();
            case 9:  return k_nothalt.raw();
            case 10: return k_bhf2a.raw();
            case 11: return k_bhf2b.raw();
            case 12: return k_bhf4a.raw();
            case 13: return k_bhf4b.raw();
            default: return false;
        }
    };

    uint16_t kontaktLevel = 0;
    for (uint8_t i = 0; i < M2_DIAG_NUM_KONTAKTE; ++i) {
        if (readKontakt(i)) kontaktLevel |= (1u << i);
    }

    const uint8_t schaltLevel =
        (g_s11.levelActive() ? (1u << 0) : 0) |
        (g_s12.levelActive() ? (1u << 1) : 0) |
        (g_s13.levelActive() ? (1u << 2) : 0) |
        (g_s14.levelActive() ? (1u << 3) : 0) |
        (g_s15.levelActive() ? (1u << 4) : 0) |
        (g_s16.levelActive() ? (1u << 5) : 0);

    auto saneU8 = [](uint8_t v) -> uint8_t {
        // 0xFF == invalid/uninitialized -> treat as 0 for diagnostics
        return (v == 0xFF) ? 0 : v;
    };
    
    const uint8_t schaltRise[M2_DIAG_NUM_SCHALT] = {
        saneU8(g_s11.riseCount()), saneU8(g_s12.riseCount()), saneU8(g_s13.riseCount()),
        saneU8(g_s14.riseCount()), saneU8(g_s15.riseCount()), saneU8(g_s16.riseCount())
    };
    const uint8_t schaltFall[M2_DIAG_NUM_SCHALT] = {
        saneU8(g_s11.fallCount()), saneU8(g_s12.fallCount()), saneU8(g_s13.fallCount()),
        saneU8(g_s14.fallCount()), saneU8(g_s15.fallCount()), saneU8(g_s16.fallCount())
    };

    bool diagChanged = false;

    if (!s_diagHasLast) {
        s_diagHasLast = true;
        s_diagSnap.seq = 1;
        s_diagSnap.kontaktLevelMask = kontaktLevel;
        memset(s_diagSnap.kontaktRise4, 0, sizeof(s_diagSnap.kontaktRise4));
        memset(s_diagSnap.kontaktFall4, 0, sizeof(s_diagSnap.kontaktFall4));
        s_diagSnap.schaltLevelMask  = schaltLevel;
        memcpy(s_diagSnap.schaltRise, schaltRise, sizeof(s_diagSnap.schaltRise));
        memcpy(s_diagSnap.schaltFall, schaltFall, sizeof(s_diagSnap.schaltFall));
        s_diagLastKontaktLevel = kontaktLevel;
        s_diagLastSchaltLevel  = schaltLevel;
        memcpy(s_diagLastSchaltRise, schaltRise, sizeof(s_diagLastSchaltRise));
        memcpy(s_diagLastSchaltFall, schaltFall, sizeof(s_diagLastSchaltFall));
    } else {
        const uint16_t diff = (uint16_t)(kontaktLevel ^ s_diagLastKontaktLevel);
        if (diff) {
            // Per-contact 4-bit edge counters (cumulative, wrap 0..15).
            // NOTE: We keep the last level separately to detect edges reliably.
                for (uint8_t i = 0; i < M2_DIAG_NUM_KONTAKTE; ++i) {
                    const uint16_t bit = (uint16_t)(1u << i);
                    if (diff & bit) {
                        if (kontaktLevel & bit) incNibble(s_diagSnap.kontaktRise4, i);
                        else                    incNibble(s_diagSnap.kontaktFall4, i);
                    }
                }
            s_diagSnap.kontaktLevelMask = kontaktLevel;
            s_diagLastKontaktLevel = kontaktLevel;
            diagChanged = true;
        }

        if (schaltLevel != s_diagLastSchaltLevel) {
            s_diagSnap.schaltLevelMask = schaltLevel;
            s_diagLastSchaltLevel = schaltLevel;
            diagChanged = true;
        }
        for (uint8_t i = 0; i < M2_DIAG_NUM_SCHALT; ++i) {
            if (schaltRise[i] != s_diagLastSchaltRise[i]) {
                s_diagLastSchaltRise[i] = schaltRise[i];
                s_diagSnap.schaltRise[i] = schaltRise[i];
                diagChanged = true;
            }
            if (schaltFall[i] != s_diagLastSchaltFall[i]) {
                s_diagLastSchaltFall[i] = schaltFall[i];
                s_diagSnap.schaltFall[i] = schaltFall[i];
                diagChanged = true;
            }
        }

        if (diagChanged) {
            s_diagSnap.seq = (uint8_t)(s_diagSnap.seq + 1);
            if (megaI2C_diagIsActive()) {
                pendingSet(M2_PEND_DIAG_SENSORS);
            }
        }
    }

    // Diag relays snapshot (pin levels, active-low)
    {
        const uint32_t relayMask = buildDiagRelaysMaskActiveLow();
        if (s_diagRelaysSnap.seq == 0)
        {
            s_diagRelaysSnap.seq = 1;
            s_diagRelaysSnap.levelMask = relayMask;
        }
        else if (relayMask != s_diagRelaysSnap.levelMask)
        {
            s_diagRelaysSnap.levelMask = relayMask;
            s_diagRelaysSnap.seq = (uint8_t)(s_diagRelaysSnap.seq + 1);
            if (megaI2C_diagIsActive())
                pendingSet(M2_PEND_DIAG_RELAYS);
        }
    }

    buildMega2SafetyStatus(curSafety);
    buildMega2BlockStatus(curBlocks, blockController);
    for (uint8_t i = 0; i < M2_NUM_BLOCKS; i++)
    {
        // DRDY-digitale Blocks: nur stabile/digitale Flags (kein Analog-Jitter)
        // - stromEin/besetzt hängen typischerweise an stromRaw -> kann rauschen -> DRDY bleibt sonst dauernd LOW
        curBlockFlags[i] = (uint8_t)((curBlocks[i].kontakt     ? 1u  : 0u) |
                                     (curBlocks[i].kurzschluss ? 2u  : 0u) |
                                     (curBlocks[i].nothalt     ? 4u  : 0u));
    }

    buildMega2ShadowStatus(curSbh, shadowController);
    buildEntryMatrix(curEntry);
    buildEntryPreviewMatrix(curPreview);

    if (!s_hasLast)
    {
        s_lastSafety = curSafety;
        memcpy(s_lastBlocks,  curBlocks,  sizeof(curBlocks));
        memcpy(s_lastBlockFlags, curBlockFlags, sizeof(curBlockFlags));
        s_lastSbh = curSbh;
        memcpy(s_lastEntry,   curEntry,   sizeof(curEntry));
        memcpy(s_lastPreview, curPreview, sizeof(curPreview));
        s_lastOccMask = curOccMask;
        s_lastTurnoutSoll = curTurnoutSoll;
        s_lastTurnoutIst  = curTurnoutIst;
        s_hasLast = true;
        return;
    }

    const uint16_t bits =
        ((memcmp(&s_lastSafety, &curSafety, sizeof(curSafety)) != 0) ? M2_PEND_SAFETY     : 0) |
        ((memcmp( s_lastBlockFlags, curBlockFlags, sizeof(curBlockFlags)) != 0) ? M2_PEND_BLOCKS : 0) |
        ((s_lastOccMask != curOccMask) ? M2_PEND_BLOCKS : 0) |
        ((memcmp(&s_lastSbh,    &curSbh,    sizeof(curSbh))    != 0) ? M2_PEND_SHADOW     : 0) |
        ((memcmp( s_lastEntry,   curEntry,  sizeof(curEntry))  != 0) ? M2_PEND_ENTRY      : 0) |
        ((memcmp( s_lastPreview, curPreview,sizeof(curPreview))!= 0) ? M2_PEND_ENTRY_PREV : 0) |
        (((s_lastTurnoutSoll != curTurnoutSoll) || (s_lastTurnoutIst != curTurnoutIst)) ? M2_PEND_TURNOUTS : 0);

    if (bits)
    {

#if MEGA2_DEBUG
        dbgPrintPendBits(bits);
        if (bits & M2_PEND_BLOCKS)
        {
            DBG_PRINTF("[PEND] occMask %04X -> %04X\n", s_lastOccMask, curOccMask);
        }
        if (bits & (M2_PEND_ENTRY|M2_PEND_ENTRY_PREV))
        {
            const uint8_t dE = diffCountBytes(s_lastEntry,   curEntry,   sizeof(curEntry));
            const uint8_t dP = diffCountBytes(s_lastPreview, curPreview, sizeof(curPreview));
            DBG_PRINT(F("[PEND] entryDiff=")); DBG_PRINT(dE);
            DBG_PRINT(F(" prevDiff=")); DBG_PRINTLN(dP);
        }
        if (bits & M2_PEND_TURNOUTS)
        {
            DBG_PRINTF("[PEND] turnoutSoll %04X -> %04X  turnoutIst %04X -> %04X\n",
                       s_lastTurnoutSoll, curTurnoutSoll,
                       s_lastTurnoutIst,  curTurnoutIst);
        }
#endif

        s_lastSafety = curSafety;
        memcpy(s_lastBlocks,  curBlocks,  sizeof(curBlocks));
        memcpy(s_lastBlockFlags, curBlockFlags, sizeof(curBlockFlags)); 
        s_lastSbh = curSbh;
        memcpy(s_lastEntry,   curEntry,   sizeof(curEntry));
        memcpy(s_lastPreview, curPreview, sizeof(curPreview));
        s_lastOccMask = curOccMask;
        s_lastTurnoutSoll = curTurnoutSoll;
        s_lastTurnoutIst  = curTurnoutIst;

        pendingSet(bits);
    }
}



