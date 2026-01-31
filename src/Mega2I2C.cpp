#include <Wire.h>
#include <Arduino.h>
#include <string.h>

extern uint16_t g_bootId;

#include "BlockController.h"
#include "ShadowYardController.h"

#include "proto_common.h"
#include "proto_mega2.h"

#include "Mega2I2C.h"
#include "Mega2Status.h"
#include "system/status_system.h"
#include "safety.h"
#include "mega2_debug.h"
 
// Analog payload (fixed point): see proto_common.h Mega2AnalogPayload
// ------------------------------------------------------------
// DataReady (Mega2 -> ESP): active LOW, latched until a digital read is served
// ------------------------------------------------------------
constexpr uint8_t PIN_DATA_READY_M2 = 12;

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
             p.seq   = ++s_analogSeq;
             // flags bit1: voltages invalid (solange Trafo-Spannungsmessung noch nicht sauber verdrahtet ist)
             p.flags = 0x02;
 
             // Spannungen bei invalid konsequent auf 0xFFFF setzen (kein Drift-/Floating-Müll im Payload)
             p.vA10 = 0xFFFF;
             p.vB10 = 0xFFFF;
 
             
             for (uint8_t i = 0; i < M2_NUM_BLOCKS; i++)
             {
                 // Authoritative source: gefilterter Strom in mA aus BlockController
                 int32_t mA = (int32_t)blockController.stromFiltered(i);
                 if (mA < 0) mA = 0;
                 if (mA > 5000) mA = 5000; // Plausibilitätsgrenze (UI erwartet typ. <= ~1500)
                 p.i_mA[i] = (uint16_t)mA;
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
            DBG_PRINTLN("[I2C] SBHF selftest retry started");
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
            DBG_PRINTLN("[I2C] SBHF selftest startup started");
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



