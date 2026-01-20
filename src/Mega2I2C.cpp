#include <Wire.h>

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

static bool    s_cmdResponsePending = false;
static uint8_t s_cmdResponseOk      = 0;
static uint8_t s_pendingResponse    = 0;
static uint8_t s_analogSeq          = 0;

// Selftest-Retry darf NICHT im I2C-Callback gestartet werden (kann onRequest verhungern lassen)
static volatile bool s_pendingSelftestRetry = false;

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
        s_cmdResponseOk      = ok ? 1 : 0;
        s_cmdResponsePending = true;
        return;
    }

    // --------------------------------------------------
    // Bestehende GET-Kommandos
    // --------------------------------------------------
    s_pendingResponse = cmd;
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
            break;
        }

        case CMD_GET_M2_BLOCKS:
        {
            BlockStatus blocks[M2_NUM_BLOCKS]{};
            buildMega2BlockStatus(blocks, blockController);
            Wire.write(reinterpret_cast<uint8_t*>(blocks), sizeof(blocks));
            break;
        }

        case CMD_GET_M2_SBH:
        {
            ShadowYardStatus st{};
            buildMega2ShadowStatus(st, shadowController);
            Wire.write(reinterpret_cast<uint8_t*>(&st), sizeof(st));
            break;
        }
         
         case CMD_GET_M2_ANALOG:
         {
             Mega2AnalogPayload p{};
             p.seq   = ++s_analogSeq;
             p.flags = 0;
 
             // TODO: Trafo RMS values are not wired in the uploaded files.
             // Set to 0 for now; patch once we know the authoritative source.
             p.vA10 = 0;
             p.vB10 = 0;
 
             // Currents:
             // We currently don't have an mA conversion source in the provided files.
             // As a safe compile-time placeholder, we reuse BlockStatus.stromRaw (ADC).
             // This gives you "changing numbers" in the UI immediately; calibration can follow.
             BlockStatus blocks[M2_NUM_BLOCKS]{};
             buildMega2BlockStatus(blocks, blockController);
             for (uint8_t i = 0; i < M2_NUM_BLOCKS; i++)
             {
                 p.i_mA[i] = blocks[i].stromRaw; // placeholder
             }
 
             Wire.write(reinterpret_cast<uint8_t*>(&p), sizeof(p));
             break;
         }

        
case CMD_GET_M2_ENTRY:
{
    // Antwort: uint16_t[M2_NUM_BLOCKS] (FROM->TO bitmask)
    // Index: from-1; Bit(to-1)=1 => Einfahrt erlaubt
    uint16_t entry[M2_NUM_BLOCKS]{};
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

    Wire.write(reinterpret_cast<uint8_t*>(entry), sizeof(entry));
    break;
}

case CMD_GET_M2_ENTRY_PREVIEW:
{
    // Antwort: uint16_t[M2_NUM_BLOCKS] (FROM->TO bitmask)
    // Semantik: "prinzipiell möglich" (Preview) – Topologie + Ziel frei + keine globale Safety-Sperre
    uint16_t entry[M2_NUM_BLOCKS]{};

    // Globaler Lock -> alles rot
    if (safetyIsLocked())
    {
        Wire.write(reinterpret_cast<uint8_t*>(entry), sizeof(entry));
        break;
    }

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

    Wire.write(reinterpret_cast<uint8_t*>(entry), sizeof(entry));
    break;
}

        default:
            // unbekannt → als Fallback SystemStatus senden (verhindert 0-Reads)
            {
                SystemStatus st{};
                buildMega2SystemStatus(st);
                Wire.write(reinterpret_cast<uint8_t*>(&st), sizeof(st));
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

        // Startup-Checklist: darf auch bei Boot-ERR starten (SYS_ERROR_PRESENT),
        // aber weiterhin NICHT bei HW-Notaus / Safety-Lock / already running.
        const bool started = shadowController.startSelftestStartup(true);
        if (started) DBG_PRINTLN("[I2C] SBHF selftest retry started");
        else         DBG_PRINTF("[I2C] SBHF selftest retry rejected: state=%u selftestActive=%u lock=%u notaus=%u warn=0x%02X allow=0x%02X\n",
                                (unsigned)shadowController.state(),
                                (unsigned)shadowController.isSelftestActive(),
                                (unsigned)safetyIsLocked(),
                                (unsigned)safetyIsEmergencyActive(),
                                (unsigned)shadowController.warningMask(),
                                (unsigned)shadowController.allowedGleisMask());
    }
}

