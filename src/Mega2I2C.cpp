#include <Wire.h>

#include "BlockController.h"
#include "ShadowYardController.h"

#include "proto_common.h"
#include "proto_mega2.h"

#include "Mega2I2C.h"
#include "Mega2Status.h"
#include "system/status_system.h"
#include "safety.h"

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
static bool    s_cmdResponsePending = false;
static uint8_t s_cmdResponseOk      = 0;
static uint8_t s_pendingResponse    = 0;

// ------------------------------------------------------------
// I2C Receive (Master → Slave)
// ------------------------------------------------------------
void i2cOnReceive(int len)
{
    if (len <= 0) return;

    const uint8_t cmd = Wire.read();

    // --------------------------------------------------
    // SAFETY: Notaus quittieren (ACK)
    // --------------------------------------------------
    if (cmd == M2_CMD_ACK_ERROR)
    {
        bool ok = safetyResetEmergency();

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

        // Niemals SSR schalten, wenn Notaus aktiv
        if (!safetyIsEmergencyActive())
        {
            if (ssrIndex == 0)
            {
                safetySetSSR(SafetySSR::SSR_TRAFO_A, enable != 0);
                ok = true;
            }
            else if (ssrIndex == 1)
            {
                safetySetSSR(SafetySSR::SSR_TRAFO_B, enable != 0);
                ok = true;
            }
        }


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
    // --------------------------------------------------
    if (s_pendingResponse == 0)
    {
        Wire.write(
            reinterpret_cast<uint8_t*>(&g_systemStatus),
            sizeof(SystemStatus)
        );
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

        default:
            // unbekannt → nichts senden
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
    // Platzhalter für spätere Erweiterungen
}
