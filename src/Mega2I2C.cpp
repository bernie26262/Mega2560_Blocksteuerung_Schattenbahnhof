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
// Globale Controller (aus main.cpp)
// ------------------------------------------------------------
extern BlockController& blockController;
extern ShadowYardController& shadowController;

// ------------------------------------------------------------
// Externe Statusdaten (werden in main.cpp gebaut)
// ------------------------------------------------------------
extern SystemStatus g_systemStatus;

// ------------------------------------------------------------
// Interner I2C-Zustand
// ------------------------------------------------------------
static uint8_t s_pendingResponse      = 0;
static bool    s_cmdResponsePending   = false;
static uint8_t s_cmdResponseOk        = 0;

// ------------------------------------------------------------
// I2C Receive (ESP -> Mega2)
// ------------------------------------------------------------
void i2cOnReceive(int len)
{
    if (len <= 0)
        return;

    uint8_t cmd = Wire.read();

    // --------------------------------------------------
    // SAFETY: Fehler / Notaus quittieren
    // --------------------------------------------------
    if (cmd == M2_CMD_ACK_ERROR)
    {
        bool ok = safetyResetEmergency();

        s_cmdResponseOk      = ok ? 1 : 0;
        s_cmdResponsePending = true;
        s_pendingResponse    = 0;   // Sicherheitshalber resetten

        return;
    }

    // --------------------------------------------------
    // Bestehende GET-Kommandos
    // --------------------------------------------------
    s_pendingResponse = cmd;
}

// ------------------------------------------------------------
// I2C Request (Mega2 -> ESP)
// ------------------------------------------------------------
void i2cOnRequest()
{
    // --------------------------------------------------
    // Pending Command Response (ACK / FAIL)
    // --------------------------------------------------
    if (s_cmdResponsePending)
    {
        Wire.write(&s_cmdResponseOk, sizeof(s_cmdResponseOk));
        s_cmdResponsePending = false;
        return;
    }

    // --------------------------------------------------
    // Default: kein Command → SystemStatus (read-only)
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
    // Command-basierte Antworten (bestehend)
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
            // unbekanntes Kommando → nichts senden
            break;
    }

    // Command abgearbeitet
    s_pendingResponse = 0;
}

// ------------------------------------------------------------
// Initialisierung I2C (Slave)
// ------------------------------------------------------------
void megaI2C_begin()
{
    // Mega2 hat die feste Adresse 0x11
    Wire.begin(0x11);

    Wire.onReceive(i2cOnReceive);
    Wire.onRequest(i2cOnRequest);
}

// ------------------------------------------------------------
// Zyklisches Update (derzeit leer, bewusst)
// ------------------------------------------------------------
void megaI2C_update()
{
    // aktuell nichts nötig
    // Platzhalter für spätere Erweiterungen
}
