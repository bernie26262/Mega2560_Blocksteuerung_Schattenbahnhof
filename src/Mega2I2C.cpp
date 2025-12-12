#include "BlockController.h"
#include "ShadowYardController.h"
#include "proto_common.h"
#include "proto_mega2.h"
#include <Wire.h>
#include "Mega2I2C.h"


// globale Controller aus main.cpp
extern BlockController      blockController;
extern ShadowYardController shadowController;

// ------------------------------------------------------------
// interner Zustand
// ------------------------------------------------------------
static uint8_t s_pendingResponse = 0;

// ------------------------------------------------------------
// I2C Receive
// ------------------------------------------------------------
void i2cOnReceive(int len)
{
    if (len <= 0) return;

    uint8_t cmd = Wire.read();
    s_pendingResponse = cmd;
}

// ------------------------------------------------------------
// I2C Request
// ------------------------------------------------------------
void i2cOnRequest()
{
    switch (s_pendingResponse)
    {
        case CMD_GET_M2_SAFETY:
        {
            Mega2SafetyStatus st{};
            buildMega2SafetyStatus(st);
            Wire.write((uint8_t*)&st, sizeof(st));
            break;
        }

        case CMD_GET_M2_BLOCKS:
        {
            BlockStatus blocks[M2_NUM_BLOCKS]{};
            buildMega2BlockStatus(blocks, blockController);
            Wire.write((uint8_t*)blocks, sizeof(blocks));
            break;
        }

        case CMD_GET_M2_SBH:
        {
            ShadowYardStatus st{};
            buildMega2ShadowStatus(st, shadowController);
            Wire.write((uint8_t*)&st, sizeof(st));
            break;
        }

        default:
            break;
    }

    s_pendingResponse = 0;
}

// --------------------------------------------------
// Initialisierung I2C (Slave)
// --------------------------------------------------
void megaI2C_begin()
{
    // Adresse anpassen, falls nötig
    Wire.begin(0x20);

    Wire.onReceive(i2cOnReceive);
    Wire.onRequest(i2cOnRequest);
}

// --------------------------------------------------
// Zyklisches Update (derzeit leer, bewusst)
// --------------------------------------------------
void megaI2C_update()
{
    // aktuell nichts nötig
    // Platzhalter für spätere Erweiterungen
}
