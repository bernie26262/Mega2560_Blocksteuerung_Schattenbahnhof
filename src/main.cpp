#include <Arduino.h>
#include <Wire.h>

#include "mega2_pins.h"

#include "Block.h"
#include "BlockController.h"
#include "Weiche.h"

#include "PulseSensor.h"
#include "SensorKontakt.h"
#include "SensorStrom.h"
#include "SensorTrafoAC.h"

#include "PowerControl.h"
#include "Mega2PowerControl.h"

#include "ShadowYardController.h"
#include "Mega2I2C.h"

#include "proto_mega2.h"
#include "safety.h"
#include "mega2_debug.h"
#include "Mega2Debug.h"

// ============================================================================
// GLOBALE OBJEKTE
// ============================================================================

// --------------------- BLOCKS -----------------------------------------------
Block* g_blocks[16];
BlockController g_bc(g_blocks, 16);
BlockController& blockController = g_bc;

// --------------------- POWER CONTROL ----------------------------------------
Mega2PowerControl g_power;

// --------------------- KONTAKTGLEISE ----------------------------------------
SensorKontakt k_block1(PIN_KONTAKT_BLOCK1);
SensorKontakt k_block2(PIN_KONTAKT_BLOCK2);
SensorKontakt k_block3(PIN_KONTAKT_BLOCK3);
SensorKontakt k_block4(PIN_KONTAKT_BLOCK4);
SensorKontakt k_block5(PIN_KONTAKT_BLOCK5);
SensorKontakt k_block6(PIN_KONTAKT_BLOCK6);

SensorKontakt k_sbhf1(PIN_KONTAKT_SBH_GF1);
SensorKontakt k_sbhf2(PIN_KONTAKT_SBH_GF2);
SensorKontakt k_sbhf3(PIN_KONTAKT_SBH_GF3);

SensorKontakt k_bhf2a(PIN_KONTAKT_BHF2_A);
SensorKontakt k_bhf2b(PIN_KONTAKT_BHF2_B);
SensorKontakt k_bhf4a(PIN_KONTAKT_BHF4_A);
SensorKontakt k_bhf4b(PIN_KONTAKT_BHF4_B);

// --------------------- STROMSENSOREN ----------------------------------------
SensorStrom strom1(PIN_ADC_BLOCK1);
SensorStrom strom2(PIN_ADC_BLOCK2);
SensorStrom strom3(PIN_ADC_BLOCK3);
SensorStrom strom4(PIN_ADC_BLOCK4);
SensorStrom strom5(PIN_ADC_BLOCK5);
SensorStrom strom6(PIN_ADC_BLOCK6);

SensorStrom stromSbhf1(PIN_ADC_SBH_GL1);
SensorStrom stromSbhf2(PIN_ADC_SBH_GL2);
SensorStrom stromSbhf3(PIN_ADC_SBH_GL3);

// --------------------- BLOCK-OBJEKTE ----------------------------------------
void initBlocks()
{
    g_blocks[0] = nullptr;

    g_blocks[1] = new Block(1, &k_block1, &strom1);
    g_blocks[2] = new Block(2, &k_block2, &strom2, &k_bhf2a, &k_bhf2b);
    g_blocks[3] = new Block(3, &k_block3, &strom3);
    g_blocks[4] = new Block(4, &k_block4, &strom4, &k_bhf4a, &k_bhf4b);
    g_blocks[5] = new Block(5, &k_block5, &strom5);
    g_blocks[6] = new Block(6, &k_block6, &strom6);

    g_blocks[7] = new Block(7, &k_sbhf1, &stromSbhf1);
    g_blocks[8] = new Block(8, &k_sbhf2, &stromSbhf2);
    g_blocks[9] = new Block(9, &k_sbhf3, &stromSbhf3);

    for (int i = 1; i <= 9; i++)
        g_blocks[i]->begin();
}

// --------------------- SCHALTGLEISE -----------------------------------------
PulseSensor g_s11(PIN_SCHALTGLEIS_S11);
PulseSensor g_s12(PIN_SCHALTGLEIS_S12);
PulseSensor g_s13(PIN_SCHALTGLEIS_S13);
PulseSensor g_s14(PIN_SCHALTGLEIS_S14);
PulseSensor g_s15(PIN_SCHALTGLEIS_S15);
PulseSensor g_s16(PIN_SCHALTGLEIS_S16);

// --------------------- WEICHEN ----------------------------------------------
SensorKontakt sensorW12(PIN_W12_RM_ABBIEG);
SensorKontakt sensorW13(PIN_W13_RM_ABBIEG);
SensorKontakt sensorW14(PIN_W14_RM_ABBIEG);
SensorKontakt sensorW15(PIN_W15_RM_ABBIEG);

Weiche w12(12, PIN_W12_GERADE, PIN_W12_ABBIEGEN, &sensorW12);
Weiche w13(13, PIN_W13_GERADE, PIN_W13_ABBIEGEN, &sensorW13);
Weiche w14(14, PIN_W14_GERADE, PIN_W14_ABBIEGEN, &sensorW14);
Weiche w15(15, PIN_W15_GERADE, PIN_W15_ABBIEGEN, &sensorW15);

// --------------------- SBHF --------------------------------------------------
ShadowYardController g_sbhf(&g_bc);
ShadowYardController& shadowController = g_sbhf;

// --------------------- PAYLOAD ----------------------------------------------
Mega2Payload g_payload;

// ============================================================================
// TIMER
// ============================================================================
uint32_t lastBlockUpdate   = 0;
uint32_t lastSbhfUpdate    = 0;
uint32_t lastWeichenUpdate = 0;
uint32_t lastPayloadUpdate = 0;

static const uint32_t BLOCK_UPDATE_MS   = 20;
static const uint32_t SBHF_UPDATE_MS    = 10;
static const uint32_t WEICHEN_UPDATE_MS = 10;
static const uint32_t PAYLOAD_UPDATE_MS = 100;

// ============================================================================
// DEBUG SERIAL (NUR MEGA2_DEBUG)
// ============================================================================
#if MEGA2_DEBUG
static char s_dbgBuf[24];
static uint8_t s_dbgLen = 0;

static void dbgProcessLine(const char* line)
{
    if (!line || !line[0]) return;

    // Single keys
    if (line[1] == '\0')
    {
        char c = line[0];
        if (c=='1') g_sbhf.onS11();
        if (c=='2') g_sbhf.onS12();
        if (c=='3') g_sbhf.onS13();
        if (c=='4') g_sbhf.onS14();
        if (c=='5') g_sbhf.onS15();
        if (c=='6') g_sbhf.onS16();
        if (c=='r') g_sbhf.onResetAck();
        if (c=='d') mega2DebugDump();
        return;
    }

    char cmd = line[0];
    int n = atoi(&line[1]);

    if (n < 1 || n > 9)
    {
        DBG_PRINTLN("[DBG] Block-ID 1..9");
        return;
    }

    switch (cmd)
    {
        case 'o': g_bc.debugSetOccupied(n, true);  break;
        case 'O': g_bc.debugSetOccupied(n, false); break;
        case 'i': g_bc.debugSetStrom(n, true);     break;
        case 'I': g_bc.debugSetStrom(n, false);    break;
        case 'x':
        case 'X': g_bc.debugClear(n);              break;
    }
}

static void dbgHandleSerial()
{
    while (Serial.available())
    {
        char ch = Serial.read();
        if (ch == '\n' || ch == '\r')
        {
            s_dbgBuf[s_dbgLen] = 0;
            if (s_dbgLen) dbgProcessLine(s_dbgBuf);
            s_dbgLen = 0;
        }
        else if (s_dbgLen < sizeof(s_dbgBuf)-1)
            s_dbgBuf[s_dbgLen++] = ch;
    }
}
#else
    DBG_PRINTLN("[DBG] Block debug disabled");
#endif

// ============================================================================
// SETUP
// ============================================================================
void setup()
{
    DBG_BEGIN(115200);
    while (!Serial && millis() < 1000) {}

    Serial.println(F("\n=== Mega2 Schattenbahnhof startet ==="));

    safetyBegin();

    k_block1.begin(); k_block2.begin(); k_block3.begin();
    k_block4.begin(); k_block5.begin(); k_block6.begin();
    k_sbhf1.begin();  k_sbhf2.begin();  k_sbhf3.begin();
    k_bhf2a.begin();  k_bhf2b.begin();
    k_bhf4a.begin();  k_bhf4b.begin();

    strom1.begin(); strom2.begin(); strom3.begin();
    strom4.begin(); strom5.begin(); strom6.begin();
    stromSbhf1.begin(); stromSbhf2.begin(); stromSbhf3.begin();

    initBlocks();

    g_power.begin();
    g_sbhf.begin();

    w12.begin(); w13.begin(); w14.begin(); w15.begin();

    g_s11.begin(); g_s12.begin(); g_s13.begin();
    g_s14.begin(); g_s15.begin(); g_s16.begin();

    megaI2C_begin();
}

// ============================================================================
// LOOP
// ============================================================================
void loop()
{
    uint32_t now = millis();

#if MEGA2_DEBUG
    dbgHandleSerial();
#endif

    safetyUpdate();

    if (now - lastBlockUpdate >= BLOCK_UPDATE_MS)
    {
        lastBlockUpdate = now;
        g_bc.update(now);
    }

    if (now - lastSbhfUpdate >= SBHF_UPDATE_MS)
    {
        lastSbhfUpdate = now;
        g_sbhf.update(now);
    }

    if (now - lastWeichenUpdate >= WEICHEN_UPDATE_MS)
    {
        lastWeichenUpdate = now;
        w12.update(now); w13.update(now);
        w14.update(now); w15.update(now);
    }

    if (now - lastPayloadUpdate >= PAYLOAD_UPDATE_MS)
    {
        lastPayloadUpdate = now;
        mega2_buildPayload(g_payload);
        megaI2C_update();
    }
}
