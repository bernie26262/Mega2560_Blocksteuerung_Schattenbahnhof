#include <Arduino.h>
#include <Wire.h>
#include <string.h>

#include "mega2_pins.h"

#ifndef MEGA2_SIM_MODE
#define MEGA2_SIM_MODE 0
#endif

// Nothalt-Kontaktgleis (Stopzone-Kontakt) – muss in Block 6 Belegung einfließen
#ifndef PIN_KONTAKT_NOTHALT
#define PIN_KONTAKT_NOTHALT 19
#endif

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
#include "Mega2Status.h"

// ============================================================================
// GLOBALE OBJEKTE
// ============================================================================
// Boot-ID: wird bei jedem Reset inkrementiert.
// Ermöglicht es externen Clients (ESP/Web),
// einen Controller-Neustart zuverlässig zu erkennen,
// auch wenn die Verbindung kurzzeitig unterbrochen war.
uint16_t g_bootId = 0;

// --------------------- BLOCKS -----------------------------------------------
// IDs sind 1-basiert (Index 0 bleibt nullptr)
static constexpr uint8_t BLOCK_COUNT = 9;

Block* g_blocks[MEGA2_MAX_BLOCKS];                 // Reserve, aber count() = 9
BlockController g_bc(g_blocks, BLOCK_COUNT);
BlockController& blockController = g_bc;

// --------------------- POWER CONTROL ----------------------------------------
Mega2PowerControl g_power;

// --------------------- TRAFO-SPANNUNG (ZMPT101B) ----------------------------
SensorTrafoAC g_trafoOben(PIN_ADC_TRAFO_OBEN);
SensorTrafoAC g_trafoUnten(PIN_ADC_TRAFO_UNTEN);

// --------------------- KONTAKTGLEISE ----------------------------------------
SensorKontakt k_block1(PIN_KONTAKT_BLOCK1);
SensorKontakt k_block2(PIN_KONTAKT_BLOCK2);
SensorKontakt k_block3(PIN_KONTAKT_BLOCK3);
SensorKontakt k_block4(PIN_KONTAKT_BLOCK4);
SensorKontakt k_block5(PIN_KONTAKT_BLOCK5);
SensorKontakt k_block6(PIN_KONTAKT_BLOCK6);
SensorKontakt k_nothalt(PIN_KONTAKT_NOTHALT);

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
static void initBlocks()
{
    for (auto &b : g_blocks) b = nullptr;

    g_blocks[1] = new Block(1, &k_block1, &strom1);
    g_blocks[2] = new Block(2, &k_block2, &strom2, &k_bhf2a, &k_bhf2b);
    g_blocks[3] = new Block(3, &k_block3, &strom3);
    g_blocks[4] = new Block(4, &k_block4, &strom4, &k_bhf4a, &k_bhf4b);
    g_blocks[5] = new Block(5, &k_block5, &strom5);
    // Block 6: zusätzlicher Kontakt "Stopzone / Nothalt"
    g_blocks[6] = new Block(6, &k_block6, &strom6, &k_nothalt);

    g_blocks[7] = new Block(7, &k_sbhf1, &stromSbhf1);
    g_blocks[8] = new Block(8, &k_sbhf2, &stromSbhf2);
    g_blocks[9] = new Block(9, &k_sbhf3, &stromSbhf3);

    for (int i = 1; i <= BLOCK_COUNT; i++)
        if (g_blocks[i]) g_blocks[i]->begin();
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
// --------------------- SYSTEM STATUS (ESP read-only) -------------------------
SystemStatus g_systemStatus;

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

static bool s_forceSysFlagsPrint = false;

static void dbgPrintSysFlags(uint16_t flags)
{
    Serial.print(F("SYS flags=0b"));
    Serial.print(flags, BIN);
    Serial.print(F(" ["));

    bool first = true;
    auto add = [&](const __FlashStringHelper* name)
    {
        if (!first) Serial.print(F(","));
        Serial.print(name);
        first = false;
    };

    if (flags == 0)
    {
        add(F("OK"));
    }
    else
    {
        if (flags & SYS_NOTAUS_ACTIVE)   add(F("NOTAUS"));
        if (flags & SYS_POWER_ON)        add(F("PWR"));
        if (flags & SYS_ERROR_PRESENT)   add(F("ERR"));
        if (flags & SYS_WARNING_PRESENT) add(F("WARN"));
        uint16_t unknown = flags & ~(SYS_NOTAUS_ACTIVE | SYS_POWER_ON | SYS_ERROR_PRESENT | SYS_WARNING_PRESENT);
        if (unknown)
        {
            if (!first) Serial.print(F(","));
            Serial.print(F("UNK=0x"));
            Serial.print(unknown, HEX);
            first = false;
        }
    }

    Serial.println(F("]"));
}

static void dbgProcessLine(const char* line, bool logCmd)
{
    if (!line || !line[0]) return;

    if (logCmd)
    {
        Serial.print(F("[DBG] CMD="));
        Serial.println(line);
    }

    // -------------------------
    // Single-Key Commands
    // -------------------------
    if (line[1] == '\0')
    {
        const char c = line[0];

        // Shadow yard debug: Sensors (SIM only)
#if MEGA2_SIM_MODE
        if (c=='1') g_sbhf.onS11();
        if (c=='2') g_sbhf.onS12();
        if (c=='3') g_sbhf.onS13();
        if (c=='4') g_sbhf.onS14();
        if (c=='5') g_sbhf.onS15();
        if (c=='6') g_sbhf.onS16();
#else
        if (c>='1' && c<='6') { DBG_PRINTLN("[DBG] SIM sensor keys disabled (MEGA2_SIM_MODE=0)"); return; }
#endif

        if (c=='r') g_sbhf.onResetAck();
        if (c=='d') mega2DebugDump();

        // SAFETY debug
        if (c == 'p')
        {
            const bool ok = safetyPowerOn();
            DBG_PRINTLN(ok ? "[DBG] POWER ON OK" : "[DBG] POWER ON BLOCKED");
        }

        if (c == 'n')
        {
            safetySetEmergency(true);
            DBG_PRINTLN("[DBG] NOTHALT");
        }

        if (c == 'a')
        {
            const bool ok = safetyResetEmergency();
            DBG_PRINTLN(ok ? "[DBG] ACK OK" : "[DBG] ACK BLOCKED");
        }

        // Trafo-Unten Force (SIM only)
#if MEGA2_SIM_MODE
        if (c=='T') { safetyDebugForceTrafoUntenPowered(true);  DBG_PRINTLN("[DBG] TRAFO_UNTEN FORCED=ON"); }
        if (c=='t') { safetyDebugForceTrafoUntenPowered(false); DBG_PRINTLN("[DBG] TRAFO_UNTEN FORCED=OFF"); }

        // Stopzone-Kontakt (k_nothalt) Force (SIM only)
        if (c=='h') { k_nothalt.debugForce(true);  DBG_PRINTLN("[DBG] K_NOTHALT FORCED=OCC"); }
        if (c=='H') { k_nothalt.debugForce(false); DBG_PRINTLN("[DBG] K_NOTHALT FORCED=FREE"); }
#else
        if (c=='T' || c=='t' || c=='h' || c=='H') { DBG_PRINTLN("[DBG] SIM cmd disabled (MEGA2_SIM_MODE=0)"); return; }
#endif

        s_forceSysFlagsPrint = true;
        return;
    }

    // -------------------------
    // Line Commands: o6/O6/i6/I6/k6/K6/x6
    // -------------------------
    const char cmd = line[0];
    const int n    = atoi(&line[1]);

    if (n < 1 || n > BLOCK_COUNT)
    {
        DBG_PRINTLN("[DBG] Block-ID 1..9");
        return;
    }

#if !MEGA2_SIM_MODE
    // In HW-Modus keine Manipulation per Serial zulassen
    if (cmd=='o' || cmd=='O' || cmd=='i' || cmd=='I' ||
        cmd=='x' || cmd=='X' || cmd=='k' || cmd=='K')
    {
        DBG_PRINTLN("[DBG] SIM cmd disabled (MEGA2_SIM_MODE=0)");
        return;
    }
#endif

    switch (cmd)
    {
        case 'o': g_bc.debugSetOccupied(n, true);  break;
        case 'O': g_bc.debugSetOccupied(n, false); break;
        case 'i': g_bc.debugSetStrom(n, true);     break;
        case 'I': g_bc.debugSetStrom(n, false);    break;

        case 'k':
        case 'K':
#if MEGA2_SIM_MODE
            // 'k' => short ON, 'K' => short OFF
            g_bc.debugSetStromShort(n, (cmd == 'k'));
#else
            DBG_PRINTLN("[DBG] SIM cmd disabled (MEGA2_SIM_MODE=0)");
#endif
            break;

        case 'x':
        case 'X': g_bc.debugClear(n);              break;
    }

    s_forceSysFlagsPrint = true;
}

static void dbgHandleSerial()
{
    while (Serial.available())
    {
        const char ch = static_cast<char>(Serial.read());

        // Single-Key sofort (nur wenn keine Zeile im Aufbau ist)
        if (s_dbgLen == 0)
        {
            const bool isSingle =
                (ch == 'p' || ch == 'n' || ch == 'a' || ch == 'r' || ch == 'd' || ch == 'T' || ch == 't') ||
                ((ch >= '1' && ch <= '6') && MEGA2_SIM_MODE) ||
                ((ch == 'h' || ch == 'H') && MEGA2_SIM_MODE);

            if (isSingle)
            {
                char tmp[2] = { ch, 0 };
                dbgProcessLine(tmp, /*logCmd=*/true);
                continue;
            }
        }

        // Zeilenmodus
        if (ch == '\n' || ch == '\r')
        {
            s_dbgBuf[s_dbgLen] = 0;
            if (s_dbgLen)
                dbgProcessLine(s_dbgBuf, /*logCmd=*/true);
            s_dbgLen = 0;
        }
        else if (s_dbgLen < sizeof(s_dbgBuf) - 1)
        {
            s_dbgBuf[s_dbgLen++] = ch;
        }
        else
        {
            // Buffer voll -> verwerfen
            s_dbgLen = 0;
        }
    }
}

static void dbgMaybePrintSysFlags(uint16_t flags)
{
    static uint16_t lastFlags = 0xFFFF;

    if (flags != lastFlags || s_forceSysFlagsPrint)
    {
        dbgPrintSysFlags(flags);
        lastFlags = flags;
        s_forceSysFlagsPrint = false;
    }
}
#endif // MEGA2_DEBUG

// ============================================================================
// SETUP
// ============================================================================
void setup()
{
    // Neue Boot-Instanz signalisieren
    g_bootId++;

    DBG_BEGIN(115200);
    while (!Serial && millis() < 1000) {}

    safetyBegin();

    // Kontaktgleise
    k_block1.begin(); k_block2.begin(); k_block3.begin();
    k_block4.begin(); k_block5.begin(); k_block6.begin();
    k_nothalt.begin();

    k_sbhf1.begin();  k_sbhf2.begin();  k_sbhf3.begin();
    k_bhf2a.begin();  k_bhf2b.begin();
    k_bhf4a.begin();  k_bhf4b.begin();

    // Weichen-RM
    sensorW12.begin(); sensorW13.begin(); sensorW14.begin(); sensorW15.begin();

    // Stromsensoren
    strom1.begin(); strom2.begin(); strom3.begin();
    strom4.begin(); strom5.begin(); strom6.begin();
    stromSbhf1.begin(); stromSbhf2.begin(); stromSbhf3.begin();

    initBlocks();

    g_power.begin();
    g_trafoOben.begin();
    g_trafoUnten.begin();
    g_sbhf.begin();

    w12.begin(); w13.begin(); w14.begin(); w15.begin();

    g_s11.begin(); g_s12.begin(); g_s13.begin();
    g_s14.begin(); g_s15.begin(); g_s16.begin();

    // --------------------------------------------------------------------
    // IMPORTANT: ESP polls SystemStatus immediately after boot.
    // If we enable I2C before building g_systemStatus at least once,
    // the first read can return uninitialized bytes. The ESP then rejects
    // the packet (e.g. ver=2/size=4/node=0) and marks Mega2 offline.
    //
    // Therefore: build a valid status once BEFORE megaI2C_begin().
    // --------------------------------------------------------------------
    memset(&g_systemStatus, 0, sizeof(g_systemStatus));
    buildMega2SystemStatus(g_systemStatus);

    megaI2C_begin();
}

// ============================================================================
// LOOP
// ============================================================================
void loop()
{
    const uint32_t now = millis();

    g_trafoOben.update(now);
    g_trafoUnten.update(now);

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

        // interner / Debug-Payload
        mega2_buildPayload(g_payload);

        // externer, stabiler Systemstatus (ESP read-only)
        buildMega2SystemStatus(g_systemStatus);

    #if MEGA2_DEBUG
        dbgMaybePrintSysFlags(g_systemStatus.flags);
    #endif

        megaI2C_update();
    }
}
