#include <Arduino.h>
#include <Wire.h>
#include <string.h>

#include "mega2_pins.h"
#include <EEPROM.h>
#include "mem_utils.h"


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
#include "threshold_values.h"

#include "PowerControl.h"
#include "Mega2PowerControl.h"

#include "ShadowYardController.h"
#include "Mega2I2C.h"

#include "proto_mega2.h"
#include "safety.h"
#include "mega2_debug.h"
#include "Mega2Debug.h"
#include "Mega2Status.h"
#include "Mega2RunMode.h"

// ============================================================================
// GLOBALE OBJEKTE
// ============================================================================
// Boot-ID: wird bei jedem Reset inkrementiert.
// Ermöglicht es externen Clients (ESP/Web),
// einen Controller-Neustart zuverlässig zu erkennen,
// auch wenn die Verbindung kurzzeitig unterbrochen war.
uint16_t g_bootId = 0;

static uint16_t loadIncBootCounter()
{
    // Address 0..1 reserved for boot counter (uint16_t)
    uint16_t c = 0;
    EEPROM.get(0, c);
    c++;
    EEPROM.put(0, c);
    return c;
}


static uint16_t makeBootId16()
{
    // Guaranteed change per boot via EEPROM counter, then mix with timing.
    uint16_t x = loadIncBootCounter();

    x ^= (uint16_t)micros();
    x ^= (uint16_t)(millis() & 0xFFFFu);
#if defined(__AVR__)
    x ^= (uint16_t)TCNT1;
    x ^= (uint16_t)(TCNT0 << 8);
    x ^= (uint16_t)(TCNT2 << 4);
#endif

    // light diffusion
    x ^= (x << 7);
    x ^= (x >> 9);
    x ^= (x << 8);

    if (x == 0) x = 1;
    return x;
}


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
SensorStrom strom1(PIN_ADC_BLOCK1, THR_BLOCK_OCC_COUNTS);
SensorStrom strom2(PIN_ADC_BLOCK2, THR_BLOCK_OCC_COUNTS);
SensorStrom strom3(PIN_ADC_BLOCK3, THR_BLOCK_OCC_COUNTS);
SensorStrom strom4(PIN_ADC_BLOCK4, THR_BLOCK_OCC_COUNTS);
SensorStrom strom5(PIN_ADC_BLOCK5, THR_BLOCK_OCC_COUNTS);
SensorStrom strom6(PIN_ADC_BLOCK6, THR_BLOCK_OCC_COUNTS);

SensorStrom stromSbhf1(PIN_ADC_SBH_GL1, THR_BLOCK_OCC_COUNTS);
SensorStrom stromSbhf2(PIN_ADC_SBH_GL2, THR_BLOCK_OCC_COUNTS);
SensorStrom stromSbhf3(PIN_ADC_SBH_GL3, THR_BLOCK_OCC_COUNTS);

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

// Forward decl: g_sbhf wird weiter unten definiert, aber hier schon benutzt.
class ShadowYardController;
extern ShadowYardController g_sbhf;

// --------------------- SCHALTGLEISE -> SBHF DISPATCH -------------------------
static void sbhfHandleSchaltgleise()
{
    // PulseSensor ist flankenbasiert (HIGH->LOW).
    // Wichtig: muss zyklisch aufgerufen werden, sonst triggert S11..S16 nie.
    // In DIAG_TEST sollen Sensoren weiterhin gelesen/gezählt werden,
    // aber KEINE Automatik-Events in die SBHF-Logik einspeisen.
    const bool dispatch = !mega2IsDiagTest();

    if (g_s11.fellEdge()) { DBG_PRINTLN(F("[SBHF] S11 pulse")); if (dispatch) g_sbhf.onS11(); }
    if (g_s12.fellEdge()) { DBG_PRINTLN(F("[SBHF] S12 pulse")); if (dispatch) g_sbhf.onS12(); }
    if (g_s13.fellEdge()) { DBG_PRINTLN(F("[SBHF] S13 pulse")); if (dispatch) g_sbhf.onS13(); }
    if (g_s14.fellEdge()) { DBG_PRINTLN(F("[SBHF] S14 pulse")); if (dispatch) g_sbhf.onS14(); }
    if (g_s15.fellEdge()) { DBG_PRINTLN(F("[SBHF] S15 pulse")); if (dispatch) g_sbhf.onS15(); }
    if (g_s16.fellEdge()) { DBG_PRINTLN(F("[SBHF] S16 pulse")); if (dispatch) g_sbhf.onS16(); }
}


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
static uint32_t lastBlockUpdate = 0;
static uint32_t lastSbhfUpdate = 0;
static uint32_t lastWeichenUpdate = 0;
static uint32_t lastStromUpdate  = 0;
static uint32_t lastPayloadUpdate = 0;

static const uint32_t BLOCK_UPDATE_MS   = 20;
static const uint32_t SBHF_UPDATE_MS    = 10;
static const uint32_t WEICHEN_UPDATE_MS = 10;
static const uint32_t STROM_UPDATE_MS   = 20; // ADC-Update für Stromsensoren
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
    // Doku: SYS flags in HEX (on-change) für stabilere Logs
    Serial.print(F("SYS flags=0x"));
    if (flags < 0x1000) Serial.print('0');
    if (flags < 0x0100) Serial.print('0');
    if (flags < 0x0010) Serial.print('0');
    Serial.print(flags, HEX);
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

        // ------------------------------------------------------------
        // Mini-Help (SIM): '?' zeigt Debug/SIM-Kommandos
        // Hinweis: 'h'/'H' sind Stopzone-Kontakt Force (SIM)
        // ------------------------------------------------------------
        if (c == '?')
        {
            DBG_PRINTLN(F("=== SIM CMD HELP (MEGA2_SIM_MODE=1) ==="));
            DBG_PRINTLN(F("Single-Key:"));
            DBG_PRINTLN(F("  d    -> Debug Dump"));
            DBG_PRINTLN(F("  a    -> ACK senden"));
            DBG_PRINTLN(F("  n    -> NOTHALT setzen (EMERGENCY)"));
            DBG_PRINTLN(F("  p    -> POWER ON (wenn erlaubt)"));
#if MEGA2_SIM_MODE
            DBG_PRINTLN(F("  T/t  -> Trafo-Unten Force ON/OFF (SIM)"));
            DBG_PRINTLN(F("  h/H  -> Stopzone Kontakt Force OCC/FREE (SIM)"));
            DBG_PRINTLN(F("  1..6 -> SBHF Sensor-Keys S11..S16 (SIM)"));
#endif
            DBG_PRINTLN(F("Line Commands (Block N=1..9):"));
            DBG_PRINTLN(F("  oN/ON -> Block belegt/frei"));
            DBG_PRINTLN(F("  uN/UN -> HARD: BN = 1400mA / OFF (Double-Occ hard)"));
            DBG_PRINTLN(F("  vN/VN -> BASE: BN = 200mA / OFF (EMA-Basis)"));
            DBG_PRINTLN(F("  wN/WN -> STEP: BN = 350mA / zurueck auf 200mA"));
            DBG_PRINTLN(F("  kN/KN -> SHORT ON/OFF (SIM)"));
            DBG_PRINTLN(F("  xN/XN -> Clear Debug-State (SIM)"));
            DBG_PRINTLN(F(""));
            DBG_PRINTLN(F("Beispiele (B2):"));
            DBG_PRINTLN(F("  Hard:     o2, u2, (>=1s), a, U2, a"));
            DBG_PRINTLN(F("  Adaptive: o2, v2, (>=2-3s), w2, (>=1s), a, W2, a"));
            DBG_PRINTLN(F("TESTPLAN v1.2 (B2): Hard=o2 u2 wait a U2 a | Adapt=o2 v2 wait w2 wait a W2 a"));
            DBG_PRINTLN(F("==================================="));
            return;
        }

        // Shadow yard debug: Sensors (SIM only)
#if MEGA2_SIM_MODE
        if (c=='1') g_sbhf.onS11();
        if (c=='2') g_sbhf.onS12();
        if (c=='3') g_sbhf.onS13();
        if (c=='4') g_sbhf.onS14();
        if (c=='5') g_sbhf.onS15();
        if (c=='6') g_sbhf.onS16();
#else
        if (c>='1' && c<='6') { DBG_PRINTLN(F("[DBG] SIM sensor keys disabled (MEGA2_SIM_MODE=0)")); return; }
#endif

        // 'r' nur in SIM (Fehlbedienung in HW vermeiden)
#if MEGA2_SIM_MODE
        if (c=='r') g_sbhf.onResetAck();
#endif
        if (c=='d') mega2DebugDump();

        // SAFETY debug
        if (c == 'p')
        {
            const bool ok = safetyPowerOn();
            if (ok) DBG_PRINTLN(F("[DBG] POWER ON OK"));
            else    DBG_PRINTLN(F("[DBG] POWER ON BLOCKED"));
        }

        // Power OFF ohne Emergency (nur SSR/Outputs aus)
        // -> hilfreich für Service/Debug ohne SafetyLock
        if (c == 'N')
        {
            g_power.setMainPower(false);
            g_power.setSbhfGleis(1, false);
            g_power.setSbhfGleis(2, false);
            g_power.setSbhfGleis(3, false);
            g_power.setBlock5ToSBhf(false);
            DBG_PRINTLN(F("[DBG] POWER OFF (no emergency)"));
        }


        if (c == 'n')
        {
            safetySetEmergency(true);
            DBG_PRINTLN(F("[DBG] NOTHALT"));
        }

        if (c == 'a')
        {
            const bool ok = safetyResetEmergency();
            if (ok) DBG_PRINTLN(F("[DBG] ACK OK"));
            else    DBG_PRINTLN(F("[DBG] ACK BLOCKED"));
        }

        // Trafo-Unten Force (SIM only)
#if MEGA2_SIM_MODE
        if (c=='T') { safetyDebugForceTrafoUntenPowered(true);  DBG_PRINTLN(F("[DBG] TRAFO_UNTEN FORCED=ON")); }
        if (c=='t') { safetyDebugForceTrafoUntenPowered(false); DBG_PRINTLN(F("[DBG] TRAFO_UNTEN FORCED=OFF")); }

        // Stopzone-Kontakt (k_nothalt) Force (SIM only)
        if (c=='h') { k_nothalt.debugForce(true);  DBG_PRINTLN(F("[DBG] K_NOTHALT FORCED=OCC")); }
        if (c=='H') { k_nothalt.debugForce(false); DBG_PRINTLN(F("[DBG] K_NOTHALT FORCED=FREE")); }
#else
        if (c=='T' || c=='t' || c=='h' || c=='H') { DBG_PRINTLN(F("[DBG] SIM cmd disabled (MEGA2_SIM_MODE=0)")); return; }
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
        DBG_PRINTLN(F("[DBG] Block-ID 1..9"));
        return;
    }

#if !MEGA2_SIM_MODE
    // In HW-Modus keine Manipulation per Serial zulassen
    if (cmd=='o' || cmd=='O' || cmd=='i' || cmd=='I' ||
        cmd=='x' || cmd=='X' || cmd=='k' || cmd=='K' ||
        cmd=='u' || cmd=='U' || cmd=='v' || cmd=='V' || cmd=='w' || cmd=='W')
    {
        DBG_PRINTLN(F("[DBG] SIM cmd disabled (MEGA2_SIM_MODE=0)"));
        return;
    }
#endif

    switch (cmd)
    {
        case 'o': g_bc.debugSetOccupied(n, true);  break;
        case 'O': g_bc.debugSetOccupied(n, false); break;
        case 'i': g_bc.debugSetStrom(n, true);     break;
        case 'I': g_bc.debugSetStrom(n, false);    break;

        // 'u' => force synthetic current for safety heuristics (e.g. double occupancy)
        // 'U' => clear forced current
        // force hard current for safety tests (DOUBLE_OCC hard case)
        case 'u':
            safetyDebugForceBlockCurrentMa(n, 1400); // >1200mA and <SHORT_THRESHOLD_MA
            DBG_PRINTF("[DBG] FORCE_CURRENT: B%d = 1400mA\n", n);
            break;
        case 'U':
            safetyDebugForceBlockCurrentMa(n, 0);
            DBG_PRINTF("[DBG] FORCE_CURRENT: B%d = OFF\n", n);
            break;

        // adaptive double-occupancy test: baseline current
        case 'v':
            safetyDebugForceBlockCurrentMa(n, 200);   // base current
            DBG_PRINTF("[DBG] FORCE_CURRENT BASE: B%d = 200mA\n", n);
            break;
        case 'V':
            safetyDebugForceBlockCurrentMa(n, 0);
            DBG_PRINTF("[DBG] FORCE_CURRENT BASE: B%d = OFF\n", n);
            break;

        // adaptive double-occupancy test: second consumer
        case 'w':
            safetyDebugForceBlockCurrentMa(n, 350);   // second load
            DBG_PRINTF("[DBG] FORCE_CURRENT STEP: B%d = 350mA\n", n);
            break;
        case 'W':
            safetyDebugForceBlockCurrentMa(n, 200);   // back to base
            DBG_PRINTF("[DBG] FORCE_CURRENT STEP: B%d -> 200mA\n", n);
            break;

        case 'k':
        case 'K':
#if MEGA2_SIM_MODE
            // 'k' => short ON, 'K' => short OFF
            g_bc.debugSetStromShort(n, (cmd == 'k'));
#else
            DBG_PRINTLN(F("[DBG] SIM cmd disabled (MEGA2_SIM_MODE=0)"));
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
                (ch == 'p' || ch == 'n' || ch == 'N' || ch == 'a' || ch == 'r' || ch == 'd' || ch == 'T' || ch == 't') ||
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
    // Mega2560 I2C Pins: SDA=20, SCL=21 (Hardware I2C/TWI)
    // Neue Boot-Instanz signalisieren
    // BootId must change on every reboot so ESP can detect Mega reboots reliably.
    // (Not persistent; "random per boot" is enough.)
    g_bootId = makeBootId16();

    DBG_BEGIN(115200);
    while (!Serial && millis() < 1000) {}

    // --------------------------------------------------------------------
    // DEFENSIVE I2C BUS RELEASE (wichtig bei "SCL hängt LOW beim gemeinsamen Boot")
    // - garantiert: wir treiben SDA/SCL NICHT aktiv LOW
    // - Pullups helfen, dass der Bus früh sauber HIGH ist
    // --------------------------------------------------------------------
    pinMode(20, INPUT_PULLUP); // SDA
    pinMode(21, INPUT_PULLUP); // SCL

    safetyBegin();

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

    // I2C Slave früh aktivieren, bevor lange Hardware-Inits laufen.
    megaI2C_begin();


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

    MemUtils::resetMinFree();

    DBG_PRINT(F("[MEM] boot free="));
    DBG_PRINT(MemUtils::freeMemory());
    DBG_PRINT(F(" min="));
    DBG_PRINT(MemUtils::minFreeMemory());
    DBG_PRINTLN(F(""));

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
  #if MEGA2_DEBUG_ANALOG_TICK
    mega2DebugAnalogTick(now);
  #endif
#endif

    safetyUpdate();

    // Schaltgleise S11..S16 (flankenbasiert) -> SBHF-Events
    sbhfHandleSchaltgleise();

    // Stromsensoren (ADC) regelmäßig aktualisieren, damit Block::update() überThreshold() sinnvoll ist
    if (now - lastStromUpdate >= STROM_UPDATE_MS)
    {
        lastStromUpdate = now;
        strom1.update(); strom2.update(); strom3.update();
        strom4.update(); strom5.update(); strom6.update();
        stromSbhf1.update(); stromSbhf2.update(); stromSbhf3.update();
    }


    // ------------------------------------------------------------
    // AUTOMATIK-PFADE
    // In DIAG_TEST werden die Automatik-Schaltpfade pausiert.
    // Safety bleibt aktiv und kann weiterhin Relais hart abschalten.
    // ------------------------------------------------------------
    if (!mega2IsDiagTest())
    {
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
    
#if MEGA2_DEBUG
    static uint32_t s_lastMemLogMs = 0;
    

    if ((uint32_t)(now - s_lastMemLogMs) >= 5000u) {
        s_lastMemLogMs = now;

        const int freeNow = MemUtils::freeMemory();
        // updateMinFree() brauchst du hier nicht zwingend, minFreeMemory() wird im Setup initialisiert
        const int minNow  = MemUtils::minFreeMemory();

        DBG_PRINT(F("[MEM] free="));
        DBG_PRINT(freeNow);
        DBG_PRINT(F(" min="));
        DBG_PRINT(minNow);
        DBG_PRINTLN(F(""));
    }
#endif
}
