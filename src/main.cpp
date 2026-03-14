#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <math.h>

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
#include "AdcScheduler.h"
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
#include <util/atomic.h>

// Performance test helper:
//  - MEGA2_PERF_NO_I2C=1 disables Mega2 I2C slave init/update to measure
//    I2C impact on loop jitter (maxGapUs). Sampling is ISR-driven and remains.
#ifndef MEGA2_PERF_NO_I2C
#define MEGA2_PERF_NO_I2C 0
#endif

#ifndef MEGA2_ADC_ONLY_TRAFO_OBEN
#define MEGA2_ADC_ONLY_TRAFO_OBEN 0
#endif

#ifndef MEGA2_DEBUG_STROM_BLOCKS
#define MEGA2_DEBUG_STROM_BLOCKS 0
#endif

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

// ADC scheduler sink: feed trafo sensors inside ADC ISR (jitter-proof).
static void adcIsrSink(uint8_t channel, uint16_t value)
{
    const uint8_t chOben  = (uint8_t)(PIN_ADC_TRAFO_OBEN  - A0);
    const uint8_t chUnten = (uint8_t)(PIN_ADC_TRAFO_UNTEN - A0);
    static uint8_t s_lastTrafoChannel = 0xFF;
    static uint8_t s_sameTrafoCount = 0;

    if (channel == chOben || channel == chUnten) {
        // After switching between ADC channels, the first conversions can still
        // be contaminated by the previous channel / S&H capacitor state.
        // Diagnostic step: require multiple same-channel conversions before we
        // accept a trafo sample.
        if (channel != s_lastTrafoChannel) {
            s_lastTrafoChannel = channel;
            s_sameTrafoCount = 1;
            return;
        }

        if (s_sameTrafoCount < 0xFFu) s_sameTrafoCount++;

        // Use only the later samples of each 5-sample burst.
        // 1st/2nd/3rd sample after a switch are discarded.
        if (s_sameTrafoCount <= 3u) return;

        if (channel == chOben)  { g_trafoOben.onSampleISR(value); return; }
        if (channel == chUnten) { g_trafoUnten.onSampleISR(value); return; }
    }

    // Any non-trafo channel means the ADC MUX has moved away from the trafo
    // input. Force a fresh discard when we return to a trafo channel.
    s_lastTrafoChannel = 0xFF;
    s_sameTrafoCount = 0;
}

// --------------------- ANALOG SNAPSHOT (I2C) --------------------------------
// Mega2I2C serves analog via Wire.onRequest (ISR context). We must not perform
// heavy sampling or floating-point work there.
// Therefore we build an analog snapshot in loop() and serve it as-is.
// Double-buffered to avoid tearing while the ISR is sending bytes.
Mega2AnalogPayload g_analogSnapBuf[2] = {};
volatile uint8_t g_analogSnapIdx = 0;
static uint8_t s_analogSeq = 0;

static inline void publishAnalogSnapshot(const Mega2AnalogPayload& p)
{
    const uint8_t next = (uint8_t)(g_analogSnapIdx ^ 1u);
    g_analogSnapBuf[next] = p;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        g_analogSnapIdx = next;
    }
}

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
SensorStrom strom1(PIN_ADC_BLOCK1, 0);
SensorStrom strom2(PIN_ADC_BLOCK2, 0);
SensorStrom strom3(PIN_ADC_BLOCK3, 0);
SensorStrom strom4(PIN_ADC_BLOCK4, 0);
SensorStrom strom5(PIN_ADC_BLOCK5, 0);
SensorStrom strom6(PIN_ADC_BLOCK6, 0);

SensorStrom stromSbhf1(PIN_ADC_SBH_GL1, 0);
SensorStrom stromSbhf2(PIN_ADC_SBH_GL2, 0);
SensorStrom stromSbhf3(PIN_ADC_SBH_GL3, 0);

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
#if defined(MEGA2_DEBUG_TRAFO_RAW) || defined(MEGA2_DEBUG_STROM_BLOCKS)
     Serial.begin(115200);
#endif


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
  #if !MEGA2_PERF_NO_I2C
    megaI2C_begin();
  #else
    DBG_PRINTLN(F("[PERF] I2C disabled (MEGA2_PERF_NO_I2C=1)"));
  #endif


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

    // --- Counts -> mA Skalierung (vorläufige Werte, später kalibrieren!)
    const uint16_t KI_NUM = 1;   // TODO: nach Kalibrierung anpassen
    const uint16_t KI_DEN = 1;   // TODO: nach Kalibrierung anpassen

    strom1.setScaleCountsToMA(KI_NUM, KI_DEN);
    strom2.setScaleCountsToMA(KI_NUM, KI_DEN);
    strom3.setScaleCountsToMA(KI_NUM, KI_DEN);
    strom4.setScaleCountsToMA(KI_NUM, KI_DEN);
    strom5.setScaleCountsToMA(KI_NUM, KI_DEN);
    strom6.setScaleCountsToMA(KI_NUM, KI_DEN);
    stromSbhf1.setScaleCountsToMA(KI_NUM, KI_DEN);
    stromSbhf2.setScaleCountsToMA(KI_NUM, KI_DEN);
    stromSbhf3.setScaleCountsToMA(KI_NUM, KI_DEN);

    // --- Schwelle jetzt in mA definieren
    strom1.setThreshold_mA(THR_BLOCK_OCC_MA);
    strom2.setThreshold_mA(THR_BLOCK_OCC_MA);
    strom3.setThreshold_mA(THR_BLOCK_OCC_MA);
    strom4.setThreshold_mA(THR_BLOCK_OCC_MA);
    strom5.setThreshold_mA(THR_BLOCK_OCC_MA);
    strom6.setThreshold_mA(THR_BLOCK_OCC_MA);
    stromSbhf1.setThreshold_mA(THR_BLOCK_OCC_MA);
    stromSbhf2.setThreshold_mA(THR_BLOCK_OCC_MA);
    stromSbhf3.setThreshold_mA(THR_BLOCK_OCC_MA);

    initBlocks();

    g_power.begin();
    g_trafoOben.begin();
    g_trafoUnten.begin();
    g_trafoOben.setDebugLabel("OBEN");
    g_trafoUnten.setDebugLabel("UNTEN");

    // --------------------------------------------------------------------
    // ADC Scheduler: deterministic sampling independent from loop() jitter.
    //
    // 79e Diagnose-/Zwischenstand:
    //   - Trafo-Messung wurde deutlich stabiler, sobald die langen Cluster
    //     durch kurze, haeufige Bursts ersetzt wurden.
    //   - adcIsrSink() verwirft derzeit nach jedem Trafo-Kanalwechsel
    //     die ersten 3 Samples.
    //
    // Daher fuer den naechsten Test:
    //   - Trafo weiter in kurzen 4er-Bursts
    //   - Strom vorsichtig wieder dazu, jeweils in 2er-Bursts
    //   - zunaechst nur Block 1..6, noch ohne SBHF 1..3
    //
    // Ziel:
    //   - pruefen, ob die Trafo-Stabilitaet auch mit "echten" Fremdkanaelen
    //     im MUX-Rad erhalten bleibt
    // --------------------------------------------------------------------
#if MEGA2_ADC_ONLY_TRAFO_UNTEN
    static const uint8_t s_adcSchedule[28] = {
        PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN,
        PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN,
        PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN,
        PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN,
        PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN,
        PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN,
        PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN,
    };
#elif MEGA2_ADC_ONLY_TRAFO_OBEN
    static const uint8_t s_adcSchedule[28] = {
        PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN,
        PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN,
        PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN,
        PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN,
        PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN,
        PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN,
        PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN, PIN_ADC_TRAFO_OBEN,
    };
#else
    static const uint8_t s_adcSchedule[28] = {
        // 28 Slots gesamt:
        //   - Trafo oben:   2 Bursts x 4 Slots
        //   - Trafo unten:  2 Bursts x 4 Slots
        //   - Strom Block1..6: je 1 Burst x 2 Slots
        //
        // Wirkung mit aktuellem adcIsrSink():
        //   - Trafo:  3 discard + 1 Nutzsample pro 4er-Burst
        //   - Strom:  unveraendert ueber Queue/SensorStrom
        //
        // Die Reihenfolge ist absichtlich kurzburstig:
        //   4x OBEN, 4x UNTEN, 2x B1, 2x B4,
        //   4x OBEN, 4x UNTEN, 2x B2, 2x B5, 2x B3, 2x B6
        //
        // So bleiben die Trafo-Samples zeitlich gut verteilt, waehrend wir
        // gleichzeitig kontrolliert echte Fremdkanaele in den MUX-Rad holen.
        PIN_ADC_TRAFO_OBEN,  PIN_ADC_TRAFO_OBEN,  PIN_ADC_TRAFO_OBEN,  PIN_ADC_TRAFO_OBEN,
        PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN,
        PIN_ADC_BLOCK1,      PIN_ADC_BLOCK1,
        PIN_ADC_BLOCK4,      PIN_ADC_BLOCK4,

        PIN_ADC_TRAFO_OBEN,  PIN_ADC_TRAFO_OBEN,  PIN_ADC_TRAFO_OBEN,  PIN_ADC_TRAFO_OBEN,
        PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN, PIN_ADC_TRAFO_UNTEN,
        PIN_ADC_BLOCK2,      PIN_ADC_BLOCK2,
        PIN_ADC_BLOCK5,      PIN_ADC_BLOCK5,
        PIN_ADC_BLOCK3,      PIN_ADC_BLOCK3,
        PIN_ADC_BLOCK6,      PIN_ADC_BLOCK6,
    };
#endif

    adcSchedSetIsrSink(adcIsrSink);
    adcSchedBegin(s_adcSchedule, (uint8_t)sizeof(s_adcSchedule));


    Serial.println(F("[CFG] Mega2 boot"));
    Serial.print(F("[CFG] TRAW="));
#if defined(MEGA2_DEBUG_TRAFO_RAW)
    Serial.print(1);
#else
    Serial.print(0);
#endif
    Serial.print(F(" A10ONLY="));
#if MEGA2_ADC_ONLY_TRAFO_UNTEN
    Serial.print(1);
#else
    Serial.print(0);
#endif
    Serial.print(F(" A9ONLY="));
#if MEGA2_ADC_ONLY_TRAFO_OBEN
    Serial.print(1);
#else
    Serial.print(0);
#endif
    Serial.print(F(" WINDOW_MS="));
    Serial.print(SensorTrafoAC::windowMs());
    Serial.print(F(" SAMPLE_HZ="));
    Serial.print(SensorTrafoAC::sampleHz());
    Serial.print(F(" WINDOW_SAMPLES="));
    Serial.print(SensorTrafoAC::windowSamples());
    Serial.print(F(" WIN_Q="));
    Serial.println(SensorTrafoAC::winQ());

    // Trafo channels are processed in ISR (min/max window). Disable queueing
    // to avoid pointless ring drops.
    adcSchedSetQueueEnabled(PIN_ADC_TRAFO_OBEN, false);
    adcSchedSetQueueEnabled(PIN_ADC_TRAFO_UNTEN, false);
    
    // Trafo scaling (ADC-domain Vrms -> Trafo Vrms)
    g_trafoOben.setScale((float)KV_TRAFO_OBEN_NUM / (float)KV_TRAFO_OBEN_DEN);
    g_trafoUnten.setScale((float)KV_TRAFO_UNTEN_NUM / (float)KV_TRAFO_UNTEN_DEN);
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
    

#if MEGA2_DEBUG
    // Simple loop counter (debug): average loops per second over 5s
    static uint32_t s_loopCount = 0;
    static uint32_t s_loopCountStartMs = 0;
    static uint32_t s_stromMissedTicks = 0;
    static uint32_t s_lastStromUpdateMsSeen = 0;
#endif

#if MEGA2_DEBUG
    if (s_loopCountStartMs == 0) s_loopCountStartMs = now;
    s_loopCount++;
    const uint32_t lpsDt = (uint32_t)(now - s_loopCountStartMs);
    if (lpsDt >= 5000u) {
        const uint32_t lps = (s_loopCount * 1000UL) / (lpsDt ? lpsDt : 1u);
        Serial.print(F("[LPS] loopsPerSec="));
        Serial.print(lps);
        Serial.print(F(" stromMissedTicks="));
        Serial.print(s_stromMissedTicks);
        Serial.print(F(" trafoMissedOben="));
        Serial.print(g_trafoOben.missedSamples());
        Serial.print(F(" trafoMissedUnten="));
        Serial.println(g_trafoUnten.missedSamples());
        s_stromMissedTicks = 0;
        s_loopCount = 0;
        s_loopCountStartMs = now;
    }
#endif // MEGA2_DEBUG



    // Loop timing diagnostics (max gap) – optional
#if DEBUG_LOOP_PERFORMANCE
    const uint32_t nowUs = micros();    
    static uint32_t s_lastLoopUs = 0;
    uint32_t loopDtUs = 0;
    if (s_lastLoopUs != 0) loopDtUs = (uint32_t)(nowUs - s_lastLoopUs);
    s_lastLoopUs = nowUs;
    mega2DebugLoopTick(now, loopDtUs);
#endif

    // Update Trafo sensors early so the snapshot uses fresh values.
    // Sampling itself is done in ISR (AdcScheduler).
    g_trafoOben.update(now);
    g_trafoUnten.update(now);

    // Build analog snapshot at a low rate for the ESP/UI.
    // NOTE: Analog snapshot is built from non-blocking sensors (no blocking sampling).
    static uint32_t s_lastAnalogSnapMs = 0;
    if ((uint32_t)(now - s_lastAnalogSnapMs) >= 200u)
    {
        s_lastAnalogSnapMs = now;

        Mega2AnalogPayload p{};
        p.seq = ++s_analogSeq;

        // flags bit4 (0x10): RAW debug mode
        //   vA10/vB10 carry Vrms * 10 (sensor-domain, uncalibrated)
        //   i_mA[]    carries RMS counts from current sensors (not mA)
        p.flags = 0x10;

// Fast path (non-blocking). Uses SensorTrafoAC peak-to-peak approximation and SensorStrom EMA.
        const float vA = g_trafoOben.rms();
        const float vB = g_trafoUnten.rms();
        p.vA10 = (vA <= 0.0f) ? 0u : (uint16_t)lroundf(vA * 10.0f);
        p.vB10 = (vB <= 0.0f) ? 0u : (uint16_t)lroundf(vB * 10.0f);
        for (uint8_t i = 0; i < M2_NUM_BLOCKS; i++)
        {
            Block* b = g_blocks[i + 1];
            p.i_mA[i] = b ? b->stromRmsCounts() : 0;
        }

        publishAnalogSnapshot(p);
    }


#if MEGA2_DEBUG
    dbgHandleSerial();
  #if MEGA2_DEBUG_ANALOG_TICK
    #ifndef MEGA2_PERF_NO_ANALOGTICK
      mega2DebugAnalogTick(now);
    #endif
  #endif
#endif

    safetyUpdate();

    // Schaltgleise S11..S16 (flankenbasiert) -> SBHF-Events
    sbhfHandleSchaltgleise();

    // Stromsensoren (ADC) regelmäßig aktualisieren, damit Block::update() überThreshold() sinnvoll ist
    if (now - lastStromUpdate >= STROM_UPDATE_MS)
    {
#if MEGA2_DEBUG
        // count missed scheduler ticks (if loop was blocked)
        if (s_lastStromUpdateMsSeen != 0) {
            const uint32_t dt = (uint32_t)(now - s_lastStromUpdateMsSeen);
            if (dt > (uint32_t)STROM_UPDATE_MS) {
                const uint32_t steps = dt / (uint32_t)STROM_UPDATE_MS;
                if (steps > 1u) s_stromMissedTicks += (steps - 1u);
            }
        }
        s_lastStromUpdateMsSeen = now;
#endif // MEGA2_DEBUG

        lastStromUpdate = now;
        // Power mapping: Block 1-3 = Trafo oben, Block 4-6 + SBHF 1-3 = Trafo unten
        const bool pOben  = g_trafoOben.isPowered();
        const bool pUnten = g_trafoUnten.isPowered();
        strom1.setPowered(pOben);  strom2.setPowered(pOben);  strom3.setPowered(pOben);
        strom4.setPowered(pUnten); strom5.setPowered(pUnten); strom6.setPowered(pUnten);
        stromSbhf1.setPowered(pUnten); stromSbhf2.setPowered(pUnten); stromSbhf3.setPowered(pUnten);
        strom1.update(); strom2.update(); strom3.update();
        strom4.update(); strom5.update(); strom6.update();
        stromSbhf1.update(); stromSbhf2.update(); stromSbhf3.update();
        
#if MEGA2_DEBUG_STROM_BLOCKS
        static uint32_t s_lastStromBlockLogMs = 0;
        if ((uint32_t)(now - s_lastStromBlockLogMs) >= 250u)
        {
            s_lastStromBlockLogMs = now;

            auto logStromBlock = [&](uint8_t blockId, const SensorStrom& s)
            {
                Serial.print(F("[IBLOCK] t="));
                Serial.print(now);
                Serial.print(F(" block="));
                Serial.print(blockId);
                Serial.print(F(" raw="));
                Serial.print(s.raw());
                Serial.print(F(" off="));
                Serial.print(s.offset());
                Serial.print(F(" abs="));
                Serial.print(s.absDevCounts());
                Serial.print(F(" rmsRaw="));
                Serial.print(s.rmsCountsRaw());
                Serial.print(F(" rms="));
                Serial.print(s.rmsCounts());
                Serial.print(F(" mA="));
                Serial.print(s.rms_mA());
                Serial.print(F(" min="));
                Serial.print(s.lastWinMin());
                Serial.print(F(" max="));
                Serial.print(s.lastWinMax());
                Serial.print(F(" n="));
                Serial.println(s.lastWinSamples());
            };

            logStromBlock(2, strom2);
            logStromBlock(4, strom4);
        }
#endif
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

      #if !MEGA2_PERF_NO_I2C
        megaI2C_update();
      #endif
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
