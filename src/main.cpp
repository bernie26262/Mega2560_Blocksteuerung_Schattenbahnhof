#include <Arduino.h>
#include "mega2_pins.h"

#include "Block.h"
#include "BlockController.h"
#include "Weiche.h"

#include "PulseSensor.h"
#include "PowerControl.h"
#include "Mega2PowerControl.h"

#include "ShadowYardController.h"
#include "Mega2I2C.h"   // sendStatus() später implementieren

// ============================================================================
// GLOBALE OBJEKTE
// ============================================================================

// --------------------- BLOCKS --------------------------
Block* g_blocks[16];           // zentrale Definition
BlockController g_bc(g_blocks);

// --------------------- POWER ---------------------------
Mega2PowerControl g_power;

// --------------------- SENSOREN S11–S16 ----------------
PulseSensor g_s11(PIN_SCHALTGLEIS_S11);
PulseSensor g_s12(PIN_SCHALTGLEIS_S12);
PulseSensor g_s13(PIN_SCHALTGLEIS_S13);
PulseSensor g_s14(PIN_SCHALTGLEIS_S14);
PulseSensor g_s15(PIN_SCHALTGLEIS_S15);
PulseSensor g_s16(PIN_SCHALTGLEIS_S16);

// --------------------- WEICHEN -------------------------
extern Weiche g_weichen[20];

#define W12_INDEX 12
#define W13_INDEX 13
#define W14_INDEX 14
#define W15_INDEX 15

// --------------------- SCHATTENBAHNHOF -----------------
ShadowYardController g_sbhf(
    &g_bc,
    &g_weichen[W12_INDEX],
    &g_weichen[W13_INDEX],
    &g_weichen[W14_INDEX],
    &g_weichen[W15_INDEX],
    &g_power,
    &g_s11, &g_s12, &g_s13, &g_s14, &g_s15, &g_s16
);

// ============================================================================
// TIMER SYSTEM
// ============================================================================

uint32_t lastBlockUpdate = 0;
uint32_t lastSbhfUpdate  = 0;
uint32_t lastI2CUpdate   = 0;
uint32_t lastSafetyCheck = 0;

static const uint32_t BLOCK_UPDATE_INTERVAL = 20;   // 50 Hz
static const uint32_t SBHF_UPDATE_INTERVAL  = 10;   // 100 Hz
static const uint32_t I2C_SEND_INTERVAL     = 100;  // 10 Hz
static const uint32_t SAFETY_INTERVAL       = 50;   // 20 Hz



// ============================================================================
// SETUP
// ============================================================================
void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.println(F("\n=== Mega2 Schattenbahnhof startet ==="));

    // Sensordefinition
    g_s11.begin();
    g_s12.begin();
    g_s13.begin();
    g_s14.begin();
    g_s15.begin();
    g_s16.begin();

    // Block-Objekte initialisieren
    for (int i = 0; i < 16; i++)
        if (g_blocks[i]) g_blocks[i]->begin();

    g_bc.begin();
    g_power.begin();
    g_sbhf.begin();

    // DataReady-Pin (Signal an ESP)
    pinMode(PIN_DATA_READY_M2, OUTPUT);
    digitalWrite(PIN_DATA_READY_M2, LOW);

    // I2C
    Mega2I2C::begin();
}



// ============================================================================
// LOOP — Timerbasierte Task Engine
// ============================================================================
void loop()
{
    uint32_t now = millis();

    // ---------------- BLOCK UPDATE (50 Hz) -------------------
    if (now - lastBlockUpdate >= BLOCK_UPDATE_INTERVAL)
    {
        lastBlockUpdate = now;
        g_bc.update(now);
    }

    // ---------------- SBHF UPDATE (100 Hz) -------------------
    if (now - lastSbhfUpdate >= SBHF_UPDATE_INTERVAL)
    {
        lastSbhfUpdate = now;
        g_sbhf.update(now);
    }

    // ---------------- SAFETY (20 Hz) -------------------------
    if (now - lastSafetyCheck >= SAFETY_INTERVAL)
    {
        lastSafetyCheck = now;

        // TODO: Blockströme, Trafos, weitere Not-Aus-Signale
    }

    // ---------------- I2C SEND STATUS (10 Hz) ----------------
    if (now - lastI2CUpdate >= I2C_SEND_INTERVAL)
    {
        lastI2CUpdate = now;

        Mega2I2C::sendStatus(g_bc, g_sbhf, g_power);

        // DataReady-Pin kurz pulsen
        digitalWrite(PIN_DATA_READY_M2, HIGH);
        delayMicroseconds(200);
        digitalWrite(PIN_DATA_READY_M2, LOW);
    }
}
