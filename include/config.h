#pragma once
#include <Arduino.h>

// -------------------------------------------------------------
// Mega2 – Allgemeine Konfiguration
// -------------------------------------------------------------

// I2C-Adresse für ESP32 <-> Mega2
#define I2C_SLAVE_ADDR      0x11

// DataReady-Pin Mega2 → ESP32
#include "mega2_pins.h"
#define PIN_I2C_INT         PIN_DATA_READY_M2

// -------------------------------------------------------------
// BLOCK- / SBHF- / WEICHEN-KONSTANTEN (für Payloadgrößen)
// -------------------------------------------------------------

// Anzahl Blöcke (Mega2: 6 reguläre + SBhf 3 + zukünftige Reserven)
#define NUM_BLOCKS              16

// Anzahl SBhf-Gleise
#define NUM_SBH_GLEISE          8

// Anzahl Weichen auf Mega2 (W12–W15)
#define NUM_WEICHEN             4


// -------------------------------------------------------------
// Globale Zeiger auf Block-Objekte (kommen aus BlockController.cpp)
// -------------------------------------------------------------
extern class Block* g_blocks[NUM_BLOCKS];







// -------------------------------------------------------------
// ZMPT-Spannungsmesser (oben/unten)
// -------------------------------------------------------------
extern class ZMPT101B g_trafoOben;
extern class ZMPT101B g_trafoUnten;

// -------------------------------------------------------------
// ACS-Stromsensoren (Blockströme + SBhf)
// -------------------------------------------------------------

extern class ACSStromSensor g_stromBlock1;
extern class ACSStromSensor g_stromBlock2;
extern class ACSStromSensor g_stromBlock3;
extern class ACSStromSensor g_stromBlock4;
extern class ACSStromSensor g_stromBlock5;
extern class ACSStromSensor g_stromBlock6;

extern class ACSStromSensor g_stromSBhf1;
extern class ACSStromSensor g_stromSBhf2;
extern class ACSStromSensor g_stromSBhf3;

// -------------------------------------------------------------
// Kontaktgleise (werden im BlockController verknüpft)
// Definiert in mega2_pins.h (bereits inkludiert)
// -------------------------------------------------------------
