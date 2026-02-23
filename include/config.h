#pragma once
#include <Arduino.h>

// -------------------------------------------------------------
// Mega2 – Allgemeine Konfiguration (minimal, konfliktfrei)
// -------------------------------------------------------------

// I2C-Adresse für ESP32 <-> Mega2
#define I2C_SLAVE_ADDR 0x11

#include "mega2_pins.h"

// DataReady-Pin Mega2 → ESP32
#define PIN_I2C_INT PIN_DATA_READY_M2

// -------------------------------------------------------------
// Größen (Payload / Reserven)
// -------------------------------------------------------------

// Anzahl Blöcke (B1..B9)
#define BLOCK_COUNT 9

// Arrays meist 1-basiert => +1 für Index 0
#define NUM_BLOCKS  (BLOCK_COUNT + 1)

// SBHF-Gleise (GF1..GF3)
#define NUM_SBH_GLEISE 3

// Weichen im SBHF
#define NUM_WEICHEN 4

// -------------------------------------------------------------
// Globale Zeiger auf Block-Objekte (werden in main.cpp gesetzt)
// -------------------------------------------------------------

class Block;
extern Block* g_blocks[];
