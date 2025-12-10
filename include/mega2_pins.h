#pragma once
#include <Arduino.h>

/*
   ---------------------------------------------------------
                 Mega2 – Pinbelegung (FINAL)
             inkl. Bahnhofskontakte + DataReady
   ---------------------------------------------------------
*/

// ---------------------------------------------------------
// Weichensteuerung W12–W15 (SBhf)
// ---------------------------------------------------------

constexpr uint8_t PIN_W12_GERADE    = 2;
constexpr uint8_t PIN_W12_ABBIEGEN  = 3;
constexpr uint8_t PIN_W12_RM_ABBIEG = 37;

constexpr uint8_t PIN_W13_GERADE    = 4;
constexpr uint8_t PIN_W13_ABBIEGEN  = 5;
constexpr uint8_t PIN_W13_RM_ABBIEG = 38;

constexpr uint8_t PIN_W14_GERADE    = 6;
constexpr uint8_t PIN_W14_ABBIEGEN  = 7;
constexpr uint8_t PIN_W14_RM_ABBIEG = 39;

constexpr uint8_t PIN_W15_GERADE    = 8;
constexpr uint8_t PIN_W15_ABBIEGEN  = 9;
constexpr uint8_t PIN_W15_RM_ABBIEG = 40;


// ---------------------------------------------------------
// Kontaktgleise (Blockkontakte)
// ---------------------------------------------------------

constexpr uint8_t PIN_KONTAKT_BLOCK1 = 22;
constexpr uint8_t PIN_KONTAKT_BLOCK2 = 23;
constexpr uint8_t PIN_KONTAKT_BLOCK3 = 24;
constexpr uint8_t PIN_KONTAKT_BLOCK4 = 25;
constexpr uint8_t PIN_KONTAKT_BLOCK5 = 26;
constexpr uint8_t PIN_KONTAKT_BLOCK6 = 30;

constexpr uint8_t PIN_KONTAKT_SBH_GF1 = 27;
constexpr uint8_t PIN_KONTAKT_SBH_GF2 = 28;
constexpr uint8_t PIN_KONTAKT_SBH_GF3 = 29;

constexpr uint8_t PIN_KONTAKT_NOTHALT = 19;


// ---------------------------------------------------------
// Bahnhofskontaktgleise (NEU)
// ---------------------------------------------------------

// Block 2
constexpr uint8_t PIN_KONTAKT_BHF2_A = 14;
constexpr uint8_t PIN_KONTAKT_BHF2_B = 15;

// Block 4
constexpr uint8_t PIN_KONTAKT_BHF4_A = 16;
constexpr uint8_t PIN_KONTAKT_BHF4_B = 11;


// ---------------------------------------------------------
// Schaltgleise S11–S16 (PulseSensor)
// ---------------------------------------------------------

constexpr uint8_t PIN_SCHALTGLEIS_S11 = 31;
constexpr uint8_t PIN_SCHALTGLEIS_S12 = 32;
constexpr uint8_t PIN_SCHALTGLEIS_S13 = 33;
constexpr uint8_t PIN_SCHALTGLEIS_S14 = 34;
constexpr uint8_t PIN_SCHALTGLEIS_S15 = 35;
constexpr uint8_t PIN_SCHALTGLEIS_S16 = 36;


// ---------------------------------------------------------
// Stromrelais (low-aktiv)
// ---------------------------------------------------------

constexpr uint8_t PIN_RELAY_NOTHALT         = 52;

constexpr uint8_t PIN_RELAY_SBH_GL1_NACH6   = 49;
constexpr uint8_t PIN_RELAY_SBH_GL2_NACH6   = 50;
constexpr uint8_t PIN_RELAY_SBH_GL3_NACH6   = 51;

constexpr uint8_t PIN_RELAY_BLOCK1_NACH2    = 43;
constexpr uint8_t PIN_RELAY_BLOCK2_NACH3    = 44;
constexpr uint8_t PIN_RELAY_BLOCK3_NACH4    = 45;
constexpr uint8_t PIN_RELAY_BLOCK4_NACH1    = 46;
constexpr uint8_t PIN_RELAY_BLOCK4_NACH5    = 47;
constexpr uint8_t PIN_RELAY_BLOCK5_NACH_SBH = 48;
constexpr uint8_t PIN_RELAY_BLOCK6_NACH4    = 53;


// ---------------------------------------------------------
// ZMPT101B Trafospannungssensoren
// ---------------------------------------------------------

constexpr uint8_t PIN_ADC_TRAFO_OBEN  = A9;
constexpr uint8_t PIN_ADC_TRAFO_UNTEN = A10;


// ---------------------------------------------------------
// Stromsensoren (ACS…)
// ---------------------------------------------------------

constexpr uint8_t PIN_ADC_BLOCK1  = A0;
constexpr uint8_t PIN_ADC_BLOCK2  = A1;
constexpr uint8_t PIN_ADC_BLOCK3  = A2;
constexpr uint8_t PIN_ADC_BLOCK4  = A3;
constexpr uint8_t PIN_ADC_BLOCK5  = A4;
constexpr uint8_t PIN_ADC_BLOCK6  = A8;

constexpr uint8_t PIN_ADC_SBH_GL1 = A5;
constexpr uint8_t PIN_ADC_SBH_GL2 = A6;
constexpr uint8_t PIN_ADC_SBH_GL3 = A7;


// ---------------------------------------------------------
// Globaler Nothalt (Trafo-Abschaltrelais, low aktiv)
// ---------------------------------------------------------

constexpr uint8_t PIN_RELAY_TRAFO_OBEN_CUT  = 41;
constexpr uint8_t PIN_RELAY_TRAFO_UNTEN_CUT = 42;


// ---------------------------------------------------------
// DataReady-Pin Mega2 → ESP32 (NEU)
// ---------------------------------------------------------

constexpr uint8_t PIN_DATA_READY_M2 = 12;
