#pragma once
#include <Arduino.h>


// ============================================================================
// threshold_values.h
// Zentrale Schwellwerte / Kalibrier-Modus für Mega2
//
// Ziel:
// - Keine "wild" verteilten Thresholds mehr in main.cpp / safety.cpp / ...
// - Ein Ort zum Nachschauen und Justieren.
// - Optional: MEGA2_CALIB_MODE (Build-Flag) zum Entschärfen der Safety-Heuristiken
//
// Hinweis:
// - THR_BLOCK_OCC_COUNTS ist in "ADC RMS Counts" (SensorStrom::rmsCounts()).
// - Safety-Schwellen sind in mA (nach späterer Kalibrierung/Umrechnung).
// ============================================================================

// ------------------------------------------------------------
// Build-time Calibration Mode
// ------------------------------------------------------------
// In PlatformIO z.B.:
//   build_flags = -DMEGA2_CALIB_MODE=1
//
#ifndef MEGA2_CALIB_MODE
#define MEGA2_CALIB_MODE 0
#endif

// ------------------------------------------------------------
// Block occupancy via current sensor (RMS counts)
// ------------------------------------------------------------
// Ab wann gilt ein Block als "Strom an / besetzt" über Strommessung?
// Default bisher war 8 (sehr empfindlich). Für Realanlage/Kalibrierung
// typischerweise höher setzen, z.B. 30..80 je nach Noise-Floor.
static constexpr uint16_t THR_BLOCK_OCC_COUNTS = 80;  // war 40 als Startwert

// ------------------------------------------------------------
// Safety thresholds (mA)  (nach Kalibrierung!)
// ------------------------------------------------------------
static constexpr uint16_t THR_SHORT_MA       = 1800;
static constexpr uint16_t THR_NO_CURRENT_MA  = 100;
static constexpr uint16_t THR_DOUBLE_OCC_MA  = 1200; // TODO: später kalibrieren

// Timing / debouncing for safety heuristics
static constexpr uint32_t THR_SHORT_DETECT_MS     = 200;
static constexpr uint32_t THR_DOUBLE_OCC_DETECT_MS= 600;
static constexpr uint32_t THR_POWER_STABLE_MS     = 400;

// Adaptive Double-Occ (zusätzlich zum Hard-Trigger)
static constexpr uint32_t THR_DOUBLE_OCC_TAU_MS       = 2000; // ~2s EMA
static constexpr uint16_t THR_DOUBLE_OCC_FACTOR_NUM   = 3;    // 1.5x
static constexpr uint16_t THR_DOUBLE_OCC_FACTOR_DEN   = 2;
static constexpr uint16_t THR_DOUBLE_OCC_DELTA_MIN_MA = 120;  // absoluter Sprung
static constexpr uint16_t THR_DOUBLE_OCC_BASE_MIN_MA  = 80;   // Basis muss "echt" sein

// ------------------------------------------------------------
// Effective thresholds for calibration mode
// ------------------------------------------------------------
// Im Kalibrierbetrieb wollen wir keine False-Trips durch Kurzschluss/DoubleOcc.
// Daher werden die Trigger-Schwellen temporär "unmöglich" hoch gesetzt.
static constexpr uint16_t THR_SHORT_MA_EFF =
    (MEGA2_CALIB_MODE ? 65000u : THR_SHORT_MA);

static constexpr uint16_t THR_DOUBLE_OCC_MA_EFF =
    (MEGA2_CALIB_MODE ? 65000u : THR_DOUBLE_OCC_MA);
