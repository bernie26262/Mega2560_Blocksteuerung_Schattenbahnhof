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
// - THR_BLOCK_OCC_MA ist in "ADC RMS Counts" (SensorStrom::rmsCounts()).
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
// Block occupancy via current sensor (mA)
// ------------------------------------------------------------
// Ab wann gilt ein Block als "Strom an / besetzt" über Strommessung?
// Empfehlung: 3–5× Noise-Floor (Peak) im Leerlauf.
static constexpr uint16_t THR_BLOCK_OCC_MA = 250;  // Startwert, nach Noise-Floor anpassen

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
// Calibration factors (fixed point)
// ------------------------------------------------------------
// counts -> mA:  i_mA = (rmsCounts * KI_NUM + KI_DEN/2) / KI_DEN
// Start default is 1:1 so existing behaviour remains usable until calibrated.
static constexpr uint16_t KI_COUNTS_TO_MA_NUM = 1;
static constexpr uint16_t KI_COUNTS_TO_MA_DEN = 1;

// Trafo sensor-domain dV (Vrms*10 from SensorTrafoAC) -> real track dV
// v_dV_real = (v_dV_sensor * KV_NUM + KV_DEN/2) / KV_DEN
// Trafo RMS scaling (ADC-domain Vrms -> Trafo Vrms)
// Derived from multimeter measurements (initial values):
//   TOben :  11.54Vrms / 0.508Vrms ≈ 22.72
//            19.14Vrms / 0.843Vrms ≈ 22.71
//            28.04Vrms / 1.230Vrms ≈ 22.80
//   TUnten:  12.38Vrms / 0.544Vrms ≈ 22.77
//            18.81Vrms / 0.825Vrms ≈ 22.80
//            27.67Vrms / 1.211Vrms ≈ 22.85
// Use a rational factor NUM/DEN to keep it stable across builds.
static constexpr uint16_t KV_TRAFO_OBEN_NUM  = 2276;  // 22.76
static constexpr uint16_t KV_TRAFO_OBEN_DEN  = 100;
static constexpr uint16_t KV_TRAFO_UNTEN_NUM = 2282;  // 22.82
static constexpr uint16_t KV_TRAFO_UNTEN_DEN = 100;

// ------------------------------------------------------------
// Effective thresholds for calibration mode
// ------------------------------------------------------------
// Im Kalibrierbetrieb wollen wir keine False-Trips durch Kurzschluss/DoubleOcc.
// Daher werden die Trigger-Schwellen temporär "unmöglich" hoch gesetzt.
static constexpr uint16_t THR_SHORT_MA_EFF =
    (MEGA2_CALIB_MODE ? 65000u : THR_SHORT_MA);

static constexpr uint16_t THR_DOUBLE_OCC_MA_EFF =
    (MEGA2_CALIB_MODE ? 65000u : THR_DOUBLE_OCC_MA);