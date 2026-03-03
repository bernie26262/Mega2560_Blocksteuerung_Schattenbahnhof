#pragma once

#include <Arduino.h>

// -----------------------------------------------------------------------------
// ADC Scheduler (Mega2560)
//
// Motivation:
// - We need deterministic sampling independent from loop() jitter (I2C/Serial).
// - The Arduino analogRead() API is blocking and shares the single ADC.
// - Therefore we run the ADC from a timer-driven ISR and provide per-channel
//   sample queues that loop() can consume.
//
// Design:
// - Timer1 CTC tick starts one ADC conversion per tick.
// - A fixed schedule table selects the next analog pin per tick.
// - ADC ISR stores samples into a small per-channel ring buffer.
// - loop() consumes samples via adcSchedPop(pin,...).
//
// Notes:
// - The schedule table contains Arduino analog pin constants (A0..A15).
// - This module must be the only ADC user. Do not call analogRead() elsewhere.
// -----------------------------------------------------------------------------

void adcSchedBegin(const uint8_t* schedulePins, uint8_t scheduleLen);

// Optional ISR sink called from ADC ISR for every sample.
// Must be ISR-safe (no Serial, no floats, keep it fast).
using AdcIsrSink = void(*)(uint8_t channel, uint16_t value);
void adcSchedSetIsrSink(AdcIsrSink sink);

// Pop one queued sample for the given analog pin.
// Returns true if a sample was returned.
bool adcSchedPop(uint8_t analogPin, uint16_t& out);

// Disable/enable queueing for a channel. Useful if a channel is fully consumed
// in the ISR sink (e.g. trafo min/max), to avoid useless ring drops.
void adcSchedSetQueueEnabled(uint8_t analogPin, bool enabled);

// Stats
uint32_t adcSchedTicks();
uint32_t adcSchedSamples();
uint16_t adcSchedDropped(uint8_t analogPin);