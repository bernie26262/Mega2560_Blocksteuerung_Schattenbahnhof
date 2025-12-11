#pragma once
#include <Arduino.h>
#include "mega2_pins.h"
#include "PowerControl.h"

class Mega2PowerControl : public PowerControl {
public:
    void begin() {
        pinMode(PIN_RELAY_BLOCK5_NACH_SBH, OUTPUT);
        pinMode(PIN_RELAY_SBH_GL1_NACH6,   OUTPUT);
        pinMode(PIN_RELAY_SBH_GL2_NACH6,   OUTPUT);
        pinMode(PIN_RELAY_SBH_GL3_NACH6,   OUTPUT);
        pinMode(PIN_RELAY_NOTHALT,         OUTPUT);

        digitalWrite(PIN_RELAY_BLOCK5_NACH_SBH, HIGH);
        digitalWrite(PIN_RELAY_SBH_GL1_NACH6,   HIGH);
        digitalWrite(PIN_RELAY_SBH_GL2_NACH6,   HIGH);
        digitalWrite(PIN_RELAY_SBH_GL3_NACH6,   HIGH);
        digitalWrite(PIN_RELAY_NOTHALT,         HIGH);
    }

    void setBlock5ToSBhf(bool on) override {
        digitalWrite(PIN_RELAY_BLOCK5_NACH_SBH, on ? LOW : HIGH);
    }

    void setSbhfGleis(uint8_t gleis, bool on) override {
        uint8_t pin = 0;
        if (gleis == 1) pin = PIN_RELAY_SBH_GL1_NACH6;
        if (gleis == 2) pin = PIN_RELAY_SBH_GL2_NACH6;
        if (gleis == 3) pin = PIN_RELAY_SBH_GL3_NACH6;
        if (pin) digitalWrite(pin, on ? LOW : HIGH);
    }

    void setNothalt(bool on) override {
        digitalWrite(PIN_RELAY_NOTHALT, on ? LOW : HIGH);
    }
};
