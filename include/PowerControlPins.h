#pragma once
#include <Arduino.h>
#include "mega2_pins.h"

// Einzige Klasse, die ALLE Relais des Mega2 steuert.
// Alle Relais sind LOW-AKTIV.

class PowerControlPins {
public:

    void begin() {
        // BLOCK-Relais
        pinMode(PIN_RELAY_BLOCK1_NACH2, OUTPUT);
        pinMode(PIN_RELAY_BLOCK2_NACH3, OUTPUT);
        pinMode(PIN_RELAY_BLOCK3_NACH4, OUTPUT);
        pinMode(PIN_RELAY_BLOCK4_NACH1, OUTPUT);
        pinMode(PIN_RELAY_BLOCK4_NACH5, OUTPUT);
        pinMode(PIN_RELAY_BLOCK5_NACH_SBH, OUTPUT);
        pinMode(PIN_RELAY_BLOCK6_NACH4, OUTPUT);

        // SBhf-Gleis-Relais
        pinMode(PIN_RELAY_SBH_GL1_NACH6, OUTPUT);
        pinMode(PIN_RELAY_SBH_GL2_NACH6, OUTPUT);
        pinMode(PIN_RELAY_SBH_GL3_NACH6, OUTPUT);

        // Nothalt
        pinMode(PIN_RELAY_NOTHALT, OUTPUT);

        // Globale Trafo-Trennrelais
        pinMode(PIN_RELAY_TRAFO_OBEN_CUT,  OUTPUT);
        pinMode(PIN_RELAY_TRAFO_UNTEN_CUT, OUTPUT);

        allOff();
        trafosOn();
    }

    // LOW = aktiv
    void setRelay(uint8_t pin, bool on) {
        if (pin != 255)
            digitalWrite(pin, on ? LOW : HIGH);
    }

    // Alle Block- und SBhf-Relais AUS
    void allOff() {
        digitalWrite(PIN_RELAY_BLOCK1_NACH2,   HIGH);
        digitalWrite(PIN_RELAY_BLOCK2_NACH3,   HIGH);
        digitalWrite(PIN_RELAY_BLOCK3_NACH4,   HIGH);
        digitalWrite(PIN_RELAY_BLOCK4_NACH1,   HIGH);
        digitalWrite(PIN_RELAY_BLOCK4_NACH5,   HIGH);
        digitalWrite(PIN_RELAY_BLOCK5_NACH_SBH,HIGH);
        digitalWrite(PIN_RELAY_BLOCK6_NACH4,   HIGH);

        digitalWrite(PIN_RELAY_SBH_GL1_NACH6, HIGH);
        digitalWrite(PIN_RELAY_SBH_GL2_NACH6, HIGH);
        digitalWrite(PIN_RELAY_SBH_GL3_NACH6, HIGH);

        digitalWrite(PIN_RELAY_NOTHALT, HIGH);
    }

    // Not-Aus: beide Trafos AUS
    void trafosOff() {
        digitalWrite(PIN_RELAY_TRAFO_OBEN_CUT,  LOW);
        digitalWrite(PIN_RELAY_TRAFO_UNTEN_CUT, LOW);
    }

    // Normalbetrieb: beide Trafos AN
    void trafosOn() {
        digitalWrite(PIN_RELAY_TRAFO_OBEN_CUT,  HIGH);
        digitalWrite(PIN_RELAY_TRAFO_UNTEN_CUT, HIGH);
    }
};
