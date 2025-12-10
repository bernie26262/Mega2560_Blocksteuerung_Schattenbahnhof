#pragma once
#include <Arduino.h>
#include "SensorKontakt.h"
#include "PulseSensor.h"
#include "Block.h"
#include "ShadowYardController.h"

class SimulationEngine {
public:
    SimulationEngine() {}

    void begin() {
        Serial.println(F("[SIM] Simulation aktiviert"));
        lastStep = millis();
    }

    // Haupt-Simulationsroutine
    void update(uint32_t now,
                Block** blocks,
                PulseSensor** schalt,
                ShadowYardController* sy)
    {
        if (now - lastStep < SIM_STEP_MS)
            return;

        lastStep = now;
        step++;

        // ========================================================
        // ZYKLUS:
        // 1) S11 → Zug kommt in Block 5
        // 2) S12/S13/S14 → Ausfahrt aus SBhf
        // 3) S15 → Einfahrstrom einschalten
        // 4) S12/S13/S14 → Einfahrt in SBhf
        // ========================================================

        if (step == 1) {
            Serial.println(F("[SIM] S11 → Zug erreicht Block 5"));
            trigger(schalt[11]);
            blocks[5]->setBesetztSim(true);
        }

        if (step == 4) {
            uint8_t g = sy->targetGleis();
            Serial.print(F("[SIM] Ausfahrt erreicht (S"));
            Serial.print(11 + g);
            Serial.println(")");
            trigger(schalt[11 + g]);
            blocks[6]->setBesetztSim(true);
            blocks[5]->setBesetztSim(false);
        }

        if (step == 6) {
            Serial.println(F("[SIM] Block 6 frei"));
            blocks[6]->setBesetztSim(false);
        }

        if (step == 7) {
            Serial.println(F("[SIM] S15 → Einfahrstrom EIN"));
            trigger(schalt[15]);
        }

        if (step == 10) {
            uint8_t g = sy->targetGleis();
            Serial.print(F("[SIM] Einfahrt SBhf Gleis "));
            Serial.println(g);

            trigger(schalt[11 + g]);
            blocks[6 + g]->setBesetztSim(true);
        }

        if (step >= 14) {
            Serial.println(F("[SIM] Zyklus beendet, Neustart"));
            step = 0;
        }
    }

private:
    const uint16_t SIM_STEP_MS = 800;
    uint32_t lastStep = 0;
    uint8_t step = 0;

    void trigger(PulseSensor* ps) {
        ps->simulateLow();
        delay(20);
        ps->simulateHigh();
    }
};
