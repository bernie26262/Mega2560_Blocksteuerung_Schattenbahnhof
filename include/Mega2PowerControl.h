#pragma once
#include <Arduino.h>
#include <util/atomic.h>
#include "mega2_pins.h"
#include "PowerControl.h"

class Mega2PowerControl : public PowerControl {
public:
    // NOTE: PowerControl has no begin(); this is Mega2-specific init.
    void begin() {
        // Global Power-Relais: sicherer AUS-Zustand = HIGH
        // Glitch-sicher: erst Pegel setzen, dann OUTPUT aktivieren.
        digitalWrite(PIN_RELAY_TRAFO_OBEN_CUT,  HIGH);
        digitalWrite(PIN_RELAY_TRAFO_UNTEN_CUT, HIGH);
        pinMode(PIN_RELAY_TRAFO_OBEN_CUT,  OUTPUT);
        pinMode(PIN_RELAY_TRAFO_UNTEN_CUT, OUTPUT);

        // Stromrelais (low-aktiv): IMMER als OUTPUT initialisieren.
        // Wichtig: Ohne OUTPUT bleibt digitalWrite() nur Pull-Up-Steuerung (Input-mode)
        // und kann bei Relaisboards zu "Geisterschalten"/Kopplungen führen.
        initRelayPinLowActive(PIN_RELAY_BLOCK1_NACH2);
        initRelayPinLowActive(PIN_RELAY_BLOCK2_NACH3);
        initRelayPinLowActive(PIN_RELAY_BLOCK3_NACH4);
        initRelayPinLowActive(PIN_RELAY_BLOCK4_NACH1);
        initRelayPinLowActive(PIN_RELAY_BLOCK4_NACH5);
        initRelayPinLowActive(PIN_RELAY_BLOCK5_NACH_SBH);
        initRelayPinLowActive(PIN_RELAY_BLOCK6_NACH4);
        initRelayPinLowActive(PIN_RELAY_SBH_GL1_NACH6);
        initRelayPinLowActive(PIN_RELAY_SBH_GL2_NACH6);
        initRelayPinLowActive(PIN_RELAY_SBH_GL3_NACH6);
        initRelayPinLowActive(PIN_RELAY_NOTHALT);

        // Default: Boot-sicher → Power OFF (Cut aktiv)
        setMainPower(false);
    }

    void setBlock1To2(bool on) {
        digitalWrite(PIN_RELAY_BLOCK1_NACH2, on ? LOW : HIGH);
    }

    void setBlock2To3(bool on) {
        digitalWrite(PIN_RELAY_BLOCK2_NACH3, on ? LOW : HIGH);
    }

    void setBlock3To4(bool on) {
        digitalWrite(PIN_RELAY_BLOCK3_NACH4, on ? LOW : HIGH);
    }

    void setBlock4To1(bool on) {
        digitalWrite(PIN_RELAY_BLOCK4_NACH1, on ? LOW : HIGH);
    }

    void setBlock4To5(bool on) {
        digitalWrite(PIN_RELAY_BLOCK4_NACH5, on ? LOW : HIGH);
    }

    void setBlock6To4(bool on) {
        digitalWrite(PIN_RELAY_BLOCK6_NACH4, on ? LOW : HIGH);
    }

    void setBlock5ToSBhf(bool on) override {
        m_block5ToSbhfActive = on;
        digitalWrite(PIN_RELAY_BLOCK5_NACH_SBH, on ? LOW : HIGH);
    }

    bool isBlock5ToSBhfActive() const override {
        return m_block5ToSbhfActive;
    }

    void setSbhfGleis(uint8_t gleis, bool on) override {
        uint8_t pin = 0;
        if (gleis == 1) pin = PIN_RELAY_SBH_GL1_NACH6;
        if (gleis == 2) pin = PIN_RELAY_SBH_GL2_NACH6;
        if (gleis == 3) pin = PIN_RELAY_SBH_GL3_NACH6;
        if (pin) digitalWrite(pin, on ? LOW : HIGH);
    }

    void setNothalt(bool on) override {
        m_nothaltActive = on;
        digitalWrite(PIN_RELAY_NOTHALT, on ? LOW : HIGH);
    }

    bool isNothaltActive() const {
        return m_nothaltActive;
    }

    // ------------------------------------------------------------
    // Safety / SSR API (für safety.cpp)
    // ------------------------------------------------------------
    void setMainPower(bool on)
    {
        m_mainPowerOn = on;

        // "Main" bedeutet bei Mega2: beide Trafos freigeben/sperren
        setSsrTrafoA(on);
        setSsrTrafoB(on);
    }

    bool isMainPowerOn() const { return m_mainPowerOn; }

    // SSR_TRAFO_A = Trafo oben (CUT Relais)
    void setSsrTrafoA(bool enable)
    {
        m_ssrTrafoAEnabled = enable;
        digitalWrite(PIN_RELAY_TRAFO_OBEN_CUT, enable ? LOW : HIGH);
    }

    // SSR_TRAFO_B = Trafo unten (CUT Relais)
    void setSsrTrafoB(bool enable)
    {
        m_ssrTrafoBEnabled = enable;
        digitalWrite(PIN_RELAY_TRAFO_UNTEN_CUT, enable ? LOW : HIGH);
    }

    bool isSsrTrafoA() const { return m_ssrTrafoAEnabled; }
    bool isSsrTrafoB() const { return m_ssrTrafoBEnabled; }
    
private:
    static inline void initRelayPinLowActive(uint8_t pin) {
        // atomic nicht zwingend nötig, aber verhindert RMW-Kollisionen falls später ISR/Parallelzugriffe dazu kommen
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            // Glitch-sicher: zuerst Portwert setzen, dann OUTPUT aktivieren.
            digitalWrite(pin, HIGH); // default OFF (low-active)
            pinMode(pin, OUTPUT);
        }
    }

    bool m_nothaltActive = false;
    bool m_mainPowerOn = false;
    bool m_block5ToSbhfActive = false;
    bool m_ssrTrafoAEnabled = false;
    bool m_ssrTrafoBEnabled = false;
};
