#pragma once
#include <Arduino.h>
#include "BlockController.h"
#include "Weiche.h"
#include "PulseSensor.h"
#include "PowerControl.h"

// Betriebsmodi
enum class SBhfMode : uint8_t {
    Serial = 1,  // Gleis 1 -> 2 -> 3 -> 1 ...
    Random = 2   // zufälliges belegtes Gleis
};

// Zustände des Schattenbahnhof-Automaten
enum class SBhfState : uint8_t {
    Idle            = 0,   // wartet auf S11

    CycleActive     = 10,  // Ausfahr- + Einfahr-Sequenz läuft
    Blocked         = 11,  // Block 6 war besetzt: Überfüllung / Warnzustand

    ErrorNothalt    = 20   // Zug steht unerlaubt im Nothalt-Bereich
};

class ShadowYardController {
public:
    ShadowYardController(BlockController* bc,
                         Weiche* w12,
                         Weiche* w13,
                         Weiche* w14,
                         Weiche* w15,
                         PowerControl* power,
                         PulseSensor* s11,
                         PulseSensor* s12,
                         PulseSensor* s13,
                         PulseSensor* s14,
                         PulseSensor* s15,
                         PulseSensor* s16);

    void begin();
    void update(uint32_t now);

    // Mode setzen (seriell / random)
    void setMode(SBhfMode mode) { m_mode = mode; }
    SBhfMode mode() const { return m_mode; }

    // Status für I2C / WebUI
    SBhfState state() const { return m_state; }
    uint8_t   targetGleis() const { return m_targetGleis; }
    uint8_t   exitGleis() const { return m_exitGleis; }

    // Hilfsinfos für Statuspaket
    bool nothaltAktiv() const { return m_nothaltAktiv; }
    bool errorNothalt() const { return m_errorNothalt; }

private:
    BlockController* m_bc;
    Weiche*          m_w12;
    Weiche*          m_w13;
    Weiche*          m_w14;
    Weiche*          m_w15;
    PowerControl*    m_power;

    PulseSensor* m_s11;
    PulseSensor* m_s12;
    PulseSensor* m_s13;
    PulseSensor* m_s14;
    PulseSensor* m_s15;
    PulseSensor* m_s16;

    SBhfMode  m_mode  = SBhfMode::Serial;
    SBhfState m_state = SBhfState::Idle;

    uint8_t m_counter     = 0;  // für Serial-Mode 1..3
    uint8_t m_targetGleis = 0;  // Zielgleis Einfahrt (1..3)
    uint8_t m_exitGleis   = 0;  // Ausfahrgleis (1..3)

    bool m_cycleActive   = false;
    bool m_nothaltAktiv  = false;
    bool m_errorNothalt  = false;

    // --- interne Hilfsmethoden ---
    void handleNothalt();          // S15 / S16 + Error-Check
    void handleNewArrival();       // S11
    void handleEntrySensors();     // S12 / S13 / S14
    void handleS15ForEntry();      // S15: Block5->SBhf einschalten

    void chooseGleis();            // wählt m_exitGleis + m_targetGleis
    void startExitAndPrepareEntry();

    void applyEntryWeichen();
    void applyExitWeichen();

    bool isNothaltKontaktAktiv() const;  // -> von dir mit realem Kontakt zu füttern
};
