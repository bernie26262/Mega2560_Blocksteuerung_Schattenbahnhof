#pragma once
#include <Arduino.h>
#include "BlockController.h"
#include "Weiche.h"
#include "PulseSensor.h"
#include "PowerControl.h"

enum class SBhfMode : uint8_t {
    Serial = 1,
    Random = 2
};

enum class SBhfState : uint8_t {
    Idle        = 0,
    CycleActive = 10,
    Blocked     = 11,
    Error       = 12
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

    SBhfMode mode() const { return m_mode; }
    void setMode(SBhfMode m) { m_mode = m; }

    SBhfState state() const { return m_state; }

    uint8_t exitGleis() const { return m_exitGleis; }
    uint8_t targetGleis() const { return m_targetGleis; }

    bool nothaltAktiv() const { return m_nothaltAktiv; }

private:
    BlockController* m_bc;

    Weiche* m_w12;
    Weiche* m_w13;
    Weiche* m_w14;
    Weiche* m_w15;

    PowerControl* m_power;

    PulseSensor* m_s11;
    PulseSensor* m_s12;
    PulseSensor* m_s13;
    PulseSensor* m_s14;
    PulseSensor* m_s15;
    PulseSensor* m_s16;

    SBhfMode  m_mode  = SBhfMode::Serial;
    SBhfState m_state = SBhfState::Idle;

    uint8_t m_counter = 0;
    uint8_t m_exitGleis = 0;
    uint8_t m_targetGleis = 0;

    bool m_cycleActive = false;
    bool m_nothaltAktiv = false;
    bool m_error = false;

    void handleNothalt();
    void handleNewArrival();
    void handleEntrySensors();
    void handleS15ForEntry();

    void chooseGleis();
    void startExitAndPrepareEntry();

    void applyEntryWeichen();
    void applyExitWeichen();

    bool checkForRealErrorCondition();
};
