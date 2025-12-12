#pragma once
#include <Arduino.h>

class BlockController;

enum class SBhfState : uint8_t
{
    Idle,
    Entry,
    Exit,
    Error
};

class ShadowYardController
{
public:
    ShadowYardController() : m_bc(nullptr) {}
    explicit ShadowYardController(BlockController* bc);

    void begin();
    void update(uint32_t nowMs);

    SBhfState state() const { return m_state; }
    bool nothaltAktiv() const { return m_nothalt; }

    uint8_t exitGleis() const   { return m_exitGleis; }
    uint8_t targetGleis() const { return m_targetGleis; }

private:
    void rebuildMasks();
    bool isBlock6Blocking() const;
    void chooseGleis();

    BlockController* m_bc;

    SBhfState m_state = SBhfState::Idle;
    bool      m_nothalt = false;

    uint8_t   m_exitGleis   = 0;
    uint8_t   m_targetGleis = 0;
};
