#pragma once

#include <Arduino.h>

class Block;

class BlockController
{
public:
    BlockController() :
        m_blocks(nullptr),
        m_count(0)
    {}

    explicit BlockController(Block** blocks, uint8_t count) :
        m_blocks(blocks),
        m_count(count)
    {}

    // --------------------------------------------------
    // Laufzeit
    // --------------------------------------------------
    void update(uint32_t nowMs);

    uint8_t count() const { return m_count; }

    // --------------------------------------------------
    // Status-API (produktiv)
    // --------------------------------------------------
    bool     isOccupied(uint8_t id) const;
    uint16_t stromFiltered(uint8_t id) const;
    bool     stromOverThreshold(uint8_t id) const;
    bool     canEnter(uint8_t fromBlock, uint8_t toBlock) const;

    // --------------------------------------------------
    // Debug-API (Simulation, no-op wenn MEGA2_DEBUG=0)
    // --------------------------------------------------
    void debugSetOccupied(uint8_t id, bool occ);
    void debugSetStrom(uint8_t id, bool active);

private:
    Block**  m_blocks;
    uint8_t m_count;
};
