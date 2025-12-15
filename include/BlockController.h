#pragma once

#include <Arduino.h>

class Block;

class BlockController
{
public:
    BlockController(Block** blocks, uint8_t count);

    void update(uint32_t nowMs);

    uint8_t count() const { return m_count; }

    // Status
    bool     isOccupied(uint8_t id) const;
    uint16_t stromFiltered(uint8_t id) const;
    bool     stromOverThreshold(uint8_t id) const;

    // Blockfreigabe
    bool canEnter(uint8_t fromBlock, uint8_t toBlock) const;

#if MEGA2_DEBUG
    // Debug-Override (nur Block 1..9)
    void debugSetOccupied(uint8_t id, bool occ);
    void debugSetStrom(uint8_t id, bool active);
    void debugClear(uint8_t id);
#endif

private:
    Block**  m_blocks;
    uint8_t  m_count;

    // normale Stromlogik (bestehend)
    uint16_t* m_stromFiltered;
    bool*     m_stromActive;

#if MEGA2_DEBUG
    // Debug-Zustand
    bool     m_dbgActive[16];
    bool     m_dbgOcc[16];
    bool     m_dbgStrom[16];
    uint32_t m_dbgLastFreeMs[16];
#endif
};
