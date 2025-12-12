#pragma once
#include <Arduino.h>

class Block;   // ✅ MUSS bleiben (für Block**)

class BlockController
{
public:
    BlockController() : m_blocks(nullptr), m_count(0) {}

    explicit BlockController(Block** blocks, uint8_t count)
        : m_blocks(blocks), m_count(count) {}

    void update(uint32_t nowMs);

    uint8_t count() const { return m_count; }

    // Status-API (Stub, Logik folgt später)
    bool     isOccupied(uint8_t) const { return false; }
    uint16_t stromFiltered(uint8_t) const { return 0; }

private:
    Block**  m_blocks;   // interner Besitz, kein Zugriff von außen
    uint8_t m_count;
};
