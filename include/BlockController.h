#pragma once

#include <Arduino.h>   // <-- WICHTIG: uint8_t, uint32_t, nullptr

class Block;           // <-- Forward Declaration (zwingend!)

class BlockController
{
public:
    // Default-Konstruktor für globale Instanzen
    BlockController() : m_blocks(nullptr), m_count(0) {}

    // Konstruktor mit Block-Array
    explicit BlockController(Block** blocks, uint8_t count)
        : m_blocks(blocks), m_count(count) {}

    // Nur Deklaration! (Implementierung in .cpp)
    void update(uint32_t nowMs);

    uint8_t count() const { return m_count; }

    // 🔧 TEMPORÄR für Alt-Code
    Block* block(uint8_t id) const
    {
        return m_blocks ? m_blocks[id] : nullptr;
    }

    // Status-API (Stub, keine Logik)
    bool     isOccupied(uint8_t) const { return false; }
    uint16_t stromFiltered(uint8_t) const { return 0; }

private:
    Block**  m_blocks;
    uint8_t m_count;
};
