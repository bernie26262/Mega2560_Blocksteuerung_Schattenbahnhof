#pragma once

#include <Arduino.h>

class Block;   // Forward Declaration – Block bleibt gekapselt

class BlockController
{
public:
    // Default-Konstruktor (z. B. Platzhalter)
    BlockController() :
        m_blocks(nullptr),
        m_count(0),
        m_stromFiltered(nullptr),
        m_stromActive(nullptr)
    {}

    // Konstruktor mit Block-Array
    explicit BlockController(Block** blocks, uint8_t count) :
        m_blocks(blocks),
        m_count(count)
    {
        // EMA-Stromwerte (Start = 0)
        m_stromFiltered = new uint16_t[count]();

        // Hysterese-Zustand (Start = false)
        m_stromActive   = new bool[count]();
    }

    // Zyklisches Update (derzeit bewusst leer)
    void update(uint32_t nowMs);

    uint8_t count() const { return m_count; }

    // ---------------------------------
    // Status-API
    // ---------------------------------
    bool     isOccupied(uint8_t id) const;        // Kontaktgleis (+ evtl. interne Logik)
    uint16_t stromFiltered(uint8_t id) const;     // EMA-gefilterter Strom
    bool     stromOverThreshold(uint8_t id) const;// B1.2: Strom > Schwellwert (Hysterese)

    // B2: Blockfreigabe-Logik
    bool canEnter(uint8_t fromBlock, uint8_t toBlock) const;

private:
    Block**    m_blocks;
    uint8_t   m_count;

    // B1.1: EMA-gefilterte Stromwerte
    uint16_t* m_stromFiltered;

    // B1.2: Hysterese-Zustand pro Block
    bool*     m_stromActive;
};
