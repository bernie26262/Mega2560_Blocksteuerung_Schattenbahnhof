#include <Arduino.h>
#include "BlockController.h"
#include "Block.h"

// --------------------------------------------------
// Update (noch leer)
// --------------------------------------------------

void BlockController::update(uint32_t /*nowMs*/)
{
    // bewusst leer
}

// --------------------------------------------------
// Status
// --------------------------------------------------

bool BlockController::isOccupied(uint8_t id) const
{
    if (!m_blocks || id >= m_count || !m_blocks[id])
        return false;

    return m_blocks[id]->besetzt();
}

uint16_t BlockController::stromFiltered(uint8_t /*id*/) const
{
    return 0;
}

bool BlockController::stromOverThreshold(uint8_t /*id*/) const
{
    return false;
}

// --------------------------------------------------
// B4.2: zentrale Fahrfreigabe
// --------------------------------------------------

bool BlockController::canEnter(uint8_t from, uint8_t to) const
{
    if (!m_blocks || from >= m_count || to >= m_count)
        return false;

    Block* fromBlock = m_blocks[from];
    Block* toBlock   = m_blocks[to];

    if (!fromBlock || !toBlock)
        return false;

    uint32_t now = millis();

    // 🔴 B4.2: zeitlich stabile Freigabe
    if (!toBlock->isReallyFree(now))
        return false;

    // --------------------------------------------------
    // Sonderfall: Zielblock 4
    // --------------------------------------------------
    if (to == 4)
    {
        uint8_t occupied123 = 0;
        for (uint8_t i = 1; i <= 3; i++)
        {
            if (m_blocks[i] && m_blocks[i]->besetzt())
                occupied123++;
        }

        // von Block 6
        if (from == 6)
        {
            return (occupied123 <= 2);
        }

        // von Block 3
        if (from == 3)
        {
            bool block6Occupied = (m_blocks[6] && m_blocks[6]->besetzt());

            if (!block6Occupied)
                return true;

            return (occupied123 > 2);
        }

        return false;
    }

    // --------------------------------------------------
    // Standardregeln
    // --------------------------------------------------

    if (from == 1 && to == 2) return true;
    if (from == 2 && to == 3) return true;
    if (from == 4 && to == 5) return true;
    if (from == 5 && (to == 7 || to == 8 || to == 9)) return true;
    if ((from == 7 || from == 8 || from == 9) && to == 6) return true;

    return false;
}
