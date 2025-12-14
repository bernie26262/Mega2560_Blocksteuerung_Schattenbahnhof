#include <Arduino.h>
#include "BlockController.h"
#include "Block.h"

// --------------------------------------------------
// Implementation
// --------------------------------------------------

void BlockController::update(uint32_t /*nowMs*/)
{
    // B1: bewusst leer
}

bool BlockController::isOccupied(uint8_t id) const
{
    if (!m_blocks || id >= m_count || !m_blocks[id])
        return false;

    // Block entscheidet selbst (Kontaktgleis + interne Logik)
    return m_blocks[id]->besetzt();
}

uint16_t BlockController::stromFiltered(uint8_t /*id*/) const
{
    // B1: keine Stromanzeige im Controller
    // Stromlogik ist vollständig im Block gekapselt
    return 0;
}

bool BlockController::stromOverThreshold(uint8_t /*id*/) const
{
    // B1: bewusst immer false
    // Block::besetzt() berücksichtigt Strom bereits
    return false;
}

// --------------------------------------------------
// Block-Logik: kann der Zug einfahren? (B2)
// --------------------------------------------------

bool BlockController::canEnter(uint8_t from, uint8_t to) const
{
    // --- Grundchecks ---
    if (!m_blocks || from >= m_count || to >= m_count)
        return false;

    Block* fromBlock = m_blocks[from];
    Block* toBlock   = m_blocks[to];

    if (!fromBlock || !toBlock)
        return false;

    // Zielblock muss physisch und zeitlich frei sein
    if (!toBlock->isFreeForEntry())
        return false;

    // --------------------------------------------------
    // Sonderfall: Zielblock 4
    // --------------------------------------------------
    if (to == 4)
    {
        // Block 4 selbst darf nicht besetzt sein
        if (toBlock->besetzt())
            return false;

        // Anzahl besetzter Blöcke 1..3 zählen
        uint8_t occupied123 = 0;
        for (uint8_t i = 1; i <= 3; i++)
        {
            if (m_blocks[i] && m_blocks[i]->besetzt())
                occupied123++;
        }

        // --- von Block 6 ---
        if (from == 6)
        {
            // erlaubt, wenn nicht mehr als 2 Züge in 1..3
            return (occupied123 <= 2);
        }

        // --- von Block 3 ---
        if (from == 3)
        {
            bool block6Occupied = (m_blocks[6] && m_blocks[6]->besetzt());

            // Block 6 frei → immer erlaubt
            if (!block6Occupied)
                return true;

            // Block 6 besetzt → nur wenn mehr als 2 Züge in 1..3
            return (occupied123 > 2);
        }

        // alle anderen → verboten
        return false;
    }

    // --------------------------------------------------
    // Standardregeln
    // --------------------------------------------------

    // 1 -> 2
    if (from == 1 && to == 2)
        return true;

    // 2 -> 3
    if (from == 2 && to == 3)
        return true;

    // 4 -> 5
    if (from == 4 && to == 5)
        return true;

    // 5 -> SBhf (7,8,9)
    if (from == 5 && (to == 7 || to == 8 || to == 9))
        return true;

    // SBhf -> 6
    if ((from == 7 || from == 8 || from == 9) && to == 6)
        return true;

    // sonst nicht erlaubt
    return false;
}
