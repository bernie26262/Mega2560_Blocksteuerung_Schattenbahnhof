#include <Arduino.h>
#include "BlockController.h"
#include "Block.h"
#include "mega2_debug.h"

// --------------------------------------------------
// Update aller Blöcke
// --------------------------------------------------
void BlockController::update(uint32_t nowMs)
{
    for (uint8_t i = 0; i < m_count; i++)
    {
        if (m_blocks[i])
            m_blocks[i]->update(nowMs);
    }
}

// --------------------------------------------------
// Besetzt-Abfrage
// --------------------------------------------------
bool BlockController::isOccupied(uint8_t id) const
{
    if (!m_blocks || id >= m_count || !m_blocks[id])
        return false;

    return m_blocks[id]->besetzt();
}

// --------------------------------------------------
// Darf von -> nach eingefahren werden?
// --------------------------------------------------
bool BlockController::canEnter(uint8_t from, uint8_t to) const
{
    if (!m_blocks || from >= m_count || to >= m_count)
    {
        DBG_PRINT("[BC] ");
        DBG_PRINT(from);
        DBG_PRINT(" -> ");
        DBG_PRINT(to);
        DBG_PRINTLN(" FAIL (invalid index)");
        return false;
    }

    Block* fromBlock = m_blocks[from];
    Block* toBlock   = m_blocks[to];
    if (!fromBlock || !toBlock)
    {
        DBG_PRINT("[BC] ");
        DBG_PRINT(from);
        DBG_PRINT(" -> ");
        DBG_PRINT(to);
        DBG_PRINTLN(" FAIL (null block)");
        return false;
    }

    uint32_t now = millis();

    // 🔒 Block muss zeitstabil frei sein
    if (!toBlock->isReallyFree(now))
    {
        DBG_PRINT("[BC] ");
        DBG_PRINT(from);
        DBG_PRINT(" -> ");
        DBG_PRINT(to);
        DBG_PRINTLN(" FAIL (not stable free)");
        return false;
    }

    // ==================================================
    // Sonderlogik BLOCK 4
    // ==================================================
    if (to == 4)
    {
        uint8_t occ123 = 0;
        for (uint8_t i = 1; i <= 3; i++)
            if (m_blocks[i] && m_blocks[i]->besetzt())
                occ123++;

        // 6 -> 4
        if (from == 6)
        {
            DBG_PRINT("[BC] 6 -> 4 occ123=");
            DBG_PRINTLN(occ123);

            if (occ123 <= 2)
            {
                DBG_PRINTLN("[BC] 6 -> 4 OK");
                return true;
            }

            DBG_PRINTLN("[BC] 6 -> 4 FAIL (occ123 > 2)");
            return false;
        }

        // 3 -> 4
        if (from == 3)
        {
            bool block6Occ = m_blocks[6] && m_blocks[6]->besetzt();

            DBG_PRINT("[BC] 3 -> 4 occ123=");
            DBG_PRINT(occ123);
            DBG_PRINT(" block6=");
            DBG_PRINTLN(block6Occ);

            if (!block6Occ)
            {
                DBG_PRINTLN("[BC] 3 -> 4 OK (block6 frei)");
                return true;
            }

            if (occ123 > 2)
            {
                DBG_PRINTLN("[BC] 3 -> 4 OK (block6 besetzt, occ123 > 2)");
                return true;
            }

            DBG_PRINTLN("[BC] 3 -> 4 FAIL");
            return false;
        }

        DBG_PRINTLN("[BC] -> 4 FAIL (from block not allowed)");
        return false;
    }

    // ==================================================
    // Standardfreigaben
    // ==================================================
    if (from == 1 && to == 2)
    {
        DBG_PRINTLN("[BC] 1 -> 2 OK");
        return true;
    }

    if (from == 2 && to == 3)
    {
        DBG_PRINTLN("[BC] 2 -> 3 OK");
        return true;
    }

    if (from == 4 && to == 5)
    {
        DBG_PRINTLN("[BC] 4 -> 5 OK");
        return true;
    }

    if (from == 5 && (to == 7 || to == 8 || to == 9))
    {
        DBG_PRINT("[BC] 5 -> ");
        DBG_PRINT(to);
        DBG_PRINTLN(" OK (SBhf)");
        return true;
    }

    if ((from == 7 || from == 8 || from == 9) && to == 6)
    {
        DBG_PRINT("[BC] ");
        DBG_PRINT(from);
        DBG_PRINTLN(" -> 6 OK");
        return true;
    }

    DBG_PRINT("[BC] ");
    DBG_PRINT(from);
    DBG_PRINT(" -> ");
    DBG_PRINT(to);
    DBG_PRINTLN(" FAIL (no rule)");

    return false;
}

// --------------------------------------------------
// Stromanzeige (Debug / Status)
// --------------------------------------------------
uint16_t BlockController::stromFiltered(uint8_t id) const
{
    if (!m_blocks || id >= m_count || !m_blocks[id])
        return 0;

    // Debug-Phase:
    // echte Stromlogik liegt im Block (SensorStrom)
    // hier nur Anzeige / I2C / Diagnose
    return m_blocks[id]->stromAktiv() ? 1 : 0;
}
