#include "BlockController.h"
#include "Block.h"
#include "mega2_debug.h"

// --------------------------------------------------
void BlockController::update(uint32_t nowMs)
{
    for (uint8_t i = 0; i < m_count; i++)
    {
        if (m_blocks[i])
        {
            m_blocks[i]->update(nowMs);
        }
    }
}

// --------------------------------------------------
bool BlockController::isOccupied(uint8_t id) const
{
    if (!m_blocks || id >= m_count || !m_blocks[id])
        return false;

    return m_blocks[id]->besetzt();
}

// --------------------------------------------------
uint16_t BlockController::stromFiltered(uint8_t id) const
{
    if (!m_blocks || id >= m_count || !m_blocks[id])
        return 0;

    return m_blocks[id]->stromAktiv() ? 300 : 0;
}

// --------------------------------------------------
bool BlockController::stromOverThreshold(uint8_t id) const
{
    if (!m_blocks || id >= m_count || !m_blocks[id])
        return false;

    return m_blocks[id]->stromAktiv();
}

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
    if (!toBlock->isReallyFree(now))
        return false;

    if (from == 1 && to == 2) return true;
    if (from == 2 && to == 3) return true;
    if (from == 4 && to == 5) return true;
    if (from == 5 && (to == 7 || to == 8 || to == 9)) return true;
    if ((from == 7 || from == 8 || from == 9) && to == 6) return true;

    return false;
}

// --------------------------------------------------
// Debug-Funktionen
// --------------------------------------------------
void BlockController::debugSetOccupied(uint8_t id, bool occ)
{
#if MEGA2_DEBUG
    if (!m_blocks || id >= m_count || !m_blocks[id])
        return;

    m_blocks[id]->debugSetOccupied(occ);

    DBG_PRINT("[BLOCK] Block ");
    DBG_PRINT(id);
    DBG_PRINTLN(occ ? " -> OCCUPIED" : " -> FREE");
#else
    (void)id;
    (void)occ;
#endif
}

void BlockController::debugSetStrom(uint8_t id, bool active)
{
#if MEGA2_DEBUG
    if (!m_blocks || id >= m_count || !m_blocks[id])
        return;

    m_blocks[id]->debugSetStrom(active);

    DBG_PRINT("[BLOCK] Block ");
    DBG_PRINT(id);
    DBG_PRINTLN(active ? " Strom = ON" : " Strom = OFF");
#else
    (void)id;
    (void)active;
#endif
}
