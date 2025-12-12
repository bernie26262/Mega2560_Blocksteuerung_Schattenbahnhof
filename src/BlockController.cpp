#include "BlockController.h"
#include "Block.h"   // ← WICHTIG: nur im .cpp!

void BlockController::update(uint32_t nowMs)
{
    
}

bool BlockController::isOccupied(uint8_t id) const
{
    if (!m_blocks || id >= m_count || !m_blocks[id])
        return false;

    // Block entscheidet selbst (Kontaktgleis + Strom)
    return m_blocks[id]->besetzt();
}

uint16_t BlockController::stromFiltered(uint8_t id) const
{
    if (!m_blocks || id >= m_count || !m_blocks[id])
        return 0;

    // vorerst Rohwert, Filter kommt später (B1.1)
    return m_blocks[id]->stromRaw();
}