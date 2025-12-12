#include "BlockController.h"
#include "Block.h"

// Glättungsfaktor für EMA
static constexpr float STROM_ALPHA = 0.2f;

void BlockController::update(uint32_t /*nowMs*/)
{
    // bewusst leer
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
    if (!m_blocks || id >= m_count || !m_blocks[id] || !m_stromFiltered)
        return 0;

    uint16_t raw = m_blocks[id]->stromRaw();
    uint16_t& filt = m_stromFiltered[id];

    // EMA: filt = filt + alpha * (raw - filt)
    filt = filt + (uint16_t)(
        STROM_ALPHA * ((int32_t)raw - (int32_t)filt)
    );

    return filt;
}
