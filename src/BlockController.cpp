#include "BlockController.h"
#include "Block.h"

// --------------------------------------------------
// Parameter
// --------------------------------------------------

// EMA-Glättungsfaktor
// 0.1 = sehr träge, 0.3 = sehr schnell
static constexpr float STROM_ALPHA = 0.2f;

// Strom-Schwellwerte (mA) mit Hysterese
static constexpr uint16_t STROM_ON_MA  = 60;
static constexpr uint16_t STROM_OFF_MA = 40;

// --------------------------------------------------
// Implementation
// --------------------------------------------------

void BlockController::update(uint32_t /*nowMs*/)
{
    // bewusst leer
    // spätere Sammel- oder Zeitlogik möglich
}

bool BlockController::isOccupied(uint8_t id) const
{
    if (!m_blocks || id >= m_count || !m_blocks[id])
        return false;

    // Block entscheidet selbst (Kontaktgleis + interne Logik)
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

bool BlockController::stromOverThreshold(uint8_t id) const
{
    if (!m_stromFiltered || !m_stromActive || id >= m_count)
        return false;

    uint16_t value = m_stromFiltered[id];
    bool& active   = m_stromActive[id];

    if (!active && value >= STROM_ON_MA)
    {
        active = true;
    }
    else if (active && value <= STROM_OFF_MA)
    {
        active = false;
    }

    return active;
}
