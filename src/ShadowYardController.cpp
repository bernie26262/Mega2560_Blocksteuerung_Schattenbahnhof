#include "ShadowYardController.h"
#include "BlockController.h"


ShadowYardController::ShadowYardController(BlockController* bc)
: m_bc(bc)
{
}

void ShadowYardController::begin()
{
    m_state = SBhfState::Idle;
    m_nothalt = false;
}

void ShadowYardController::update(uint32_t /*nowMs*/)
{
    rebuildMasks();

    if (isBlock6Blocking())
    {
        m_state = SBhfState::Idle;
        return;
    }

    chooseGleis();
}

void ShadowYardController::rebuildMasks()
{
    // aktuell leer – vorbereitet für Statusbildung
}

bool ShadowYardController::isBlock6Blocking() const
{
    return m_bc && m_bc->isOccupied(6);
}

void ShadowYardController::chooseGleis()
{
    // Schattenbahnhof: Block 7–9
    for (uint8_t i = 7; i <= 9; i++)
    {
        if (!m_bc->isOccupied(i)) {
            m_targetGleis = i;
        }
    }
}
