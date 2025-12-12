#include "ShadowYardController.h"
#include "BlockController.h"
#include "Block.h"

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
    Block* b6 = m_bc->block(6);
    return (b6 && b6->besetzt());
}

void ShadowYardController::chooseGleis()
{
    // Schattenbahnhof: Block 7–9
    for (uint8_t i = 7; i <= 9; i++)
    {
        Block* b = m_bc->block(i);
        if (b && !b->besetzt())
        {
            m_targetGleis = i;
            return;
        }
    }
}
