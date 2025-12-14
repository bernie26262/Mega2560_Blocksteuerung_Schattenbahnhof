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
