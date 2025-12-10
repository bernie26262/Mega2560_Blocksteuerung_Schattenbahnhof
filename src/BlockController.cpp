#include "BlockController.h"

BlockController::BlockController(Block** blocks)
: m_blocks(blocks)
{
}

void BlockController::begin()
{
}

void BlockController::update(uint32_t now)
{
    for (uint8_t i = 1; i <= 9; ++i) {
        if (m_blocks[i])
            m_blocks[i]->update(now);
    }
}

uint8_t BlockController::countZuegeOben() const
{
    uint8_t cnt = 0;
    for (uint8_t i = 1; i <= 3; ++i)
        if (m_blocks[i] && m_blocks[i]->besetzt())
            cnt++;

    return cnt;
}

// 3 → 4 Entscheidung
bool BlockController::canFahren_3_nach_4() const
{
    if (!m_trafoUntenOn) return false;

    Block* b3 = m_blocks[3];
    Block* b4 = m_blocks[4];
    Block* b6 = m_blocks[6];

    if (!b3 || !b4) return false;

    if (b4->besetzt()) return false;

    bool block6Besetzt = (b6 && b6->besetzt());
    uint8_t oben = countZuegeOben();

    if (!block6Besetzt) return true;

    return (oben > 2);
}

// 4 → oben Entscheidung
bool BlockController::canFahren_4_nach_oben() const
{
    if (!m_trafoObenOn) return false;

    Block* b4 = m_blocks[4];
    if (!b4) return false;

    if (!b4->besetzt()) return false;

    for (uint8_t i = 1; i <= 3; ++i)
        if (m_blocks[i] && m_blocks[i]->frei())
            return true;

    return false;
}
