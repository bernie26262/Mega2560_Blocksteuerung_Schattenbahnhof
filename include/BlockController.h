#pragma once
#include <Arduino.h>
#include "Block.h"

class BlockController {
public:
    BlockController(Block** blocks) : m_blocks(blocks) {}

    void begin();
    void update(uint32_t now);

    Block* block(uint8_t idx) const { return m_blocks[idx]; }

    uint8_t countZuegeOben() const;
    bool canFahren_3_nach_4() const;
    bool canFahren_4_nach_oben() const;

    bool m_trafoObenOn = true;
    bool m_trafoUntenOn = true;

    Block** blocks = m_blocks;

private:
    Block** m_blocks;
};
