#include "Block.h"
#include "SensorKontakt.h"
#include "SensorStrom.h"

Block::Block(uint8_t id,
             SensorKontakt* mainKontakt,
             SensorStrom* strom,
             SensorKontakt* bhfA,
             SensorKontakt* bhfB)
: m_id(id),
  m_main(mainKontakt),
  m_bhfA(bhfA),
  m_bhfB(bhfB),
  m_strom(strom)
{}

void Block::begin()
{
    // aktuell nichts
}

void Block::update(uint32_t /*now*/)
{
    bool b = false;

    if (m_main && m_main->isOccupied()) b = true;
    if (m_bhfA && m_bhfA->isOccupied()) b = true;
    if (m_bhfB && m_bhfB->isOccupied()) b = true;

    if (m_strom && m_strom->overThreshold()) b = true;

    m_besetzt = b;
}

bool Block::besetzt() const
{
    return m_besetzt;
}

uint16_t Block::stromRaw() const
{
    return m_strom ? m_strom->filtered() : 0;
}
