#include "Block.h"

Block::Block(uint8_t id,
             SensorKontakt* mainKontakt,
             SensorStrom* strom,
             SensorKontakt* bhf1,
             SensorKontakt* bhf2)
    : m_id(id),
      m_mainKontakt(mainKontakt),
      m_strom(strom),
      m_bhf1(bhf1),
      m_bhf2(bhf2)
{
}

void Block::begin() {}

void Block::update(uint32_t now)
{
    (void)now;

    if (m_simOverrideValid) {
        m_currentBesetzt = m_simOverride;
        return;
    }

    bool b = false;

    if (m_mainKontakt && m_mainKontakt->isOccupied()) b = true;
    if (m_bhf1        && m_bhf1->isOccupied())        b = true;
    if (m_bhf2        && m_bhf2->isOccupied())        b = true;
    if (m_strom       && m_strom->overThreshold())    b = true;

    m_currentBesetzt = b;
}

bool Block::besetzt() const {
    return m_currentBesetzt;
}

void Block::setBesetztSim(bool v) {
    m_simOverrideValid = true;
    m_simOverride      = v;
    m_currentBesetzt   = v;
}

void Block::clearSimOverride() {
    m_simOverrideValid = false;
}
