#include "Block.h"
#include "SensorKontakt.h"
#include "SensorStrom.h"

Block::Block(uint8_t id,
             SensorKontakt* kontakt,
             SensorStrom* strom,
             SensorKontakt* kontakt2,
             SensorKontakt* kontakt3)
: m_id(id)
, m_kontakt1(kontakt)
, m_kontakt2(kontakt2)
, m_kontakt3(kontakt3)
, m_strom(strom)
{
}

void Block::begin()
{
    uint32_t now = millis();
    m_lastKontaktHighMs = now;
    m_lastStromZeroMs   = now;
}

bool Block::kontaktAktiv() const
{
    // SensorKontakt: isOccupied() == true → LOW → Kontakt aktiv
    if (m_kontakt1 && m_kontakt1->isOccupied()) return true;
    if (m_kontakt2 && m_kontakt2->isOccupied()) return true;
    if (m_kontakt3 && m_kontakt3->isOccupied()) return true;
    return false;
}

bool Block::stromAktiv() const
{
    if (!m_strom) return false;
    return m_strom->overThreshold();
}

void Block::update(uint32_t nowMs)
{
    bool kontaktNow = kontaktAktiv();
    bool stromNow   = stromAktiv();

    // Kontakt: LOW → HIGH
    if (!kontaktNow && !m_kontaktHigh)
    {
        m_lastKontaktHighMs = nowMs;
        m_kontaktHigh = true;
    }
    else if (kontaktNow)
    {
        m_kontaktHigh = false;
    }

    // Strom: >Threshold → 0
    if (!stromNow && !m_stromZero)
    {
        m_lastStromZeroMs = nowMs;
        m_stromZero = true;
    }
    else if (stromNow)
    {
        m_stromZero = false;
    }

    m_physicallyOccupied = kontaktNow || stromNow;
}

bool Block::besetzt() const
{
    return m_physicallyOccupied;
}

bool Block::isFreeForEntry() const
{
    if (m_physicallyOccupied)
        return false;

    uint32_t now = millis();

    if (now - m_lastKontaktHighMs < KONTAKT_FREE_DELAY_MS)
        return false;

    if (now - m_lastStromZeroMs < STROM_FREE_DELAY_MS)
        return false;

    return true;
}
