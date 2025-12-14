#include "Block.h"
#include "SensorKontakt.h"
#include "SensorStrom.h"

// Sicherheitszeiten
static constexpr uint32_t KONTAKT_FREE_DELAY_MS = 3000;
static constexpr uint32_t STROM_FREE_DELAY_MS   = 3000;

Block::Block(uint8_t id,
             SensorKontakt* k1,
             SensorStrom* strom,
             SensorKontakt* k2,
             SensorKontakt* k3)
: m_id(id),
  m_kontakt1(k1),
  m_kontakt2(k2),
  m_kontakt3(k3),
  m_strom(strom),
  m_kontaktLow(false),
  m_stromOn(false),
  m_lastKontaktHighMs(0),
  m_lastStromZeroMs(0)
{
}

void Block::begin()
{
    uint32_t now = millis();
    m_lastKontaktHighMs = now;
    m_lastStromZeroMs   = now;
}

void Block::update(uint32_t nowMs)
{
    updateContact(nowMs);
    updateStrom(nowMs);
}

void Block::updateContact(uint32_t nowMs)
{
    bool occupied =
        (m_kontakt1 && m_kontakt1->isOccupied()) ||
        (m_kontakt2 && m_kontakt2->isOccupied()) ||
        (m_kontakt3 && m_kontakt3->isOccupied());

    // Belegt → frei (LOW → HIGH)
    if (m_kontaktLow && !occupied)
    {
        m_lastKontaktHighMs = nowMs;
    }

    m_kontaktLow = occupied;
}

void Block::updateStrom(uint32_t nowMs)
{
    bool on = m_strom && m_strom->overThreshold();

    if (m_stromOn && !on)
    {
        // Strom gerade 0 geworden
        m_lastStromZeroMs = nowMs;
    }

    m_stromOn = on;
}

// --------------------------------------------------
// Status
// --------------------------------------------------

bool Block::kontaktAktiv() const
{
    return m_kontaktLow;
}

bool Block::stromAktiv() const
{
    return m_stromOn;
}

bool Block::besetzt() const
{
    return m_kontaktLow || m_stromOn;
}

// --------------------------------------------------
// B4.1: zeitlich stabile Freigabe
// --------------------------------------------------

bool Block::isReallyFree(uint32_t nowMs) const
{
    if (besetzt())
        return false;

    if ((nowMs - m_lastKontaktHighMs) < KONTAKT_FREE_DELAY_MS)
        return false;

    if ((nowMs - m_lastStromZeroMs) < STROM_FREE_DELAY_MS)
        return false;

    return true;
}
