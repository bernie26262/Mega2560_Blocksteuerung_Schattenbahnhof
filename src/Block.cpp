#include "Block.h"
#include "SensorKontakt.h"
#include "SensorStrom.h"
#include "mega2_debug.h"

Block::Block(uint8_t id,
             SensorKontakt* k1,
             SensorStrom* strom,
             SensorKontakt* k2,
             SensorKontakt* k3)
: m_id(id),
  m_kontakt1(k1),
  m_kontakt2(k2),
  m_kontakt3(k3),
  m_strom(strom)
{
}

void Block::begin()
{
    m_kontaktLow = false;
    m_stromOn    = false;
    m_lastFreeMs = millis();
}

void Block::update(uint32_t nowMs)
{
    updateContact(nowMs);
    updateStrom(nowMs);
}

void Block::updateContact(uint32_t nowMs)
{
    bool now =
        (m_kontakt1 && m_kontakt1->isOccupied()) ||
        (m_kontakt2 && m_kontakt2->isOccupied()) ||
        (m_kontakt3 && m_kontakt3->isOccupied());

    if (now != m_kontaktLow)
    {
        m_kontaktLow = now;
        DBG_PRINT("[B"); DBG_PRINT(m_id);
        DBG_PRINT("] Kontakt ");
        DBG_PRINTLN(now ? "LOW" : "FREI");

        if (!now)
            m_lastFreeMs = nowMs;
    }
}

void Block::updateStrom(uint32_t nowMs)
{
    bool now = (m_strom && m_strom->overThreshold());

    if (now != m_stromOn)
    {
        m_stromOn = now;
        DBG_PRINT("[B"); DBG_PRINT(m_id);
        DBG_PRINT("] Strom ");
        DBG_PRINTLN(now ? "AN" : "AUS");

        if (!now)
            m_lastFreeMs = nowMs;
    }
}

bool Block::besetzt() const
{
    return m_kontaktLow || m_stromOn;
}

bool Block::isReallyFree(uint32_t nowMs) const
{
    // 3s Kontakt + 3s Sicherheitszeit
    return !besetzt() && (nowMs - m_lastFreeMs >= 6000);
}

uint16_t Block::stromFiltered() const
{
    if (!m_strom)
        return 0;
    return m_strom->filtered();
}
