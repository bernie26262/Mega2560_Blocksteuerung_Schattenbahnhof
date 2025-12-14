#include "Block.h"
#include "SensorKontakt.h"
#include "SensorStrom.h"
#include "mega2_debug.h"

// --------------------------------------------------
// Konstruktor
// --------------------------------------------------
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

// --------------------------------------------------
// Init
// --------------------------------------------------
void Block::begin()
{
    m_kontaktAktiv = false;
    m_stromAktiv   = false;
    m_lastFreeMs   = millis();

    m_lastKontaktHighMs = millis();
    m_lastStromZeroMs   = millis();
}

// --------------------------------------------------
// Update
// --------------------------------------------------
void Block::update(uint32_t nowMs)
{
    updateContact(nowMs);
    updateStrom(nowMs);
}

// --------------------------------------------------
// Kontaktlogik
// --------------------------------------------------
void Block::updateContact(uint32_t nowMs)
{
    bool now =
        (m_kontakt1 && m_kontakt1->isOccupied()) ||
        (m_kontakt2 && m_kontakt2->isOccupied()) ||
        (m_kontakt3 && m_kontakt3->isOccupied());

    if (now != m_kontaktAktiv)
    {
        DBG_PRINT("[B"); DBG_PRINT(m_id);
        DBG_PRINT("] Kontakt ");
        DBG_PRINTLN(now ? "AKTIV" : "FREI");

        m_kontaktAktiv = now;

        if (!now)
            m_lastKontaktHighMs = nowMs;
    }
}

// --------------------------------------------------
// Stromlogik
// --------------------------------------------------
void Block::updateStrom(uint32_t nowMs)
{
    bool now = (m_strom && m_strom->overThreshold());

    if (now != m_stromAktiv)
    {
        DBG_PRINT("[B"); DBG_PRINT(m_id);
        DBG_PRINT("] Strom ");
        DBG_PRINTLN(now ? "AKTIV" : "0");

        m_stromAktiv = now;

        if (!now)
            m_lastStromZeroMs = nowMs;
    }
}

// --------------------------------------------------
// Belegung
// --------------------------------------------------
bool Block::besetzt() const
{
    return m_kontaktAktiv || m_stromAktiv;
}

// --------------------------------------------------
// Zeitlich stabile Freigabe (B4.1)
// --------------------------------------------------
bool Block::isReallyFree(uint32_t nowMs) const
{
    if (m_kontaktAktiv || m_stromAktiv)
        return false;

    uint32_t dtKontakt = nowMs - m_lastKontaktHighMs;
    uint32_t dtStrom   = nowMs - m_lastStromZeroMs;

    return (dtKontakt >= 3000) && (dtStrom >= 3000);
}
