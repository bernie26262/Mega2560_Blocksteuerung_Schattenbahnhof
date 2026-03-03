#include "Block.h"

#include <Arduino.h>
#include <stdint.h>

#include "SensorKontakt.h"
#include "SensorStrom.h"
#include "mega2_debug.h"

// Optional: Event-Logging pro Block (nur Edge, nicht "polling-spam")
#ifndef MEGA2_DEBUG_BLOCK_EVENTS
#define MEGA2_DEBUG_BLOCK_EVENTS 0
#endif

// Stabilitätsfenster sind in Block.h definiert (Block::STABLE_*)

#if MEGA2_DEBUG && MEGA2_DEBUG_BLOCK_EVENTS
static inline void dbgPrintBlockEvent(uint8_t id, const __FlashStringHelper* what, bool active)
{
    DBG_PRINT(F("[B"));
    DBG_PRINT(id);
    DBG_PRINT(F("] "));
    DBG_PRINT(what);
    DBG_PRINT(F(" "));
    DBG_PRINTLN(active ? F("AKTIV") : F("frei"));
}
#endif

Block::Block(uint8_t id,
             SensorKontakt* kontakt1,
             SensorStrom* strom,
             SensorKontakt* kontakt2,
             SensorKontakt* kontakt3)
: m_id(id)
, m_kontakt1(kontakt1)
, m_kontakt2(kontakt2)
, m_kontakt3(kontakt3)
, m_strom(strom)
, m_kontaktAktiv(false)
, m_stromAktiv(false)
, m_besetzt(false)
, m_lastFreeMs(0)
, m_lastKontaktHighMs(0)
, m_lastStromZeroMs(0)
{
}

static inline bool kontaktOcc(SensorKontakt* k)
{
    // SensorKontakt arbeitet mit INPUT_PULLUP; raw() ist im Projekt als "belegt/aktiv" definiert.
    return k ? k->raw() : false;
}

void Block::begin()
{
    
    // Stromsensor optional initialisieren (idempotent halten!)
    if (m_strom)
        m_strom->begin();

// Initialzustand aus den Sensoren lesen
    const bool nowKontakt =
        kontaktOcc(m_kontakt1) ||
        kontaktOcc(m_kontakt2) ||
        kontaktOcc(m_kontakt3);

    const bool nowStrom = (m_strom ? m_strom->overThreshold() : false);

    m_kontaktAktiv = nowKontakt;
    m_stromAktiv   = nowStrom;
    m_besetzt      = (nowKontakt || nowStrom);

    const uint32_t now = millis();
    m_lastKontaktHighMs = nowKontakt ? 0 : now;
    m_lastStromZeroMs   = nowStrom   ? 0 : now;
    m_lastFreeMs        = m_besetzt  ? 0 : now;
}

void Block::update(uint32_t nowMs)
{
    const bool nowKontakt =
        kontaktOcc(m_kontakt1) ||
        kontaktOcc(m_kontakt2) ||
        kontaktOcc(m_kontakt3);

    const bool nowStrom = (m_strom ? m_strom->overThreshold() : false);

    // Kontakt-Edge
    if (nowKontakt != m_kontaktAktiv)
    {
        m_kontaktAktiv = nowKontakt;
        if (!nowKontakt) m_lastKontaktHighMs = nowMs;
        else             m_lastKontaktHighMs = 0;

    #if MEGA2_DEBUG && MEGA2_DEBUG_BLOCK_EVENTS
        dbgPrintBlockEvent(m_id, F("Kontakt"), nowKontakt);
    #endif
    }

    // Strom-Edge
    if (nowStrom != m_stromAktiv)
    {
        m_stromAktiv = nowStrom;
        if (!nowStrom) m_lastStromZeroMs = nowMs;
        else           m_lastStromZeroMs = 0;

    #if MEGA2_DEBUG && MEGA2_DEBUG_BLOCK_EVENTS
        dbgPrintBlockEvent(m_id, F("Strom"), nowStrom);
    #endif
    }

    // Gesamtzustand
    const bool nowBesetzt = (m_kontaktAktiv || m_stromAktiv);
    if (nowBesetzt != m_besetzt)
    {
        m_besetzt = nowBesetzt;

        if (!m_besetzt)
            m_lastFreeMs = nowMs;
        else
            m_lastFreeMs = 0;

    #if MEGA2_DEBUG && MEGA2_DEBUG_BLOCK_EVENTS
        dbgPrintBlockEvent(m_id, F("Block"), m_besetzt);
    #endif
    }
}

bool Block::isReallyFree(uint32_t nowMs) const
{
    if (m_besetzt) return false;

    // Wenn nie gesetzt (z.B. direkt nach Boot), nehmen wir "frei" an
    if (m_lastFreeMs == 0) return true;

    if ((nowMs - m_lastFreeMs) < Block::STABLE_FREE_MS)
        return false;

    // Optional: extra Stabilität pro Signal (falls Sensor-Prellen)
    if (m_lastKontaktHighMs != 0 && (nowMs - m_lastKontaktHighMs) < Block::STABLE_SIGNAL_MS)
        return false;

    if (m_lastStromZeroMs != 0 && (nowMs - m_lastStromZeroMs) < Block::STABLE_SIGNAL_MS)
        return false;

    return true;
}

bool Block::besetzt() const
{
    return m_besetzt;
}

uint16_t Block::stromRmsCounts() const
{
    return m_strom ? m_strom->rmsCounts() : 0;
}

uint16_t Block::stromRms_mA() const
{
    return m_strom ? m_strom->rms_mA() : 0;
}