#include "Block.h"

#include <Arduino.h>
#include <stdint.h>

#include "SensorKontakt.h"
#include "SensorStrom.h"
#include "mega2_debug.h"
#include "threshold_values.h"

#ifndef MEGA2_DEBUG_BLOCK_OCC
#define MEGA2_DEBUG_BLOCK_OCC 0
#endif

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
    // Debounced/logischer Kontaktzustand statt Rohsignal
    return k ? k->isOccupied() : false;
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

    bool nowStrom = false;
    if (m_strom)
    {
        const uint16_t ma = m_strom->rms_mA();
        nowStrom = (ma >= THR_BLOCK_OCC_SET_MA);
    }

    m_kontaktAktiv = nowKontakt;
    m_stromAktiv   = nowStrom;
    m_besetzt      = (nowKontakt || nowStrom);
    m_stromOccCount  = 0;
    m_stromFreeCount = 0;

    const uint32_t now = millis();
    m_lastKontaktHighMs = nowKontakt ? 0 : now;
    m_lastStromZeroMs   = nowStrom   ? 0 : now;
    m_lastFreeMs        = m_besetzt  ? 0 : now;
}

void Block::updateContact(uint32_t nowMs)
{
    const bool nowKontakt =
        kontaktOcc(m_kontakt1) ||
        kontaktOcc(m_kontakt2) ||
        kontaktOcc(m_kontakt3);

    if (nowKontakt != m_kontaktAktiv)
    {
        m_kontaktAktiv = nowKontakt;
        if (!nowKontakt) m_lastKontaktHighMs = nowMs;
        else             m_lastKontaktHighMs = 0;

    #if MEGA2_DEBUG && MEGA2_DEBUG_BLOCK_EVENTS
        dbgPrintBlockEvent(m_id, F("Kontakt"), nowKontakt);
    #endif
    }
}

void Block::updateStrom(uint32_t nowMs)
{
    if (!m_strom)
    {
        if (m_stromAktiv)
        {
            m_stromAktiv = false;
            m_lastStromZeroMs = nowMs;
        }
        m_stromOccCount  = 0;
        m_stromFreeCount = 0;
        return;
    }

    const uint16_t ma = m_strom->rms_mA();
    bool newStromAktiv = m_stromAktiv;

    if (!m_stromAktiv)
    {
        // Frei -> Belegt nur nach stabiler Überschreitung der SET-Schwelle
        if (ma >= THR_BLOCK_OCC_SET_MA)
        {
            if (m_stromOccCount < 255) ++m_stromOccCount;
            m_stromFreeCount = 0;
            if (m_stromOccCount >= THR_BLOCK_OCC_SET_WINDOWS)
            {
                newStromAktiv = true;
                m_stromOccCount  = 0;
                m_stromFreeCount = 0;
            }
        }
        else
        {
            m_stromOccCount = 0;
            if (ma <= THR_BLOCK_OCC_CLEAR_MA)
                m_stromFreeCount = 0;
        }
    }
    else
    {
        // Belegt -> Frei nur nach stabiler Unterschreitung der CLEAR-Schwelle
        if (ma <= THR_BLOCK_OCC_CLEAR_MA)
        {
            if (m_stromFreeCount < 255) ++m_stromFreeCount;
            m_stromOccCount = 0;
            if (m_stromFreeCount >= THR_BLOCK_OCC_CLEAR_WINDOWS)
            {
                newStromAktiv = false;
                m_stromOccCount  = 0;
                m_stromFreeCount = 0;
            }
        }
        else
        {
            m_stromFreeCount = 0;
            if (ma >= THR_BLOCK_OCC_SET_MA)
                m_stromOccCount = 0;
        }
    }

    if (newStromAktiv != m_stromAktiv)
    {
        m_stromAktiv = newStromAktiv;
        if (!m_stromAktiv) m_lastStromZeroMs = nowMs;
        else               m_lastStromZeroMs = 0;

    #if MEGA2_DEBUG && MEGA2_DEBUG_BLOCK_EVENTS
        dbgPrintBlockEvent(m_id, F("Strom"), m_stromAktiv);
    #endif
    }
}

void Block::update(uint32_t nowMs)
{
    updateContact(nowMs);
    updateStrom(nowMs);

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
#if MEGA2_DEBUG_BLOCK_OCC
    static uint32_t s_lastLogMs = 0;
    if ((nowMs - s_lastLogMs) > 250)
    {
        s_lastLogMs = nowMs;

        DBG_PRINT(F("[BOCC] B"));
        DBG_PRINT(m_id);

        DBG_PRINT(F(" I="));
        DBG_PRINT(stromRms_mA());

        DBG_PRINT(F(" occ="));
        DBG_PRINT(m_besetzt);

        DBG_PRINT(F(" k="));
        DBG_PRINT(m_kontaktAktiv);

        DBG_PRINT(F(" s="));
        DBG_PRINT(m_stromAktiv);

        DBG_PRINT(F(" oc="));
        DBG_PRINT(m_stromOccCount);

        DBG_PRINT(F(" fc="));
        DBG_PRINT(m_stromFreeCount);

        DBG_PRINTLN();
    }
#endif
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