#pragma once
#include <Arduino.h>

class SensorKontakt;
class SensorStrom;

class Block
{
public:
    Block(uint8_t id,
          SensorKontakt* k1,
          SensorStrom* strom,
          SensorKontakt* k2 = nullptr,
          SensorKontakt* k3 = nullptr);

    void begin();
    void update(uint32_t nowMs);

    bool besetzt() const;
    bool isReallyFree(uint32_t nowMs) const;

    // --- Debug / Status ---
    uint8_t  id() const { return m_id; }
    bool     kontaktAktiv() const { return m_kontaktLow; }
    bool     stromAktiv()   const { return m_stromOn; }
    uint16_t stromFiltered() const;

private:
    void updateContact(uint32_t nowMs);
    void updateStrom(uint32_t nowMs);

private:
    uint8_t m_id;

    SensorKontakt* m_kontakt1;
    SensorKontakt* m_kontakt2;
    SensorKontakt* m_kontakt3;
    SensorStrom*   m_strom;

    bool     m_kontaktLow = false;
    bool     m_stromOn    = false;
    uint32_t m_lastFreeMs = 0;
};
