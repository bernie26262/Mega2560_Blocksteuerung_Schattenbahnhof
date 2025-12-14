#pragma once
#include <Arduino.h>

class SensorKontakt;
class SensorStrom;

class Block
{
public:
    Block(uint8_t id,
          SensorKontakt* kontakt,
          SensorStrom* strom,
          SensorKontakt* kontakt2 = nullptr,
          SensorKontakt* kontakt3 = nullptr);

    void begin();
    void update(uint32_t nowMs);

    // Physischer Zustand
    bool besetzt() const;

    // B1: Freigabe für Einfahrt (zeitverzögert)
    bool isFreeForEntry() const;

    uint8_t id() const { return m_id; }

private:
    bool kontaktAktiv() const;
    bool stromAktiv() const;

private:
    uint8_t m_id;

    SensorKontakt* m_kontakt1;
    SensorKontakt* m_kontakt2;
    SensorKontakt* m_kontakt3;
    SensorStrom*   m_strom;

    // Zustände
    bool m_physicallyOccupied = false;

    // Zeitlogik
    uint32_t m_lastKontaktHighMs = 0;
    uint32_t m_lastStromZeroMs   = 0;

    bool m_kontaktHigh = true;
    bool m_stromZero   = true;

    static constexpr uint32_t KONTAKT_FREE_DELAY_MS = 3000;
    static constexpr uint32_t STROM_FREE_DELAY_MS   = 3000;
};
