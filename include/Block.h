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

    // Status
    bool besetzt() const;
    bool kontaktAktiv() const;
    bool stromAktiv() const;

    // B4.1: zeitlich stabile Freigabe
    bool isReallyFree(uint32_t nowMs) const;

    uint8_t id() const { return m_id; }

private:
    void updateContact(uint32_t nowMs);
    void updateStrom(uint32_t nowMs);

private:
    uint8_t m_id;

    SensorKontakt* m_kontakt1;
    SensorKontakt* m_kontakt2;
    SensorKontakt* m_kontakt3;
    SensorStrom*   m_strom;

    // --- Status ---
    bool m_kontaktLow;
    bool m_stromOn;

    // --- Zeitstempel ---
    uint32_t m_lastKontaktHighMs;
    uint32_t m_lastStromZeroMs;
};
