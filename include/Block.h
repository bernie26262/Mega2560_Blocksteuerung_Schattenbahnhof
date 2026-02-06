#pragma once
#include <Arduino.h>

class SensorKontakt;
class SensorStrom;

class Block
{
public:
    // Freigabe erst nach stabil "frei" (Debounce/Delay)
    // (occupied->free verzögert; occupied->true weiterhin sofort)
    static constexpr uint32_t STABLE_FREE_MS   = 2500;
    static constexpr uint32_t STABLE_SIGNAL_MS = 50;

    Block(uint8_t id,
          SensorKontakt* k1,
          SensorStrom* strom,
          SensorKontakt* k2 = nullptr,
          SensorKontakt* k3 = nullptr);

    void begin();
    void update(uint32_t nowMs);

    // Status
    bool besetzt() const;
    bool kontaktAktiv() const { return m_kontaktAktiv; }
    bool stromAktiv()   const { return m_stromAktiv; }

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

    // --- Status (einheitlich!) ---
    bool     m_kontaktAktiv = false;
    bool     m_stromAktiv   = false;
    bool     m_besetzt      = false;
    uint32_t m_lastFreeMs   = 0;

    // --- Zeitstempel ---
    uint32_t m_lastKontaktHighMs = 0;
    uint32_t m_lastStromZeroMs   = 0;
};
