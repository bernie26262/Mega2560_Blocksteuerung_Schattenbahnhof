#pragma once
#include <Arduino.h>

class SensorKontakt;
class SensorStrom;

class Block {
public:
    Block(uint8_t id, SensorKontakt* kontakt, SensorKontakt* bhf1,
          SensorKontakt* bhf2, SensorStrom* strom)
        : m_id(id), m_mainKontakt(kontakt),
          m_bhf1(bhf1), m_bhf2(bhf2), m_strom(strom) {}

    void begin();
    void update(uint32_t now);

    bool besetzt() const;
    bool frei() const { return !besetzt(); }

    uint16_t strom_mA() const;

    SensorKontakt* m_mainKontakt;
    SensorKontakt* m_bhf1;
    SensorKontakt* m_bhf2;

private:
    uint8_t m_id;
    SensorStrom* m_strom;
    bool m_isOccupied = false;
};
