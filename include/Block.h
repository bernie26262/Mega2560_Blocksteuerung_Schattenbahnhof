#pragma once
#include <Arduino.h>

class SensorKontakt;
class SensorStrom;

class Block {
public:
    Block(uint8_t id,
          SensorKontakt* mainKontakt,
          SensorStrom* strom,
          SensorKontakt* bhfA = nullptr,
          SensorKontakt* bhfB = nullptr);

    void begin();
    void update(uint32_t now);
    bool besetzt() const;

    uint16_t stromRaw() const;   // ← NEU (optional für Status)

private:
    uint8_t m_id;

    SensorKontakt* m_main;
    SensorKontakt* m_bhfA;
    SensorKontakt* m_bhfB;
    SensorStrom*   m_strom;

    bool m_besetzt = false;
};