#pragma once
#include <Arduino.h>
#include "SensorKontakt.h"
#include "SensorStrom.h"

class Block {

public:
    Block(uint8_t id,
          SensorKontakt* mainKontakt,
          SensorStrom* strom,
          SensorKontakt* bhf1 = nullptr,
          SensorKontakt* bhf2 = nullptr);

    void begin();
    void update(uint32_t now);

    bool besetzt() const;
    bool frei() const { return !besetzt(); }

    uint8_t id() const { return m_id; }

    void setBesetztSim(bool v);
    void clearSimOverride();

private:
    uint8_t        m_id;

    SensorKontakt* m_mainKontakt;
    SensorStrom*   m_strom;
    SensorKontakt* m_bhf1;
    SensorKontakt* m_bhf2;

    bool m_currentBesetzt = false;

    bool m_simOverride      = false;
    bool m_simOverrideValid = false;
};
