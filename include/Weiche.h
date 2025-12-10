#pragma once
#include <Arduino.h>
#include "SensorKontakt.h"

class Weiche
{
public:
    enum Stellung {
        GERADE = 0,
        ABBIEGEN = 1
    };

    Weiche(uint8_t id,
           uint8_t pinGerade,
           uint8_t pinAbbiegen,
           SensorKontakt* rueckmelderAbbiegen);

    void begin();
    void update(uint32_t now);

    void schalte(Stellung s);

    Stellung getStellung() const { return m_stellung; }
    bool rueckmeldungAbbiegen() const;

    void setSim(Stellung s);
    void clearSim();

private:
    uint8_t m_id;

    uint8_t m_pinGerade;
    uint8_t m_pinAbbiegen;

    SensorKontakt* m_rueckmelder;

    Stellung m_stellung = GERADE;
    Stellung m_sim      = GERADE;

    bool m_simValid = false;

    enum Phase {
        IDLE,
        IMPULS_ACTIVE,
        COOLDOWN
    };

    Phase m_phase = IDLE;
    uint32_t m_phaseStart = 0;

    static constexpr uint16_t PULSE_MS    = 500;
    static constexpr uint16_t COOLDOWN_MS = 1000;
};
