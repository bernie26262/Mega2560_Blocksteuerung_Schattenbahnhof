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

    uint8_t id() const { return m_id; }
    void update(uint32_t now);

    bool schalte(Stellung s, bool force = false);

    Stellung getStellung() const { return m_stellung; }
    bool rueckmeldungAbbiegen() const;

    void setSim(Stellung s);
    void clearSim();

    inline void setGerade() {
        (void)schalte(GERADE, false);
    }

    inline void setAbzweig() {
        (void)schalte(ABBIEGEN, false);
    }

private:
    uint8_t m_id;
    uint8_t m_pinGerade;
    uint8_t m_pinAbbiegen;

    SensorKontakt* m_rueckmelder;

    Stellung m_stellung = GERADE;
    Stellung m_sim      = GERADE;
    bool m_simValid     = false;

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
