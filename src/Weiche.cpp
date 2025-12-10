#include "Weiche.h"

Weiche::Weiche(uint8_t id,
               uint8_t pinGerade,
               uint8_t pinAbbiegen,
               SensorKontakt* rueckmelderAbbiegen)
    : m_id(id),
      m_pinGerade(pinGerade),
      m_pinAbbiegen(pinAbbiegen),
      m_rueckmelder(rueckmelderAbbiegen)
{
}

void Weiche::begin()
{
    pinMode(m_pinGerade, OUTPUT);
    pinMode(m_pinAbbiegen, OUTPUT);

    digitalWrite(m_pinGerade, HIGH);
    digitalWrite(m_pinAbbiegen, HIGH);

    if (m_rueckmelder)
        m_rueckmelder->begin();
}

void Weiche::schalte(Stellung s)
{
    if (m_simValid) {
        m_sim = s;
        return;
    }

    if (m_phase != IDLE)
        return;

    m_stellung = s;

    if (s == GERADE)
        digitalWrite(m_pinGerade, LOW);
    else
        digitalWrite(m_pinAbbiegen, LOW);

    m_phase = IMPULS_ACTIVE;
    m_phaseStart = millis();
}

void Weiche::update(uint32_t now)
{
    if (m_rueckmelder)
        m_rueckmelder->update(now);

    if (m_simValid)
        return;

    switch (m_phase)
    {
        case IDLE:
            break;

        case IMPULS_ACTIVE:
            if (now - m_phaseStart >= PULSE_MS) {
                digitalWrite(m_pinGerade, HIGH);
                digitalWrite(m_pinAbbiegen, HIGH);
                m_phase = COOLDOWN;
                m_phaseStart = now;
            }
            break;

        case COOLDOWN:
            if (now - m_phaseStart >= COOLDOWN_MS) {
                m_phase = IDLE;
            }
            break;
    }
}

bool Weiche::rueckmeldungAbbiegen() const
{
    if (!m_rueckmelder)
        return false;

    return m_rueckmelder->isOccupied();
}

void Weiche::setSim(Stellung s)
{
    m_simValid = true;
    m_sim = s;
}

void Weiche::clearSim()
{
    m_simValid = false;
}
