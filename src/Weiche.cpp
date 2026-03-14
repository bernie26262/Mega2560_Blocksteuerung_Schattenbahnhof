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
    // Boot-glitch vermeiden: erst Output-Latch setzen, dann DDR
    digitalWrite(m_pinGerade, HIGH);
    digitalWrite(m_pinAbbiegen, HIGH);
    pinMode(m_pinGerade, OUTPUT);
    pinMode(m_pinAbbiegen, OUTPUT);
}

bool Weiche::schalte(Stellung s, uint32_t nowMs, bool force)
{
    if (m_phase != IDLE)
    {
        if (!force) {
            return false;
        }
        // Force: laufenden Zustand abbrechen (für Selftest/Servicefälle)
        digitalWrite(m_pinGerade, HIGH);
        digitalWrite(m_pinAbbiegen, HIGH);
        m_phase = IDLE;
    }

    // Robustheit: bei jedem Schaltvorgang OUTPUT erzwingen (gegen versehentliches Umkonfigurieren / Störungen).
    pinMode(m_pinGerade, OUTPUT);
    pinMode(m_pinAbbiegen, OUTPUT);
    // Default OFF (low-aktiv): beide HIGH, dann den Ziel-Pin LOW pulsen.
    digitalWrite(m_pinGerade, HIGH);
    digitalWrite(m_pinAbbiegen, HIGH);

    m_stellung = s;
    m_phase = IMPULS_ACTIVE;
    m_phaseStart = nowMs;

    if (s == GERADE)
    {
        digitalWrite(m_pinGerade, LOW);
        digitalWrite(m_pinAbbiegen, HIGH);
    }
    else
    {
        digitalWrite(m_pinAbbiegen, LOW);
        digitalWrite(m_pinGerade, HIGH);
    }

    return true;
}

void Weiche::update(uint32_t now)
{
    if (m_phase == IMPULS_ACTIVE &&
        now - m_phaseStart >= PULSE_MS)
    {
        digitalWrite(m_pinGerade, HIGH);
        digitalWrite(m_pinAbbiegen, HIGH);
        m_phase = COOLDOWN;
        m_phaseStart = now;
    }

    if (m_phase == COOLDOWN &&
        now - m_phaseStart >= COOLDOWN_MS)
    {
        m_phase = IDLE;
    }
}

bool Weiche::rueckmeldungAbbiegen() const
{
    if (!m_rueckmelder)
        return false;

    // raw() nutzt in SIM auch debugForce und ist in HW ohne extra update() aktuell
    return m_rueckmelder->raw();
}
