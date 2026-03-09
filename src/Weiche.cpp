#include "Weiche.h"
#include "mega2_debug.h"

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

bool Weiche::schalte(Stellung s, bool force)
{
    DBG_SBHF_ST("[WST] schalte W%u req=%s force=%u phase=%u pinG=%u pinA=%u ms=%lu\n",
                (unsigned)m_id,
                (s == GERADE) ? "GERADE" : "ABBIEGEN",
                (unsigned)force,
                (unsigned)m_phase,
                (unsigned)m_pinGerade,
                (unsigned)m_pinAbbiegen,
                (unsigned long)millis());

    if (m_phase != IDLE)
    {
        if (!force) {
            DBG_SBHF_ST("[WST] reject  W%u busy phase=%u ms=%lu\n",
                        (unsigned)m_id,
                        (unsigned)m_phase,
                        (unsigned long)millis());
            return false;
        }
        // Force: laufenden Zustand abbrechen (für Selftest/Servicefälle)
        digitalWrite(m_pinGerade, HIGH);
        digitalWrite(m_pinAbbiegen, HIGH);
        m_phase = IDLE;
        DBG_SBHF_ST("[WST] force-abort W%u -> both HIGH ms=%lu\n",
                    (unsigned)m_id,
                    (unsigned long)millis());
    }

    // Robustheit: bei jedem Schaltvorgang OUTPUT erzwingen (gegen versehentliches Umkonfigurieren / Störungen).
    pinMode(m_pinGerade, OUTPUT);
    pinMode(m_pinAbbiegen, OUTPUT);
    // Default OFF (low-aktiv): beide HIGH, dann den Ziel-Pin LOW pulsen.
    digitalWrite(m_pinGerade, HIGH);
    digitalWrite(m_pinAbbiegen, HIGH);

    m_stellung = s;
    m_phase = IMPULS_ACTIVE;
    m_phaseStart = millis();
    DBG_SBHF_ST("[WST] pulse-start W%u phase=%u start=%lu target=%s\n",
                (unsigned)m_id,
                (unsigned)m_phase,
                (unsigned long)m_phaseStart,
                (s == GERADE) ? "GERADE" : "ABBIEGEN");

    if (s == GERADE)
    {
        digitalWrite(m_pinGerade, LOW);
        digitalWrite(m_pinAbbiegen, HIGH);
        DBG_SBHF_ST("[WST] output W%u GERADE pin%u=LOW pin%u=HIGH ms=%lu\n",
                    (unsigned)m_id,
                    (unsigned)m_pinGerade,
                    (unsigned)m_pinAbbiegen,
                    (unsigned long)millis());
    }
    else
    {
        digitalWrite(m_pinAbbiegen, LOW);
        digitalWrite(m_pinGerade, HIGH);
        DBG_SBHF_ST("[WST] output W%u ABBIEGEN pin%u=LOW pin%u=HIGH ms=%lu\n",
                    (unsigned)m_id,
                    (unsigned)m_pinAbbiegen,
                    (unsigned)m_pinGerade,
                    (unsigned long)millis());
    }

    return true;
}

void Weiche::update(uint32_t now)
{
    if (m_phase == IMPULS_ACTIVE &&
        now - m_phaseStart >= PULSE_MS)
    {
        DBG_SBHF_ST("[WST] pulse-end W%u now=%lu start=%lu dt=%lu -> both HIGH\n",
                    (unsigned)m_id,
                    (unsigned long)now,
                    (unsigned long)m_phaseStart,
                    (unsigned long)(now - m_phaseStart));
        digitalWrite(m_pinGerade, HIGH);
        digitalWrite(m_pinAbbiegen, HIGH);
        m_phase = COOLDOWN;
        m_phaseStart = now;
        DBG_SBHF_ST("[WST] cooldown W%u start=%lu\n",
                    (unsigned)m_id,
                    (unsigned long)m_phaseStart);
    }

    if (m_phase == COOLDOWN &&
        now - m_phaseStart >= COOLDOWN_MS)
    {
        DBG_SBHF_ST("[WST] idle W%u now=%lu dt=%lu\n",
                    (unsigned)m_id,
                    (unsigned long)now,
                    (unsigned long)(now - m_phaseStart));
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
