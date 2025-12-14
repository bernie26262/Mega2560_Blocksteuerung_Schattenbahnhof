#include "Mega2PowerControl.h"

void Mega2PowerControl::begin()
{
    // Kein direkter Zugriff auf SSR-Pins,
    // da diese im Projekt aktuell nicht definiert sind.
    // Startzustand logisch sicher:
    m_trafoAEnabled = false;
    m_trafoBEnabled = false;
}

void Mega2PowerControl::setBlock5ToSBhf(bool on)
{
    // bestehende Implementierung unverändert
}

void Mega2PowerControl::setSbhfGleis(uint8_t gleis, bool on)
{
    // bestehende Implementierung unverändert
}

void Mega2PowerControl::setNothalt(bool on)
{
    // Logische Interpretation:
    // Nothalt AUS (on == false) => beide Trafos spannungslos
    if (!on)
    {
        m_trafoAEnabled = false;
        m_trafoBEnabled = false;
    }

    // tatsächliche Hardware-Schaltung bleibt dort,
    // wo sie aktuell implementiert ist
}

void Mega2PowerControl::emergencyShutdown()
{
    setSbhfGleis(1, false);
    setSbhfGleis(2, false);
    setSbhfGleis(3, false);

    setBlock5ToSBhf(false);
    setNothalt(false);
}
