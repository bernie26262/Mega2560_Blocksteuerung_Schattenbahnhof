#include "ShadowYardController.h"

// ----------------------------------------------------------
// Konstruktor
// ----------------------------------------------------------
ShadowYardController::ShadowYardController(
    BlockController* bc,
    Weiche* w12,
    Weiche* w13,
    Weiche* w14,
    Weiche* w15,
    PowerControl* power,
    PulseSensor* s11,
    PulseSensor* s12,
    PulseSensor* s13,
    PulseSensor* s14,
    PulseSensor* s15,
    PulseSensor* s16)
: 
    m_bc(bc),
    m_w12(w12),
    m_w13(w13),
    m_w14(w14),
    m_w15(w15),
    m_power(power),
    m_s11(s11),
    m_s12(s12),
    m_s13(s13),
    m_s14(s14),
    m_s15(s15),
    m_s16(s16)
{
}

// ----------------------------------------------------------
// Initialisierung
// ----------------------------------------------------------
void ShadowYardController::begin()
{
    m_state         = SBhfState::Idle;
    m_cycleActive   = false;
    m_nothaltAktiv  = false;
    m_errorNothalt  = false;
    m_counter       = 0;
    m_targetGleis   = 0;
    m_exitGleis     = 0;
}

// ----------------------------------------------------------
// Haupt-Update – von loop() aufrufen
// ----------------------------------------------------------
void ShadowYardController::update(uint32_t now)
{
    // 1. Nothalt / Fehlerlogik
    handleNothalt();

    // 2. S15 schaltet Einfahrstrom (nur im aktiven Zyklus)
    handleS15ForEntry();

    // 3. S12/S13/S14 → Einfahrgleis erreicht
    handleEntrySensors();

    // 4. S11 → neuer Zug in Block 5
    handleNewArrival();
}

// ----------------------------------------------------------
// Nothalt-Logik (S15/S16) + Nothalt-Fehlerbedingung
// ----------------------------------------------------------
void ShadowYardController::handleNothalt()
{
    bool s15Edge = (m_s15 && m_s15->fellEdge());
    bool s16Edge = (m_s16 && m_s16->fellEdge());

    // S15 (vor Nothalt): Nothalt-Strom EIN
    if (s15Edge)
    {
        m_power->setNothalt(true);
        m_nothaltAktiv = true;

        // Falls vorher ein Nothalt-Error anlag und der Zug wieder korrekt
        // in Fahrtrichtung unterwegs ist, können wir ihn zurücksetzen.
        if (m_errorNothalt && m_state == SBhfState::ErrorNothalt) {
            m_errorNothalt = false;
            m_state        = SBhfState::Idle;
        }
    }

    // S16 (hinter Nothalt): Nothalt-Strom AUS
    if (s16Edge)
    {
        m_power->setNothalt(false);
        m_nothaltAktiv = false;
    }

    // --- Erweiterte Fehlerbedingung ---
    //
    // Echtes Problem nur dann:
    //  - Kontaktgleis Nothalt meldet "Zug steht im Nothaltbereich"
    //  - Block 6 ist NICHT besetzt (kein Zug davor)
    //  - Trafo unten ist eingeschaltet
    //
    // Dann steht ein Zug unerlaubt im Nothalt-Gleis und muss manuell entfernt werden.
    //
    if (isNothaltKontaktAktiv() &&
        m_bc && !m_bc->block(6)->besetzt() &&
        m_bc->m_trafoUntenOn)
    {
        m_errorNothalt = true;
        m_state        = SBhfState::ErrorNothalt;
    }
}

// ----------------------------------------------------------
// S15: Einfahrstrom Block5->SBhf EIN (nur wenn Zyklus aktiv)
// ----------------------------------------------------------
void ShadowYardController::handleS15ForEntry()
{
    if (!m_cycleActive)
        return;

    // S15 vom ausfahrenden Zug → Einfahrstrom wieder an
    if (m_s15 && m_s15->fellEdge())
    {
        m_power->setBlock5ToSBhf(true);
    }
}

// ----------------------------------------------------------
// S12/S13/S14: Zug ist im Zielgleis angekommen → SBhf-Gleis AUS
// ----------------------------------------------------------
void ShadowYardController::handleEntrySensors()
{
    if (!m_cycleActive)
        return;

    // Gleis 1
    if (m_targetGleis == 1 && m_s12 && m_s12->fellEdge())
    {
        m_power->setSbhfGleis(1, false);
        m_cycleActive = false;
        m_state       = SBhfState::Idle;
    }

    // Gleis 2
    if (m_targetGleis == 2 && m_s13 && m_s13->fellEdge())
    {
        m_power->setSbhfGleis(2, false);
        m_cycleActive = false;
        m_state       = SBhfState::Idle;
    }

    // Gleis 3
    if (m_targetGleis == 3 && m_s14 && m_s14->fellEdge())
    {
        m_power->setSbhfGleis(3, false);
        m_cycleActive = false;
        m_state       = SBhfState::Idle;
    }
}

// ----------------------------------------------------------
// S11: Neuer Zug kommt in Block 5 → Zyklus starten
// ----------------------------------------------------------
void ShadowYardController::handleNewArrival()
{
    if (!m_s11)
        return;

    if (!m_s11->fellEdge())
        return;

    // Block5->SBhf sofort AUS
    m_power->setBlock5ToSBhf(false);

    // Sicherheitsprüfung: Block 6 muss frei sein
    if (m_bc && m_bc->block(6)->besetzt())
    {
        m_state       = SBhfState::Blocked;
        m_cycleActive = false;
        return;
    }

    // Gleis auswählen
    chooseGleis();

    // Ausfahrt starten + Einfahrt vorbereiten
    startExitAndPrepareEntry();

    m_state       = SBhfState::CycleActive;
    m_cycleActive = true;
}

// ----------------------------------------------------------
// Gleiswahl (Serial oder Random Mode)
// ----------------------------------------------------------
void ShadowYardController::chooseGleis()
{
    bool g1 = m_bc && m_bc->block(7)->besetzt();
    bool g2 = m_bc && m_bc->block(8)->besetzt();
    bool g3 = m_bc && m_bc->block(9)->besetzt();

    if (!g1 && !g2 && !g3)
    {
        m_exitGleis   = 0;
        m_targetGleis = 0;
        m_cycleActive = false;
        m_state       = SBhfState::Idle;
        return;
    }

    if (m_mode == SBhfMode::Serial)
    {
        m_counter = (m_counter % 3) + 1;
        m_exitGleis   = m_counter;
        m_targetGleis = m_counter;
    }
    else
    {
        uint8_t used[3];
        uint8_t n = 0;
        if (g1) used[n++] = 1;
        if (g2) used[n++] = 2;
        if (g3) used[n++] = 3;

        if (n == 0) {
            m_exitGleis   = 0;
            m_targetGleis = 0;
            m_cycleActive = false;
            m_state       = SBhfState::Idle;
            return;
        }

        uint8_t idx = random(0, n);
        m_exitGleis   = used[idx];
        m_targetGleis = used[idx];
    }
}

// ----------------------------------------------------------
// Ausfahrt starten + Einfahrweichen setzen
// ----------------------------------------------------------
void ShadowYardController::startExitAndPrepareEntry()
{
    if (m_exitGleis == 0)
        return;

    applyEntryWeichen();
    applyExitWeichen();

    // SBhf-AUSFAHRGLEIS einschalten
    m_power->setSbhfGleis(m_exitGleis, true);

    // Block 5 bleibt stromlos bis S15
}

// ----------------------------------------------------------
// EINFAHRTWEICHEN gemäß korrigierter Logik
// ----------------------------------------------------------
//
// Gleis 1:
//   W12 = Abzweig
//   W13 = egal
//
// Gleis 2:
//   W12 = Gerade
//   W13 = Abzweig
//
// Gleis 3:
//   W12 = Gerade
//   W13 = Gerade
//
void ShadowYardController::applyEntryWeichen()
{
    switch (m_targetGleis)
    {
        case 1:
            if (m_w12) m_w12->setAbzweig();
            break;

        case 2:
            if (m_w12) m_w12->setGerade();
            if (m_w13) m_w13->setAbzweig();
            break;

        case 3:
            if (m_w12) m_w12->setGerade();
            if (m_w13) m_w13->setGerade();
            break;
    }
}

// ----------------------------------------------------------
// AUSFAHRTWEICHEN gemäß korrigierter Logik
// ----------------------------------------------------------
//
// Gleis 1:
//   W14 = Gerade
//   W15 = Gerade
//
// Gleis 2:
//   W14 = Abzweig
//   W15 = Gerade
//
// Gleis 3:
//   W14 = egal
//   W15 = Abzweig
//
void ShadowYardController::applyExitWeichen()
{
    switch (m_exitGleis)
    {
        case 1:
            if (m_w14) m_w14->setGerade();
            if (m_w15) m_w15->setGerade();
            break;

        case 2:
            if (m_w14) m_w14->setAbzweig();
            if (m_w15) m_w15->setGerade();
            break;

        case 3:
            // W14 irrelevant
            if (m_w15) m_w15->setAbzweig();
            break;
    }
}

// ----------------------------------------------------------
// Platzhalter: Nothalt-Kontakt
// ----------------------------------------------------------
//
// Diese Funktion muss von dir später mit dem realen Nothalt-Kontakt
// verknüpft werden (z. B. SensorKontakt oder eigener Block).
//
// Solange sie false zurückgibt, wird kein ErrorNothalt ausgelöst.
//
bool ShadowYardController::isNothaltKontaktAktiv() const
{
    // TODO: Hier echten Nothalt-Sensor / -Block einhängen
    return false;
}
