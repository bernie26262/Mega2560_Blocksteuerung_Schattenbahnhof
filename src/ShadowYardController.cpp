#include "ShadowYardController.h"

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
: m_bc(bc),
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

void ShadowYardController::begin()
{
    m_state = SBhfState::Idle;
    m_cycleActive = false;
    m_nothaltAktiv = false;
}

void ShadowYardController::update(uint32_t now)
{
    handleNothalt();
    handleS15ForEntry();
    handleEntrySensors();
    handleNewArrival();
}

//
// 1. Nothalt / Fehlerbedingung
//
void ShadowYardController::handleNothalt()
{
    if (m_s15 && m_s15->fellEdge())
    {
        m_power->setNothalt(true);
        m_nothaltAktiv = true;
    }

    if (m_s16 && m_s16->fellEdge())
    {
        m_power->setNothalt(false);
        m_nothaltAktiv = false;

        // Fehlerbedingung prüfen: S16 + Kontaktgleis + kein Stromfluss
        if (checkForRealErrorCondition())
            m_state = SBhfState::Error;
    }
}

bool ShadowYardController::checkForRealErrorCondition()
{
    Block* b6 = m_bc->block(6);

    if (!b6)
        return false;

    bool kontakt = b6->besetzt();
    bool strom = b6->strom_mA() > 20;

    return (kontakt && !strom);
}

//
// 2. Neuer Zug in Block 5
//
void ShadowYardController::handleNewArrival()
{
    if (!m_s11 || !m_s11->fellEdge())
        return;

    m_power->setBlock5ToSBhf(false);

    Block* b6 = m_bc->block(6);
    if (b6 && b6->besetzt())
    {
        m_state = SBhfState::Blocked;
        m_cycleActive = false;
        return;
    }

    chooseGleis();
    startExitAndPrepareEntry();

    m_cycleActive = true;
    m_state = SBhfState::CycleActive;
}

//
// 3. Gleiswahl
//
void ShadowYardController::chooseGleis()
{
    bool g1 = m_bc->block(7)->besetzt();
    bool g2 = m_bc->block(8)->besetzt();
    bool g3 = m_bc->block(9)->besetzt();

    if (!g1 && !g2 && !g3)
    {
        m_state = SBhfState::Idle;
        m_cycleActive = false;
        return;
    }

    if (m_mode == SBhfMode::Serial)
    {
        m_counter = (m_counter % 3) + 1;
        m_exitGleis = m_counter;
        m_targetGleis = m_counter;
        return;
    }

    uint8_t used[3];
    uint8_t n = 0;
    if (g1) used[n++] = 1;
    if (g2) used[n++] = 2;
    if (g3) used[n++] = 3;

    uint8_t idx = random(0, n);
    m_exitGleis = used[idx];
    m_targetGleis = used[idx];
}

//
// 4. Start der Ausfahrt
//
void ShadowYardController::startExitAndPrepareEntry()
{
    applyEntryWeichen();
    applyExitWeichen();

    m_power->setSbhfGleis(m_exitGleis, true);
}

//
// 5. Einfahr-Weichen
//
void ShadowYardController::applyEntryWeichen()
{
    switch (m_targetGleis)
    {
        case 1:
            m_w12->setAbzweig();
            break;
        case 2:
            m_w12->setGerade();
            m_w13->setAbzweig();
            break;
        case 3:
            m_w12->setGerade();
            m_w13->setGerade();
            break;
    }
}

//
// 6. Ausfahr-Weichen
//
void ShadowYardController::applyExitWeichen()
{
    switch (m_exitGleis)
    {
        case 1:
            m_w14->setGerade();
            m_w15->setGerade();
            break;
        case 2:
            m_w14->setAbzweig();
            m_w15->setGerade();
            break;
        case 3:
            m_w15->setAbzweig();
            break;
    }
}

//
// 7. S12/S13/S14 = Einfahrziel erreicht
//
void ShadowYardController::handleEntrySensors()
{
    if (!m_cycleActive)
        return;

    if (m_targetGleis == 1 && m_s12->fellEdge())
    {
        m_power->setSbhfGleis(1, false);
        m_cycleActive = false;
        m_state = SBhfState::Idle;
    }

    if (m_targetGleis == 2 && m_s13->fellEdge())
    {
        m_power->setSbhfGleis(2, false);
        m_cycleActive = false;
        m_state = SBhfState::Idle;
    }

    if (m_targetGleis == 3 && m_s14->fellEdge())
    {
        m_power->setSbhfGleis(3, false);
        m_cycleActive = false;
        m_state = SBhfState::Idle;
    }
}

//
// 8. S15 = Einfahrstrom wieder aktivieren
//
void ShadowYardController::handleS15ForEntry()
{
    if (!m_cycleActive)
        return;

    if (m_s15->fellEdge())
        m_power->setBlock5ToSBhf(true);
}
