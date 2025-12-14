#include "ShadowYardController.h"

#include "BlockController.h"
#include "Weiche.h"
#include "Mega2PowerControl.h"
#include "mega2_debug.h"

// Globale Power-Control-Instanz
extern Mega2PowerControl g_power;

// Weichen (extern aus main.cpp)
extern Weiche w12;
extern Weiche w13;
extern Weiche w14;
extern Weiche w15;

// ------------------------------------------------------------
// Weichen-Timeout Parameter (D2)
// ------------------------------------------------------------
static constexpr uint32_t WEICHE_MIN_CHECK_MS = 500;
static constexpr uint32_t WEICHE_TIMEOUT_MS  = 2500;

// ------------------------------------------------------------
// ctor / begin
// ------------------------------------------------------------
ShadowYardController::ShadowYardController(BlockController* bc)
: m_bc(bc)
{
}

void ShadowYardController::begin()
{
    m_state           = SBhfState::Idle;
    m_mode            = SbhfMode::Sequential;

    m_currentGleis    = 0;
    m_nextGleis       = 0;

    m_weichenCount    = 0;
    m_weichenIndex    = 0;
    m_phaseStartMs    = 0;

    m_errorActive     = false;
    m_exitPowerOn     = false;
    m_nothaltActive   = false;
    m_entryArmed      = false;

    m_lastError       = HardErrorReason::NONE;
    m_lastErrorWeiche = 0;

    DBG_PRINTLN("[SBHF] init");
}

// ------------------------------------------------------------
// update
// ------------------------------------------------------------
void ShadowYardController::update(uint32_t nowMs)
{
    // --------------------------------------------------
    // HARD-ERROR LOCK (robust)
    // --------------------------------------------------
    if (m_state == SBhfState::Error)
        return;

    switch (m_state)
    {
        case SBhfState::Idle:
            break;

        case SBhfState::PrepareExit:
            startWeichenSequence(nowMs);
            break;

        case SBhfState::SettingWeichen:
            processWeichenSequence(nowMs);
            break;

        case SBhfState::WaitBlock6:
            if (m_bc && !m_bc->isOccupied(6))
            {
                DBG_PRINTLN("[SBHF] Block 6 frei -> ExitRunning");
                g_power.setBlock5ToSBhf(false);
                g_power.setSbhfGleis(m_currentGleis, true);
                m_exitPowerOn = true;
                m_state = SBhfState::ExitRunning;
            }
            break;

        case SBhfState::ExitRunning:
            break;

        case SBhfState::Error:
            // wird durch Guard oben abgefangen
            break;
    }
}

// ------------------------------------------------------------
// Events S11–S16 (Hard-Error-Lock)
// ------------------------------------------------------------
void ShadowYardController::onS11()
{
    if (m_state != SBhfState::Idle)
    {
        return;
    }

    m_currentGleis = pickNextGleis();
    DBG_PRINT("[SBHF] S11 -> Gleis ");
    DBG_PRINTLN(m_currentGleis);

    buildWeichenPlan(m_currentGleis);
    m_entryArmed = false;
    m_state = SBhfState::PrepareExit;
}


void ShadowYardController::onS12()
{
    if (m_state == SBhfState::Error)
    {
        return;
    }

    if (!m_entryArmed)
    {
        return;
    }

    if (m_currentGleis == 1)
    {
        g_power.setSbhfGleis(1, false);
        m_state = SBhfState::Idle;
    }
}

void ShadowYardController::onS13()
{
    if (m_state == SBhfState::Error)
    {
        return;
    }

    if (!m_entryArmed)
    {
        return;
    }

    if (m_currentGleis == 2)
    {
        g_power.setSbhfGleis(2, false);
        m_state = SBhfState::Idle;
    }
}

void ShadowYardController::onS14()
{
    if (m_state == SBhfState::Error)
    {
        return;
    }

    if (!m_entryArmed)
    {
        return;
    }

    if (m_currentGleis == 3)
    {
        g_power.setSbhfGleis(3, false);
        m_state = SBhfState::Idle;
    }
}

void ShadowYardController::onS15()
{
    if (m_state == SBhfState::Error)
    {
        return;
    }

    m_entryArmed = true;
    g_power.setNothalt(true);
}

void ShadowYardController::onS16()
{
    if (m_state == SBhfState::Error)
    {
        return;
    }

    DBG_PRINTLN("[SBHF] NOTHALT -> HARD ERROR");
    g_power.setNothalt(false);
    triggerHardError(HardErrorReason::NOTHALT_TRIGGERED);
}

// ------------------------------------------------------------
// Gleiswahl
// ------------------------------------------------------------
uint8_t ShadowYardController::pickNextGleis()
{
    m_nextGleis = (m_nextGleis % 3) + 1;
    return m_nextGleis;
}

uint8_t ShadowYardController::pickRandomGleisNoRepeat(uint8_t last)
{
    uint8_t g;
    do {
        g = random(1, 4);
    } while (g == last);
    return g;
}

// ------------------------------------------------------------
// Weichenplan
// ------------------------------------------------------------
void ShadowYardController::buildWeichenPlan(uint8_t gleis)
{
    m_weichenCount = 0;

    auto add = [&](Weiche* w, bool abzweig)
    {
        m_weichen[m_weichenCount] = w;
        m_weichenSollAbzweig[m_weichenCount] = abzweig;
        m_weichenCount++;
    };

    if (gleis == 1)
    {
        add(&w12, true);
        add(&w14, false);
        add(&w15, false);
    }
    else if (gleis == 2)
    {
        add(&w12, false);
        add(&w13, true);
        add(&w14, true);
        add(&w15, false);
    }
    else if (gleis == 3)
    {
        add(&w12, false);
        add(&w13, false);
        add(&w15, true);
    }
}

// ------------------------------------------------------------
// Weichen-Sequenz (Soll/Ist + Timeout)
// ------------------------------------------------------------
void ShadowYardController::startWeichenSequence(uint32_t nowMs)
{
    m_weichenIndex = 0;
    m_phaseStartMs = nowMs;
    m_state = SBhfState::SettingWeichen;

    if (m_weichenSollAbzweig[0])
        m_weichen[0]->setAbzweig();
    else
        m_weichen[0]->setGerade();
}

void ShadowYardController::processWeichenSequence(uint32_t nowMs)
{
    Weiche* w = m_weichen[m_weichenIndex];
    bool sollAbzweig = m_weichenSollAbzweig[m_weichenIndex];

    uint32_t elapsed = nowMs - m_phaseStartMs;

    if (elapsed < WEICHE_MIN_CHECK_MS)
        return;

    bool istAbzweig = w->isAbzweigIst();

    if (istAbzweig == sollAbzweig)
    {
        DBG_PRINT("[SBHF] Weiche OK Index=");
        DBG_PRINTLN(m_weichenIndex);

        m_weichenIndex++;

        if (m_weichenIndex >= m_weichenCount)
        {
            DBG_PRINTLN("[SBHF] Alle Weichen OK -> WaitBlock6");
            m_state = SBhfState::WaitBlock6;
            return;
        }

        Weiche* next = m_weichen[m_weichenIndex];
        bool abzweig = m_weichenSollAbzweig[m_weichenIndex];

        if (abzweig)
            next->setAbzweig();
        else
            next->setGerade();

        m_phaseStartMs = nowMs;
        return;
    }

    if (elapsed < WEICHE_TIMEOUT_MS)
        return;

    DBG_PRINT("[SBHF] WEICHE TIMEOUT Index=");
    DBG_PRINTLN(m_weichenIndex);

    triggerHardError(
        HardErrorReason::WEICHE_TIMEOUT,
        m_weichenIndex
    );
}

// ------------------------------------------------------------
// Fehler + Reset
// ------------------------------------------------------------
void ShadowYardController::triggerHardError(
    HardErrorReason reason,
    uint8_t weichenId
)
{
    DBG_PRINT("[SBHF] HARD ERROR, reason=");
    DBG_PRINTLN((uint8_t)reason);

    m_lastError = reason;
    m_lastErrorWeiche = weichenId;

    g_power.emergencyShutdown();

    m_errorActive = true;
    m_state = SBhfState::Error;
}

void ShadowYardController::resetError()
{
    DBG_PRINTLN("[SBHF] RESET ERROR");

    m_errorActive     = false;
    m_lastError       = HardErrorReason::NONE;
    m_lastErrorWeiche = 0;

    m_weichenCount = 0;
    m_weichenIndex = 0;
    m_phaseStartMs = 0;

    m_entryArmed   = false;
    m_exitPowerOn  = false;
    m_nothaltActive = false;

    m_state = SBhfState::Idle;
}
