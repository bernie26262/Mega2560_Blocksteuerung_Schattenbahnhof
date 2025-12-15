#include "ShadowYardController.h"

#include "BlockController.h"
#include "Weiche.h"
#include "safety.h"
#include "Mega2PowerControl.h"
#include "mega2_debug.h"

// ============================================================
// Externe Objekte
// ============================================================
extern Weiche w12;
extern Weiche w13;
extern Weiche w14;
extern Weiche w15;

extern Mega2PowerControl g_power;



// ------------------------------------------------------------
// Weichen-Zeiten (D2)
// ------------------------------------------------------------
static constexpr uint32_t WEICHE_IMPULS_MS    = 500;   // Spulenimpuls
static constexpr uint32_t WEICHE_MIN_CHECK_MS = 500;   // frühester Ist-Check
static constexpr uint32_t WEICHE_TIMEOUT_MS   = 2500;  // Hard-Error

// ============================================================

ShadowYardController::ShadowYardController(BlockController* bc)
: m_bc(bc),
  m_state(SBhfState::Idle),
  m_mode(SbhfMode::Sequential),
  m_currentGleis(0),
  m_nextGleis(1),
  m_weichenCount(0),
  m_weichenIndex(0),
  m_wphase(WPhase::Idle),
  m_phaseStartMs(0),
  m_errorActive(false),
  m_exitPowerOn(false),
  m_nothaltActive(false)
{
}

void ShadowYardController::begin()
{
    m_state = SBhfState::Idle;
    m_currentGleis = 0;
    m_nextGleis = 1;
    m_errorActive = false;
    m_exitPowerOn = false;
    m_nothaltActive = false;
}

// ============================================================
// Events
// ============================================================

void ShadowYardController::onS11()
{
    if (m_state != SBhfState::Idle || m_errorActive)
        return;

    m_currentGleis = pickNextGleis();
    buildWeichenPlan(m_currentGleis);
    m_state = SBhfState::PrepareExit;
}

void ShadowYardController::onS12()
{
    if (m_state == SBhfState::ExitRunning && m_currentGleis == 1)
    {
        g_power.setSbhfGleis(1, false);
        m_exitPowerOn = false;
        m_state = SBhfState::Idle;
    }
}

void ShadowYardController::onS13()
{
    if (m_state == SBhfState::ExitRunning && m_currentGleis == 2)
    {
        g_power.setSbhfGleis(2, false);
        m_exitPowerOn = false;
        m_state = SBhfState::Idle;
    }
}

void ShadowYardController::onS14()
{
    if (m_state == SBhfState::ExitRunning && m_currentGleis == 3)
    {
        g_power.setSbhfGleis(3, false);
        m_exitPowerOn = false;
        m_state = SBhfState::Idle;
    }
}

void ShadowYardController::onS15()
{
    // NOT-AUS EIN
    g_power.setNothalt(true);
    m_nothaltActive = true;
}

void ShadowYardController::onS16()
{
    // NOT-AUS AUS -> Hard-Error
    g_power.setNothalt(false);
    m_nothaltActive = true;
    triggerHardError();
}

// ============================================================
// Update
// ============================================================

void ShadowYardController::update(uint32_t nowMs)
{
    if (m_errorActive)
        return;

    switch (m_state)
    {
        case SBhfState::Idle:
            break;

        case SBhfState::PrepareExit:
            startWeichenSequence(nowMs);
            m_state = SBhfState::SettingWeichen;
            break;

        case SBhfState::SettingWeichen:
            processWeichenSequence(nowMs);
            break;

        case SBhfState::WaitBlock6:
            if (!m_bc || !m_bc->canEnter(5, 6))
                break;

            g_power.setBlock5ToSBhf(false);
            g_power.setSbhfGleis(m_currentGleis, true);

            m_exitPowerOn = true;
            m_state = SBhfState::ExitRunning;
            break;

        case SBhfState::ExitRunning:
            break;

        case SBhfState::Error:
            break;
    }
}

// ============================================================
// Gleiswahl
// ============================================================

uint8_t ShadowYardController::pickNextGleis()
{
    if (m_mode == SbhfMode::Sequential)
    {
        uint8_t g = m_nextGleis;
        m_nextGleis = (m_nextGleis % 3) + 1;
        return g;
    }

    return pickRandomGleisNoRepeat(m_currentGleis);
}

uint8_t ShadowYardController::pickRandomGleisNoRepeat(uint8_t last)
{
    uint8_t g;
    do {
        g = random(1, 4);
    } while (g == last);
    return g;
}

// ============================================================
// Weichen
// ============================================================

void ShadowYardController::buildWeichenPlan(uint8_t gleis)
{
    m_weichenCount = 0;
    m_weichenIndex = 0;
    m_wphase = WPhase::Idle;

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
    }
    else if (gleis == 3)
    {
        add(&w12, false);
        add(&w13, false);
        add(&w15, true);
    }
}

void ShadowYardController::startWeichenSequence(uint32_t nowMs)
{
    m_weichenIndex = 0;
    m_wphase = WPhase::Idle;
    m_phaseStartMs = nowMs;
}

void ShadowYardController::processWeichenSequence(uint32_t nowMs)
{
    if (m_weichenIndex >= m_weichenCount)
    {
        m_state = SBhfState::WaitBlock6;
        return;
    }

    Weiche* w = m_weichen[m_weichenIndex];
    bool sollAbzweig = m_weichenSollAbzweig[m_weichenIndex];

    switch (m_wphase)
    {
        // --------------------------------------------------
        // 1) Impuls auslösen
        // --------------------------------------------------
        case WPhase::Idle:
            if (sollAbzweig)
                w->setAbzweig();
            else
                w->setGerade();

            m_wphase = WPhase::Impuls;
            m_phaseStartMs = nowMs;
            break;

        // --------------------------------------------------
        // 2) Impulsdauer abwarten
        // --------------------------------------------------
        case WPhase::Impuls:
            if (nowMs - m_phaseStartMs >= WEICHE_IMPULS_MS)
            {
                // Ab jetzt beginnt die eigentliche Prüfzeit
                m_wphase = WPhase::Check;
                m_phaseStartMs = nowMs;
            }
            break;

        // --------------------------------------------------
        // 3) Soll / Ist mit Zeitfenster prüfen
        // --------------------------------------------------
        case WPhase::Check:
        {
            uint32_t elapsed = nowMs - m_phaseStartMs;

            // 3.1 Noch zu früh → Mechanik hat Zeit
            if (elapsed < WEICHE_MIN_CHECK_MS)
                return;

            bool istAbbiegen  = w->rueckmeldungAbbiegen();
            bool sollAbbiegen = (w->getStellung() == Weiche::ABBIEGEN);

            // 3.2 Erfolg → nächste Weiche
            if (istAbbiegen == sollAbbiegen)
            {
                m_weichenIndex++;
                m_wphase = WPhase::Idle;
                return;
            }

            // 3.3 Noch innerhalb Timeout → weiter warten
            if (elapsed < WEICHE_TIMEOUT_MS)
                return;

            // 3.4 Timeout → HARD ERROR
            triggerHardError();
            return;
        }
    }
}


// ============================================================
// Fehler / Reset (D3)
// ============================================================

void ShadowYardController::triggerHardError()
{
    if (m_errorActive)
        return;

    m_errorActive = true;
    m_state = SBhfState::Error;
    m_nothaltActive = true;     // ← 🔧 FEHLTE

    // HART: komplette Anlage stromlos
    safetySetEmergency(true);
}

bool ShadowYardController::canReset() const
{
    if (m_state != SBhfState::Error)
        return false;

    if (!m_nothaltActive)
        return false;

    if (m_exitPowerOn)
        return false;

    return true;
}

void ShadowYardController::resetError()
{
    m_errorActive   = false;
    m_exitPowerOn  = false;
    m_currentGleis = 0;
}

void ShadowYardController::onResetAck()
{
    if (!canReset())
    {
        DBG_PRINTLN("[SBHF] RESET ignored");
        return;
    }

    DBG_PRINTLN("[SBHF] RESET acknowledged");

    resetError();
    m_state = SBhfState::Idle;
}
