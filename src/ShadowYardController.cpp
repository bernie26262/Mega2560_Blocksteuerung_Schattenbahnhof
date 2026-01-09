#include "ShadowYardController.h"

#include "BlockController.h"
#include "Weiche.h"
#include "safety.h"
#include "Mega2PowerControl.h"
#include "mega2_debug.h"
#include "safety_error.h"

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
  m_resumePending(false),
  m_resumeGleis(0),
  m_resumeState(SBhfState::Idle)
{
}

void ShadowYardController::begin()
{
    m_state = SBhfState::Idle;
    m_currentGleis = 0;
    m_nextGleis = 1;
    m_errorActive = false;
    m_exitPowerOn = false;


    m_resumePending = false;
    m_resumeGleis = 0;
    m_resumeState = SBhfState::Idle;

    m_weichenCount = 0;
    m_weichenIndex = 0;
    m_wphase = WPhase::Idle;
    m_phaseStartMs = 0;
}

// ============================================================
// Weichen-Status Getter für Proto
// ============================================================

bool ShadowYardController::weicheIst(uint8_t idx) const
{
    if (idx >= m_weichenCount) return false;
    if (!m_weichen[idx]) return false;
    return m_weichen[idx]->rueckmeldungAbbiegen();
}

bool ShadowYardController::weicheSoll(uint8_t idx) const
{
    if (idx >= m_weichenCount) return false;
    return m_weichenSollAbzweig[idx];
}

bool ShadowYardController::isSafetyBlocked() const
{
    // Gate für SBhf-Sensor-Events:
    // - SBhf im Error
    // - Selftest läuft
    // - globaler Safety-Lock / NOTAUS
    return (m_state == SBhfState::Error) ||
           m_errorActive ||
           m_selftestActive ||
           safetyIsLocked() ||
           safetyIsEmergencyActive();
}


// ============================================================
// Events
// ============================================================

void ShadowYardController::onS11()
{
    if (m_state == SBhfState::Error)
    {
        DBG_PRINTLN("[SBHF] S11 ignored because SBhfState::Error");
        return;
    }

    if (safetyIsLocked() || safetyIsEmergencyActive() || m_selftestActive)
    {
        DBG_PRINTLN("[SBHF] S11 ignored (SAFETY-LOCK)");
        return;
    }

    if (m_state != SBhfState::Idle || m_errorActive)
        return;

    m_currentGleis = pickNextGleis();
    buildWeichenPlan(m_currentGleis);
    m_state = SBhfState::PrepareExit;
}

void ShadowYardController::onS12()
{
    if (m_state == SBhfState::Error)
    {
        DBG_PRINTLN("[SBHF] S12 ignored (ERROR-LOCK)");
        return;
    }

    if (m_state == SBhfState::ExitRunning && m_currentGleis == 1)
    {
        g_power.setSbhfGleis(1, false);
        m_exitPowerOn = false;

        m_state = SBhfState::Idle;
    }
}

void ShadowYardController::onS13()
{
    if (m_state == SBhfState::Error)
    {
        DBG_PRINTLN("[SBHF] S13 ignored (ERROR-LOCK)");
        return;
    }

    if (m_state == SBhfState::ExitRunning && m_currentGleis == 2)
    {
        g_power.setSbhfGleis(2, false);
        m_exitPowerOn = false;

        m_state = SBhfState::Idle;
    }
}

void ShadowYardController::onS14()
{
    if (m_state == SBhfState::Error)
    {
        DBG_PRINTLN("[SBHF] S14 ignored (ERROR-LOCK)");
        return;
    }

    if (m_state == SBhfState::ExitRunning && m_currentGleis == 3)
    {
        g_power.setSbhfGleis(3, false);
        m_exitPowerOn = false;

        m_state = SBhfState::Idle;
    }
}

void ShadowYardController::onS15()
{
    if (m_state == SBhfState::Error)
    {
        DBG_PRINTLN("[SBHF] S15 ignored (ERROR-LOCK)");
        return;
    }

    // S15: Nothaltgleis EIN (Freigabe)
    g_power.setNothalt(false);
    DBG_PRINTLN("[SBHF] S15 -> Nothalt frei (Gleis EIN)");
}

void ShadowYardController::onS16()
{
    if (m_state == SBhfState::Error)
    {
        DBG_PRINTLN("[SBHF] S16 ignored (ERROR-LOCK)");
        return;
    }

    // S16: Nothaltgleis AUS (Stopzone scharf)
    g_power.setNothalt(true);
    DBG_PRINTLN("[SBHF] S16 -> Nothalt aktiv (Gleis AUS)");

    // KEIN Hard-Error hier!
}

// ============================================================
// Update
// ============================================================

void ShadowYardController::update(uint32_t nowMs)
{
    if (m_selftestActive)
    {
        selftestUpdate(nowMs);
        return;
    }

    // SafetyLock / NOTAUS: SBHF-Automat darf keine Aktionen durchführen
    if (safetyIsLocked() || safetyIsEmergencyActive())
        return;


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
    if (m_allowedMask == 0)
        return 0; // kein sicherer Pfad ableitbar

    auto isAllowed = [&](uint8_t g) -> bool
    {
        if (g < 1 || g > 3) return false;
        return (m_allowedMask & (1 << (g - 1))) != 0;
    };

    if (m_mode == SbhfMode::Sequential)
    {
        // Nächster Index, aber nur erlaubte Gleise wählen
        for (uint8_t tries = 0; tries < 3; tries++)
        {
            uint8_t g = m_nextGleis;
            if (g < 1 || g > 3) g = 1;

            // Next pointer rotieren
            m_nextGleis = (g % 3) + 1;

            if (isAllowed(g))
                return g;
        }

        // Fallback: erstes erlaubtes Gleis
        for (uint8_t g = 1; g <= 3; g++)
            if (isAllowed(g)) return g;

        return 0;
    }

    // Random
    return pickRandomGleisNoRepeat(m_currentGleis);
}


uint8_t ShadowYardController::pickRandomGleisNoRepeat(uint8_t last)
{
    uint8_t allowed[3];
    uint8_t cnt = 0;
    for (uint8_t g = 1; g <= 3; g++)
    {
        if (m_allowedMask & (1 << (g - 1)))
            allowed[cnt++] = g;
    }

    if (cnt == 0)
        return 0;

    if (cnt == 1)
        return allowed[0];

    // Wenn möglich: nicht das gleiche Gleis wie zuvor
    for (uint8_t tries = 0; tries < 8; tries++)
    {
        uint8_t g = allowed[random(0, cnt)];
        if (g != last) return g;
    }

    return allowed[0];
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
        if (m_weichenCount >= MAX_WEICHEN) return;
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
        add(&w15, false);   // <- fehlte vorher, jetzt konsistent
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

            // Noch zu früh → Mechanik hat Zeit
            if (elapsed < WEICHE_MIN_CHECK_MS)
                return;

            bool istAbbiegen  = w->rueckmeldungAbbiegen();
            bool sollAbbiegen = (w->getStellung() == Weiche::ABBIEGEN);

            // Erfolg → nächste Weiche
            if (istAbbiegen == sollAbbiegen)
            {
                m_weichenIndex++;
                m_wphase = WPhase::Idle;
                return;
            }

            // Noch innerhalb Timeout → weiter warten
            if (elapsed < WEICHE_TIMEOUT_MS)
                return;

            const uint8_t wid = w->id();

            // Timeout → bei W12/W13: HARD ERROR, bei W14/W15: Warning und weiter
            if (isCriticalWeiche(wid))
            {
                triggerHardError(wid);
                return;
            }

            setWarningForWeiche(wid);
            DBG_PRINTF("[SBHF] Weiche %d soft-fail (timeout/mismatch) -> WARNING, continue\n", wid);

            m_weichenIndex++;
            m_wphase = WPhase::Idle;
            return;
        }
    }
}

// ============================================================
// Fehler / Reset (D3)
// ============================================================

void ShadowYardController::triggerHardError(uint8_t weicheId)
{
    if (m_errorActive)
        return;

    // --------------------------------------------------------
    // Checkpoint: Wenn S11 bereits akzeptiert wurde, merken wir uns den Fortsetzpunkt.
    // Hintergrund: S11 wird flankenbasiert ausgewertet. Steht der Zug nach NOTAUS auf S11,
    // kommt kein neuer Trigger. Dann muss die SBhf-State-Machine nach ACK/Selbsttest fortsetzen können.
    // --------------------------------------------------------
    if (m_state != SBhfState::Idle && m_state != SBhfState::Error && m_currentGleis != 0)
    {
        m_resumePending = true;
        m_resumeGleis   = m_currentGleis;
        // Safest Resume: Weichenplan neu aufbauen und Sequenz neu starten.
        m_resumeState   = SBhfState::PrepareExit;
    }

    m_errorActive = true;
    m_state = SBhfState::Error;

    // Defekt-Bit für Debug/Status setzen (Selftest wird später final entscheiden)
    setWarningForWeiche(weicheId);

    // 🔴 Fehler: SBHF-Weiche (W12/W13 sicherheitskritisch)
    safetyErrorSet(SAFETY_ERR_SBH_WEICHE, weicheId);

    safetySetEmergency(true);
}


bool ShadowYardController::canReset() const
{
    // Normally selftest is started as part of the SBHF emergency/ACK flow
    // while we are in Error state. For an explicit UI "Retry" we also allow
    // running the selftest while we're in WARNING-only mode.
    if (m_state != SBhfState::Error && m_warningMask == 0)
        return false;

    // Kein Reset während Selftest läuft
    if (m_selftestActive)
        return false;

    // Keine aktive Ausfahrt
    if (m_exitPowerOn)
        return false;

    return true;
}

void ShadowYardController::resetError()
{
    m_errorActive   = false;
    m_exitPowerOn   = false;
    m_currentGleis  = 0;

    m_weichenCount  = 0;
    m_weichenIndex  = 0;
    m_wphase        = WPhase::Idle;
    m_phaseStartMs  = 0;
}

void ShadowYardController::onResetAck()
{
    if (!canReset())
    {
        // Aussagekräftiger statt "conditions not met"
        if (m_state != SBhfState::Error)
        {
            DBG_PRINTLN("[SBHF] RESET ignored (not in Error)");
        }
        else if (m_exitPowerOn)
        {
            DBG_PRINTLN("[SBHF] RESET ignored (exit power still on)");
        }
        else
        {
            DBG_PRINTLN("[SBHF] RESET ignored (conditions not met)");
        }
        return;
    }

    DBG_PRINTLN("[SBHF] RESET acknowledged");

    // Resume-Infos sichern, weil resetError() m_currentGleis löscht
    bool resume = m_resumePending && (m_resumeGleis >= 1 && m_resumeGleis <= 3);
    const uint8_t gleis = m_resumeGleis;
    const SBhfState st = m_resumeState;

    // Wenn im eingeschränkten Betrieb das Resume-Gleis nicht erlaubt ist: NICHT fortsetzen
    if (resume && (m_allowedMask != 0) && ((m_allowedMask & (1 << (gleis - 1))) == 0))
        resume = false;

    resetError();

    if (resume)
    {
        m_currentGleis = gleis;
        buildWeichenPlan(m_currentGleis);
        m_state = st;

        // Checkpoint verbrauchen
        m_resumePending = false;
        m_resumeGleis = 0;
        m_resumeState = SBhfState::Idle;

        DBG_PRINTLN("[SBHF] Resuming after reset from checkpoint");
    }
    else
    {
        m_state = SBhfState::Idle;
    }
}



bool ShadowYardController::isCriticalWeiche(uint8_t weicheId) const
{
    return (weicheId == 12) || (weicheId == 13);
}

void ShadowYardController::setWarningForWeiche(uint8_t weicheId)
{
    switch (weicheId)
    {
        case 12: m_warningMask |= SBHF_WARN_W12_DEFECT; break;
        case 13: m_warningMask |= SBHF_WARN_W13_DEFECT; break;
        case 14: m_warningMask |= SBHF_WARN_W14_DEFECT; break;
        case 15: m_warningMask |= SBHF_WARN_W15_DEFECT; break;
        default: break;
    }
}

bool ShadowYardController::startSelftest(bool includeNonCritical)
{
    if (m_selftestActive)
        return false;

    if (m_state != SBhfState::Error)
        return false;

    // Reset derived status; wird am Ende neu berechnet
    m_allowedMask = 0x07;
    m_warningMask = 0x00;

    m_selftestActive = true;
    m_selftestDone   = false;
    m_selftestIncludeNonCritical = includeNonCritical;

    m_selftestWeicheIdx = 0;
    m_selftestStep = 0;
    m_selftestStepStartMs = 0;

    for (uint8_t i = 0; i < 4; i++)
    {
        m_stOk[i] = false;
        m_stKnown[i] = false;
        m_stPos[i] = 0;
        m_stChk[i] = 0;
    }

    DBG_PRINTLN("[SBHF] Selftest started");
    return true;
}

void ShadowYardController::selftestUpdate(uint32_t nowMs)
{
    static constexpr uint32_t SELFTEST_SETTLE_MS = 1200;

    Weiche* const weichen[4] = { &w12, &w13, &w14, &w15 };
    const uint8_t maxWeichen = m_selftestIncludeNonCritical ? 4 : 2;

    if (m_selftestWeicheIdx >= maxWeichen)
    {
        selftestFinish();
        return;
    }

    Weiche* w = weichen[m_selftestWeicheIdx];
    const uint8_t idx = m_selftestWeicheIdx;

    // Step 0: Gerade anfahren
    if (m_selftestStep == 0)
    {
        w->setGerade();
        m_selftestStepStartMs = nowMs;
        m_selftestStep = 1;
        return;
    }

    // Step 1: Gerade settle & prüfen, dann Abbiegen anfahren
    if (m_selftestStep == 1)
    {
        if (nowMs - m_selftestStepStartMs < SELFTEST_SETTLE_MS)
            return;

        const bool rmAbbiegen = w->rueckmeldungAbbiegen();
        if (!rmAbbiegen) m_stChk[idx] |= 0x01; // Gerade ok

        w->setAbzweig();
        m_selftestStepStartMs = nowMs;
        m_selftestStep = 2;
        return;
    }

    // Step 2: Abbiegen settle & prüfen, Ergebnis auswerten, nächste Weiche
    if (m_selftestStep == 2)
    {
        if (nowMs - m_selftestStepStartMs < SELFTEST_SETTLE_MS)
            return;

        const bool rmAbbiegen = w->rueckmeldungAbbiegen();
        if (rmAbbiegen) m_stChk[idx] |= 0x02; // Abbiegen ok

        const bool ok = ((m_stChk[idx] & 0x03) == 0x03);
        m_stOk[idx] = ok;

        if (ok)
        {
            // Letzter Befehl war Abbiegen
            m_stKnown[idx] = true;
            m_stPos[idx]   = 1;
        }
        else
        {
            // Wir nehmen den aktuellen RM-Zustand als "bekannte" Stellung (stuck).
            // (Wenn der RM-Sensor defekt ist, bleibt das dennoch konservativ durch allowedMask-Logik.)
            m_stKnown[idx] = true;
            m_stPos[idx]   = rmAbbiegen ? 1 : 0;
        }

        m_selftestWeicheIdx++;
        m_selftestStep = 0;
        m_selftestStepStartMs = nowMs;
        return;
    }
}

void ShadowYardController::selftestFinish()
{
    // Warnings für defekte Weichen setzen
    if (!m_stOk[0]) m_warningMask |= SBHF_WARN_W12_DEFECT;
    if (!m_stOk[1]) m_warningMask |= SBHF_WARN_W13_DEFECT;

    if (m_selftestIncludeNonCritical)
    {
        if (!m_stOk[2]) m_warningMask |= SBHF_WARN_W14_DEFECT;
        if (!m_stOk[3]) m_warningMask |= SBHF_WARN_W15_DEFECT;
    }

    // AllowedMask aus W12/W13 ableiten
    m_allowedMask = 0x07;

    const bool w12Ok = m_stOk[0];
    const bool w13Ok = m_stOk[1];

    const bool w12Known = m_stKnown[0];
    const bool w13Known = m_stKnown[1];

    const uint8_t w12Pos = m_stPos[0]; // 0=Gerade,1=Abbiegen
    const uint8_t w13Pos = m_stPos[1];

    if (!w12Ok && !w13Ok)
    {
        // Doppeldefekt
        if (w12Known && w12Pos == 1)
        {
            m_allowedMask = 0x01; // nur Gleis 1 (W13 egal)
        }
        else if (w12Known && w12Pos == 0)
        {
            if (w13Known)
                m_allowedMask = (w13Pos == 1) ? 0x02 : 0x04; // nur Gleis2 oder nur Gleis3
            else
                m_allowedMask = 0x00; // W12=Gerade aber W13 unknown => kein sicherer Pfad
        }
        else
        {
            m_allowedMask = 0x00; // W12 unknown => kein sicherer Pfad
        }
    }
    else if (!w12Ok && w13Ok)
    {
        // Einzeldefekt W12
        if (!w12Known)
            m_allowedMask = 0x00;
        else
            m_allowedMask = (w12Pos == 1) ? 0x01 : 0x06; // W12=Abbiegen -> G1, W12=Gerade -> G2+G3
    }
    else if (w12Ok && !w13Ok)
    {
        // Einzeldefekt W13 (W12 OK => wir können Gleis1 immer anfahren)
        if (!w13Known)
            m_allowedMask = 0x01; // konservativ: nur Gleis 1
        else
            m_allowedMask = (w13Pos == 1) ? 0x03 : 0x05; // W13=Abbiegen -> G1+G2, W13=Gerade -> G1+G3
    }

    if (m_allowedMask != 0x07 && m_allowedMask != 0x00)
        m_warningMask |= SBHF_WARN_RESTRICTED;

    m_selftestActive = false;
    m_selftestDone   = true;

    DBG_PRINTF("[SBHF] Selftest done: allowedMask=0x%02X warnMask=0x%02X\n", m_allowedMask, m_warningMask);
}
