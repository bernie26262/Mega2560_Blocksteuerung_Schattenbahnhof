#include "ShadowYardController.h"
#include "mega2_debug.h"

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

    // Startup-Checklist Flags: nach jedem Boot deterministisch
    m_selftestActive = false;
    m_selftestDone   = false;

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
        DBG_PRINTLN(F("[SBHF] S11 ignored because SBhfState::Error"));
        return;
    }

    if (safetyIsLocked() || safetyIsEmergencyActive() || m_selftestActive)
    {
        DBG_PRINTLN(F("[SBHF] S11 ignored (SAFETY-LOCK)"));
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
        DBG_PRINTLN(F("[SBHF] S12 ignored (ERROR-LOCK)"));
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
        DBG_PRINTLN(F("[SBHF] S13 ignored (ERROR-LOCK)"));
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
        DBG_PRINTLN(F("[SBHF] S14 ignored (ERROR-LOCK)"));
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
        DBG_PRINTLN(F("[SBHF] S15 ignored (ERROR-LOCK)"));
        return;
    }

    // S15: Nothaltgleis EIN (Freigabe)
    g_power.setNothalt(false);
    DBG_PRINTLN(F("[SBHF] S15 -> Nothalt frei (Gleis EIN)"));
}

void ShadowYardController::onS16()
{
    if (m_state == SBhfState::Error)
    {
        DBG_PRINTLN(F("[SBHF] S16 ignored (ERROR-LOCK)"));
        return;
    }

    // S16: Nothaltgleis AUS (Stopzone scharf)
    g_power.setNothalt(true);
    DBG_PRINTLN(F("[SBHF] S16 -> Nothalt aktiv (Gleis AUS)"));

    // KEIN Hard-Error hier!
}

// ============================================================
// Update
// ============================================================

void ShadowYardController::update(uint32_t nowMs)
{
    // ------------------------------------------------------------
    // Weichen immer zyklisch updaten:
    // - beendet aktive Impulse nach PULSE_MS
    // - setzt danach COOLDOWN / später IDLE
    // Ohne diese Updates taktet Weiche::schalte() die Ausgänge nicht sauber.
    // ------------------------------------------------------------
    w12.update(nowMs);
    w13.update(nowMs);
    w14.update(nowMs);
    w15.update(nowMs);

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
            DBG_PRINTLN(F("[SBHF] RESET ignored (not in Error)"));
        }
        else if (m_exitPowerOn)
        {
            DBG_PRINTLN(F("[SBHF] RESET ignored (exit power still on)"));
        }
        else
        {
            DBG_PRINTLN(F("[SBHF] RESET ignored (conditions not met)"));
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

        DBG_PRINTLN(F("[SBHF] Resuming after reset from checkpoint"));
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

bool ShadowYardController::startSelftestImpl(bool includeNonCritical, bool allowFromCleanIdle, bool allowDuringEmergency)
{
    // HART: niemals parallel
    if (m_selftestActive)
        return false;

    // HART: niemals bei aktivem HW-NOT-AUS
    if (safetyIsEmergencyActive() && !allowDuringEmergency)
        return false;

    // Safety-Lock: im Normalbetrieb sperren, aber für Startup-Checklist Selftest erlauben
    if (!allowFromCleanIdle && safetyIsLocked())
        return false;

    // Selftest darf starten:
    // - nach Hard-Error (klassischer Flow)
    // - oder im Idle, wenn es Warnings/Restriktionen gibt (UI Retry im eingeschränkten Betrieb)
    // - oder (Startup-Checklist) im Idle auch ohne Warnungen/Restriktionen (z.B. Boot-ERR / SYS_ERROR_PRESENT)
    if (m_state != SBhfState::Error)
    {
        if (m_state != SBhfState::Idle)
            return false;

        if (!allowFromCleanIdle)
        {
            // Im Idle nur sinnvoll, wenn wirklich Warnungen/Restriktionen vorliegen
            if (m_warningMask == 0 && m_allowedMask == 0x07)
                return false;
        }
    }

    // Reset derived status; wird am Ende neu berechnet
    m_allowedMask = 0x07;
    m_warningMask = 0x00;
    // ------------------------------------------------------------
    // Safety: Während Selftest läuft darf kein Zug fahren.
    // => MainPower/SSR AUS. (Der User schaltet nach erfolgreichem Selftest
    //    bewusst wieder ein.)
    // Zusätzlich: SBHF-Abgänge/Relais definiert AUS, um Altlasten zu vermeiden.
    // ------------------------------------------------------------
    if (g_power.isMainPowerOn())
        DBG_PRINTLN(F("[SBHF] Selftest: main power ON -> turning OFF"));
    g_power.setMainPower(false);
    m_selftestForcedPowerOff = true;

    // Definiert AUS: SBHF-Abgänge & Verbindung Block5->SBHF
    g_power.setSbhfGleis(1, false);
    g_power.setSbhfGleis(2, false);
    g_power.setSbhfGleis(3, false);
    g_power.setBlock5ToSBhf(false);
    m_exitPowerOn = false;


    m_selftestActive = true;
    m_selftestDone   = false;
    m_selftestIncludeNonCritical = includeNonCritical;

    m_selftestPhase = ST_GERADE;
    m_selftestNextIdx = 0;

    // NEU: Puls-Sequencer Reset
    m_stPulseActive  = false;
    m_stPulseStartMs = 0;
    m_stPulseIdx     = 0;

    for (uint8_t i = 0; i < 4; i++)
    {
        m_stOk[i] = false;
        m_stKnown[i] = false;
        m_stPos[i] = 0;
        m_stChk[i] = 0;

        // Pipeline-Zustand reset
        m_stT[i].issuedGerade = false;
        m_stT[i].checkedGerade = false;
        m_stT[i].okGerade = false;
        m_stT[i].dueGeradeMs = 0;
        m_stT[i].expGeradeBit = 0xFF;

        m_stT[i].issuedAbbiegen = false;
        m_stT[i].checkedAbbiegen = false;
        m_stT[i].okAbbiegen = false;
        m_stT[i].dueAbbiegenMs = 0;
        m_stT[i].expAbbiegenBit = 0xFF;
    }

    DBG_PRINTLN(F("[SBHF] Selftest started"));
    m_stLoggedThisRun = false;   // pro Selftest neu loggen
    DBG_PRINTF("[SBHF] ST pipeline init: includeNonCritical=%u\n", (unsigned)m_selftestIncludeNonCritical);
    
    return true;
}

bool ShadowYardController::startSelftest(bool includeNonCritical)
{
    // normaler Flow: Idle nur wenn Warnungen/Restriktionen vorliegen
    return startSelftestImpl(includeNonCritical, false, false);
}

bool ShadowYardController::startSelftestStartup(bool includeNonCritical)
{
    // Startup-Checklist: erlaubt Start auch aus sauberem Idle (z.B. Boot-ERR)
    return startSelftestImpl(includeNonCritical, true, false);
}

bool ShadowYardController::startSelftestRetry(bool includeNonCritical)
{
    // UI Retry (nach Weichenfehler) darf auch unter NOTAUS/LOCK starten,
    // sonst entsteht ein Deadlock (ACK blockt Selftest, Selftest blockt ACK).
    // Wir erlauben das aber nur, wenn tatsächlich ein Weichen-Defekt-Warning aktiv ist.
    const uint8_t weicheWarn =
        SBHF_WARN_W12_DEFECT |
        SBHF_WARN_W13_DEFECT |
        SBHF_WARN_W14_DEFECT |
        SBHF_WARN_W15_DEFECT;

    if ((m_warningMask & weicheWarn) == 0)
        return false;

    return startSelftestImpl(includeNonCritical, true, true);
}


void ShadowYardController::selftestUpdate(uint32_t nowMs)
{
    static constexpr uint32_t SELFTEST_PULSE_MS  = 500; // wie gewünscht
    static constexpr uint32_t SELFTEST_SETTLE_MS = 500; // wie gewünscht (1x Sample am Ende)

    if (!m_stLoggedThisRun)
    {
        m_stLoggedThisRun = true;
        DBG_PRINTF("[SBHF] ST pipeline: pulse=%lums settle=%lums maxWeichen=%u\n",
                   (unsigned long)SELFTEST_PULSE_MS,
                   (unsigned long)SELFTEST_SETTLE_MS,
                   (unsigned)(m_selftestIncludeNonCritical ? 4 : 2));
    }

    Weiche* const weichen[4] = { &w12, &w13, &w14, &w15 };
    const uint8_t maxWeichen = m_selftestIncludeNonCritical ? 4 : 2;

    // ------------------------------------------------------------
    // 1) Auswerten: alles, was fällig ist, wird geprüft (parallel)
    //    RM: 1=Gerade, 0=Abbiegen  (wir loggen das explizit als rmBit)
    // ------------------------------------------------------------
    for (uint8_t i = 0; i < maxWeichen; i++)
    {
        Weiche* w = weichen[i];
        auto &st = m_stT[i];

        // Gerade prüfen
        if (st.issuedGerade && !st.checkedGerade && (nowMs >= st.dueGeradeMs))
        {
            const bool rmAb = w->rueckmeldungAbbiegen();      // semantisch: true=Abbiegen
            const uint8_t rmBit = rmAb ? 0 : 1;              // gewünschte Sicht: 1=Gerade,0=Abbiegen

            const uint8_t exp = st.expGeradeBit;
            st.okGerade = (exp != 0xFF) ? (rmBit == exp) : false;
            st.checkedGerade = true;

            DBG_PRINTF("[SBHF] ST eval W%u PH1: rmBit=%u (rmAb=%u) exp=%u ok=%u\n",
                       (unsigned)(12 + i), (unsigned)rmBit, (unsigned)rmAb, (unsigned)exp, (unsigned)st.okGerade);
            if (!st.okGerade)
            {
                m_stKnown[i] = true;
                m_stPos[i]   = rmBit ? 0 : 1; // 0=GERADE,1=ABBIEGEN (bei fault ist es aktuell Abbiegen)
                DBG_PRINTF("[SBHF] ST fault W%u PH1 -> assume stuck pos=%u\n",
                           (unsigned)(12 + i), (unsigned)m_stPos[i]);
                // NEW: If phase 1 already fails (no toggle / wrong position),
                // phase 2 is pointless for this turnout. Mark phase 2 as failed and checked.
                st.issuedAbbiegen  = true;
                st.checkedAbbiegen = true;
                st.okAbbiegen      = false;
                st.dueAbbiegenMs   = nowMs;
                st.expAbbiegenBit  = 0xFF;
                DBG_PRINTF("[SBHF] ST skip W%u PH2 (PH1 already failed)\n", (unsigned)(12 + i));

            }
        }

        // Abbiegen prüfen
        if (st.issuedAbbiegen && !st.checkedAbbiegen && (nowMs >= st.dueAbbiegenMs))
        {
            const bool rmAb = w->rueckmeldungAbbiegen();      // true=Abbiegen
            const uint8_t rmBit = rmAb ? 0 : 1;              // 1=Gerade,0=Abbiegen

            const uint8_t exp = st.expAbbiegenBit;
            st.okAbbiegen = (exp != 0xFF) ? (rmBit == exp) : false;
            st.checkedAbbiegen = true;

            DBG_PRINTF("[SBHF] ST eval W%u PH2: rmBit=%u (rmAb=%u) exp=%u ok=%u\n",
                       (unsigned)(12 + i), (unsigned)rmBit, (unsigned)rmAb, (unsigned)exp, (unsigned)st.okAbbiegen);

            if (!st.okAbbiegen)
            {
                m_stKnown[i] = true;
                m_stPos[i]   = rmBit ? 0 : 1; // wenn fault, ist die Weiche vermutlich auf Gerade (rmBit==1)
                DBG_PRINTF("[SBHF] ST fault W%u PH2 -> assume stuck pos=%u\n",
                           (unsigned)(12 + i), (unsigned)m_stPos[i]);
            }
        }
    }

    // ------------------------------------------------------------
    // 2) Puls-Sequencer: genau 1 Puls zur Zeit, PULSE_MS breit.
    //    Puls-Ende startet settle für diese Weiche, und im selben Tick
    //    kann der nächste Puls beginnen (Pipeline wie gewünscht).
    // ------------------------------------------------------------

    auto startPulseForIdx = [&](uint8_t i)
    {
        DBG_SBHF_ST("[SST] startPulse idx=%u W%u phase=%u nextIdx=%u pulseActive=%u pulseIdx=%u now=%lu\n",
                    (unsigned)i,
                    (unsigned)(12 + i),
                    (unsigned)m_selftestPhase,
                    (unsigned)m_selftestNextIdx,
                    (unsigned)m_stPulseActive,
                    (unsigned)m_stPulseIdx,
                    (unsigned long)nowMs);

        auto &st = m_stT[i];

        bool targetAb = false;
        uint8_t expBit = 0xFF;
        bool rmAbNow = weichen[i]->rueckmeldungAbbiegen(); // nur für Logging / PH1-Entscheidung

        if (m_selftestPhase == ST_GERADE)
        {
            // Phase 1:
            // zuerst immer GEGEN die aktuell gemeldete Stellung schalten
            // RM-Abbiegen: true=Abzweig, false=Gerade
            targetAb = !rmAbNow;
            // rmBit-Sicht im Selftest: 1=Gerade, 0=Abbiegen
            expBit = targetAb ? 0 : 1;
            st.expGeradeBit = expBit;
        }
        else
        {
            // Phase 2:
            // NICHT erneut aus aktuellem RM ableiten,
            // sondern die GEGENTEILIGE Richtung von Phase 1 testen.
            //
            // expGeradeBit: 1=Gerade, 0=Abbiegen
            // Gegentest = invertiertes expGeradeBit
            if (st.expGeradeBit == 0xFF) {
                DBG_PRINTF("[SBHF] ST WARN W%u PH2 without PH1 expectation\n", (unsigned)(12 + i));
                return;
            }

            expBit = (st.expGeradeBit == 1) ? 0 : 1; // opposite of phase 1
            targetAb = (expBit == 0);                // 0=Abbiegen, 1=Gerade
            st.expAbbiegenBit = expBit;
        }

        DBG_PRINTF("[SBHF] ST pulse ON  W%u -> %s (rmAbNow=%u expBit=%u)\n",
                   (unsigned)(12 + i),
                   targetAb ? "ABBIEGEN" : "GERADE",
                   (unsigned)rmAbNow,
                   (unsigned)expBit);

                   DBG_SBHF_ST("[SST] before schalte W%u target=%s rmAbNow=%u expBit=%u now=%lu\n",
                    (unsigned)(12 + i),
                    targetAb ? "ABBIEGEN" : "GERADE",
                    (unsigned)rmAbNow,
                    (unsigned)expBit,
                    (unsigned long)nowMs);


        // Selftest muss sicher pulsen dürfen, auch wenn die Weiche gerade noch in COOLDOWN hängt.
        const bool started = weichen[i]->schalte(targetAb ? Weiche::ABBIEGEN : Weiche::GERADE, true);

        DBG_SBHF_ST("[SST] after schalte W%u started=%u now=%lu\n",
                    (unsigned)(12 + i),
                    (unsigned)started,
                    (unsigned long)nowMs);

        if (!started) {
            DBG_PRINTF("[SBHF] ST WARN W%u pulse rejected (busy)\n", (unsigned)(12 + i));
            DBG_SBHF_ST("[SST] reject W%u pulse not started now=%lu\n",
                        (unsigned)(12 + i),
                        (unsigned long)nowMs);
            return;
        }

        DBG_SBHF_ST("[SST] mark-active W%u now=%lu\n",
                    (unsigned)(12 + i),
                    (unsigned long)nowMs);
        m_stPulseActive  = true;
        m_stPulseStartMs = nowMs;
        m_stPulseIdx     = i;
    };

    auto finishPulseForIdx = [&](uint8_t i)
    {
        auto &st = m_stT[i];
        DBG_SBHF_ST("[SST] finishPulse idx=%u W%u phase=%u pulseActive=%u start=%lu now=%lu dt=%lu\n",
                    (unsigned)i,
                    (unsigned)(12 + i),
                    (unsigned)m_selftestPhase,
                    (unsigned)m_stPulseActive,
                    (unsigned long)m_stPulseStartMs,
                    (unsigned long)nowMs,
                    (unsigned long)(nowMs - m_stPulseStartMs));


        if (m_selftestPhase == ST_GERADE)
        {
            st.issuedGerade = true;
            st.dueGeradeMs  = nowMs + SELFTEST_SETTLE_MS; // settle ab Puls-ENDE
            DBG_PRINTF("[SBHF] ST pulse OFF W%u %s -> due +%lums\n",
                       (unsigned)(12 + i),
                       (st.expGeradeBit == 0) ? "ABBIEGEN" : "GERADE",
                       (unsigned long)SELFTEST_SETTLE_MS);
        }
        else
        {
            st.issuedAbbiegen = true;
            st.dueAbbiegenMs  = nowMs + SELFTEST_SETTLE_MS; // settle ab Puls-ENDE
            DBG_PRINTF("[SBHF] ST pulse OFF W%u %s -> due +%lums\n",
                       (unsigned)(12 + i),
                       (st.expAbbiegenBit == 0) ? "ABBIEGEN" : "GERADE",
                       (unsigned long)SELFTEST_SETTLE_MS);
        }

        m_stPulseActive = false;
    };

    // Wenn gerade ein Puls läuft: ggf. beenden
    if (m_stPulseActive)
    {
        DBG_SBHF_ST("[SST] pulse-running W%u now=%lu start=%lu dt=%lu need=%lu\n",
                    (unsigned)(12 + m_stPulseIdx),
                    (unsigned long)nowMs,
                    (unsigned long)m_stPulseStartMs,
                    (unsigned long)(nowMs - m_stPulseStartMs),
                    (unsigned long)SELFTEST_PULSE_MS);

        if (nowMs - m_stPulseStartMs >= SELFTEST_PULSE_MS)
        {
            finishPulseForIdx(m_stPulseIdx);
            // danach darf im selben Update gleich der nächste Puls starten
        }
        else
        {
            DBG_SBHF_ST("[SST] pulse-hold W%u dt=%lu\n",
                        (unsigned)(12 + m_stPulseIdx),
                        (unsigned long)(nowMs - m_stPulseStartMs));

            // Puls läuft noch -> in diesem Tick keinen neuen Puls starten
            return;
        }
    }

    // Wenn kein Puls läuft: ggf. nächsten starten
    if (!m_stPulseActive)
    {
        // Phase GERADE
        if (m_selftestPhase == ST_GERADE)
        {
            if (m_selftestNextIdx < maxWeichen)
            {
                // NEW: do not start PH2 pulse if PH2 already marked checked (e.g. PH1 failed)
                if (m_stT[m_selftestNextIdx].checkedAbbiegen)
                {
                    m_selftestNextIdx++;
                    return;
                }
                startPulseForIdx(m_selftestNextIdx);
                m_selftestNextIdx++;
                return;
            }

            // Nur umschalten, wenn alle Gerade-Impulse auch wirklich "issued" sind
            bool allIssuedGerade = true;
            for (uint8_t i = 0; i < maxWeichen; i++)
                if (!m_stT[i].issuedGerade) { allIssuedGerade = false; break; }

            if (allIssuedGerade)
            {
                m_selftestPhase   = ST_ABBIEGEN;
                m_selftestNextIdx = 0;
                DBG_PRINTLN(F("[SBHF] ST phase switch -> ABBIEGEN"));
            }

            // Kein return hier: Abbiegen kann im nächsten Tick starten
        }

        // Phase ABBIEGEN
        if (m_selftestPhase == ST_ABBIEGEN)
        {
            if (m_selftestNextIdx < maxWeichen)
            {
                startPulseForIdx(m_selftestNextIdx);
                m_selftestNextIdx++;
                return;
            }
        }
    }

    // ------------------------------------------------------------
    // 3) Abschluss: erst wenn alle relevanten Weichen beide Checks haben
    // ------------------------------------------------------------
    bool allDone = true;
    for (uint8_t i = 0; i < maxWeichen; i++)
    {
        const auto &st = m_stT[i];
        if (!(st.checkedGerade && st.checkedAbbiegen))
        {
            allDone = false;
            break;
        }
    }
    if (!allDone)
        return;

    for (uint8_t i = 0; i < maxWeichen; i++)
    {
        const auto &st = m_stT[i];
        m_stOk[i] = (st.okGerade && st.okAbbiegen);
        // Detailed per-turnout summary: PH1/PH2 + overall
        DBG_PRINTF("[SBHF] ST sum W%u: PH1=%s PH2=%s overall=%s known=%u pos=%u\n",
                   (unsigned)(12 + i),
                   st.okGerade   ? "OK" : "FAIL",
                   st.okAbbiegen ? "OK" : "FAIL",
                   m_stOk[i]     ? "OK" : "FAIL",
                   (unsigned)m_stKnown[i],
                   (unsigned)m_stPos[i]);
    }

    selftestFinish();
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
    
    // Hinweis: Wenn wir Power für den Selftest bewusst ausgeschaltet haben,
    // bleibt sie absichtlich AUS. User muss danach manuell wieder einschalten.
    if (m_selftestForcedPowerOff && !g_power.isMainPowerOn())
        DBG_PRINTLN(F("[SBHF] Selftest finished: MAIN POWER remains OFF (manual power-on required)"));
    m_selftestForcedPowerOff = false; // one-shot
}
