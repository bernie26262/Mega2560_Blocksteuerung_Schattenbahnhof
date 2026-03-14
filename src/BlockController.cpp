#include "BlockController.h"

#include "Block.h"
#include "mega2_debug.h"
#include "safety.h"  // safetyTriggerBlockShort()

// ------------------------------------------------------------
// SIM-Currentwerte (nur Debug/Sim)
// ------------------------------------------------------------
#if MEGA2_SIM_MODE
static constexpr uint16_t DBG_SIM_CURRENT_NOMINAL_MA = 300;
#endif
static constexpr uint16_t DBG_SIM_CURRENT_SHORT_MA   = 2500;

static inline bool idOk(uint8_t id, uint8_t count)
{
    return (id >= 1) && (id <= count);
}

BlockController::BlockController(Block** blocks, uint8_t count)
: m_blocks(blocks)
, m_count(count)
{
}

void BlockController::update(uint32_t nowMs)
{
    // Wichtig: IDs sind 1-basiert (g_blocks[0] = nullptr)
    for (uint8_t id = 1; id <= m_count; ++id)
    {
        Block* b = m_blocks ? m_blocks[id] : nullptr;
        if (!b) {
            m_stromFiltered[id] = 0;
            m_stromActive[id]   = false;
            continue;
        }

        b->update(nowMs);

        const bool active = b->stromAktiv();
        m_stromActive[id]   = active;

        if (!active) {
            m_stromFiltered[id] = 0;
        }
        else {
            // Wenn SensorStrom mA liefern kann -> nutzen. Sonst fallback.
            const uint16_t ma = b->stromRms_mA();
#if MEGA2_SIM_MODE
            // Simulation fallback: provide a nominal current value when
            // the simulated/current test path reports zero.
            if (ma != 0) {
                m_stromFiltered[id] = ma;
            } else {
                m_stromFiltered[id] = DBG_SIM_CURRENT_NOMINAL_MA; // fallback, solange mvPerAmp unbekannt
            }
#else
            // Hardware: Block ist stromaktiv, also mindestens 1 anzeigen
            m_stromFiltered[id] = (ma > 0) ? ma : 1;
#endif
        }

#if MEGA2_DEBUG
        // Debug-Helper: wenn Debug-OCC aktiv ist, merken wir uns den "frei" Zeitpunkt
        if (m_dbgActive[id])
        {
            const bool dbgOcc = (m_dbgOcc[id] || m_dbgStrom[id]);
            if (!dbgOcc && m_dbgLastFreeMs[id] == 0)
                m_dbgLastFreeMs[id] = nowMs;
            if (dbgOcc)
                m_dbgLastFreeMs[id] = 0;
        }
#endif
    }
    updateGrantBlock4(nowMs);
}

// ------------------------------------------------------------
// Entry-Request/Grant – Block 4 Merge (Einfahrt aus Block 3 oder 6)
// ------------------------------------------------------------
static constexpr uint32_t ENTRY_REQ_TIMEOUT_MS = 1500;

void BlockController::updateGrantBlock4(uint32_t nowMs)
{
    // Requests timeouten, wenn nicht periodisch erneuert
    if (m_reqB4_from3_ms && (nowMs - m_reqB4_from3_ms) > ENTRY_REQ_TIMEOUT_MS) m_reqB4_from3_ms = 0;
    if (m_reqB4_from6_ms && (nowMs - m_reqB4_from6_ms) > ENTRY_REQ_TIMEOUT_MS) m_reqB4_from6_ms = 0;

    // Wenn Block 4 belegt ist, kein Grant (und Requests sind weiterhin ok, weil man blocked erkennen kann)
    if (isOccupied(4))
    {
        m_grantTo4_from = 0;
        return;
    }

    const bool req3 = (m_reqB4_from3_ms != 0);
    const bool req6 = (m_reqB4_from6_ms != 0);

    if (!req3 && !req6)
    {
        m_grantTo4_from = 0;
        return;
    }

    // Zählregeln:
    // - occ123: Anzahl belegter Blöcke 1..3
    // - occUpper: Anzahl belegter Blöcke 5 + 6 + SBhf(7..9)
    uint8_t occ123 = 0;
    for (uint8_t b = 1; b <= 3 && b <= m_count; ++b)
        if (isOccupied(b)) occ123++;

    uint8_t occUpper = 0;
    for (uint8_t b = 5; b <= 9 && b <= m_count; ++b)
        if (isOccupied(b)) occUpper++;

    // Regeln (wie besprochen):
    // - From6->4 darf NUR wenn occ123 <= 2, und hat dann Vorfahrt vor From3->4.
    // - From3->4 darf nur wenn occUpper <= 4.
    // - Wenn occ123 > 2, wird From6->4 ohnehin nicht zugelassen => From3->4 hat dann (falls zulässig) Vorfahrt.
    const bool ok6 = req6 && (occ123 <= 2);
    const bool ok3 = req3 && (occUpper <= 4);

    uint8_t grant = 0;

    if (occ123 <= 2)
    {
        // Priority: From6->4
        if (ok6)      grant = 6;
        else if (ok3) grant = 3;
    }
    else
    {
        // Priority: From3->4
        if (ok3)      grant = 3;
        else if (ok6) grant = 6; // ok6 ist hier i.d.R. false, aber der Code bleibt robust
    }

    if (grant != m_grantTo4_from)
    {
        m_grantTo4_from = grant;
        m_grantTo4_ms   = nowMs;
    }
}

void BlockController::requestEnter(uint8_t fromBlock, uint8_t toBlock)
{
    const uint32_t now = millis();

    if (toBlock == 4)
    {
        if (fromBlock == 3) m_reqB4_from3_ms = now;
        if (fromBlock == 6) m_reqB4_from6_ms = now;
        updateGrantBlock4(now);
    }
}

void BlockController::cancelEnter(uint8_t fromBlock, uint8_t toBlock)
{
    if (toBlock == 4)
    {
        if (fromBlock == 3) m_reqB4_from3_ms = 0;
        if (fromBlock == 6) m_reqB4_from6_ms = 0;
        updateGrantBlock4(millis());
    }
}

bool BlockController::entryGranted(uint8_t fromBlock, uint8_t toBlock) const
{
    if (toBlock == 4)
        return (m_grantTo4_from != 0) && (m_grantTo4_from == fromBlock);

    // Für alle anderen Blöcke derzeit keine Arbitration – "Grant" entspricht dem normalen canEnter().
    return true;
}

bool BlockController::entryBlocked(uint8_t toBlock) const
{
    if (toBlock == 4)
    {
        // "blocked" == Block ist belegt UND es gibt einen gültigen Entry-Request
        const uint32_t now = millis();
        const bool req3 = m_reqB4_from3_ms && ((now - m_reqB4_from3_ms) <= ENTRY_REQ_TIMEOUT_MS);
        const bool req6 = m_reqB4_from6_ms && ((now - m_reqB4_from6_ms) <= ENTRY_REQ_TIMEOUT_MS);
        return isOccupied(4) && (req3 || req6);
    }
    return false;
}


uint16_t BlockController::stromFiltered(uint8_t id) const
{
    if (!idOk(id, m_count))
        return 0;

#if MEGA2_DEBUG
    if (m_dbgActive[id])
    {
        if (!m_dbgStrom[id])
            return 0;
        return m_dbgStromShort[id] ? DBG_SIM_CURRENT_SHORT_MA
                                   : DBG_SIM_CURRENT_NOMINAL_MA;
    }
#endif

    return m_stromFiltered[id];
}

bool BlockController::stromOverThreshold(uint8_t id) const
{
    if (id == 0 || id > m_count) return false;

#if MEGA2_SIM_MODE
    // SIM: "Kurzschluss" heißt: debug short flag gesetzt (k6)
    if (m_dbgStromShort[id]) return true;

    // optional: normaler Strom (i6) soll NICHT als Kurzschluss gelten
    return false;
#else
    // Real: wir haben nur stromAktiv() (kein mA), daher konservativ:
    Block* b = m_blocks[id];
    return b ? b->stromAktiv() : false;
#endif
}

bool BlockController::isOccupied(uint8_t id) const
{
    if (!idOk(id, m_count))
        return false;

#if MEGA2_DEBUG
    if (m_dbgActive[id])
        return m_dbgOcc[id] || m_dbgStrom[id];
#endif

    Block* b = m_blocks ? m_blocks[id] : nullptr;
    return b ? b->besetzt() : false;
}

bool BlockController::canEnter(uint8_t fromBlock, uint8_t toBlock) const
{
    (void)fromBlock;

    if (!idOk(toBlock, m_count))
        return false;

    // Merge/Arbitration: für Block 4 nur wenn Grant für den 'fromBlock' aktiv ist
    if (toBlock == 4)
    {
        if (!entryGranted(fromBlock, toBlock))
            return false;
    }

#if MEGA2_DEBUG
    // Debug: wenn wir den Block künstlich steuern, nehmen wir unseren "frei"-Timestamp
    if (m_dbgActive[toBlock])
    {
        if (isOccupied(toBlock))
            return false;

        uint32_t t = m_dbgLastFreeMs[toBlock];
        if (t == 0)
            return true; // gerade erst frei geworden oder nie gesetzt -> erlauben

        return (millis() - t) >= Block::STABLE_FREE_MS;
    }
#endif

    // Normalbetrieb: Freigabe erst, wenn Block wirklich frei ist (Debounce/Delay)
    Block* b = m_blocks[toBlock];
    if (!b) return !isOccupied(toBlock);
    return b->isReallyFree(millis());
}

#if MEGA2_DEBUG
static inline bool dbgIdOk(uint8_t id)
{
    // wir lassen hier 1..16 zu
    return (id >= 1) && (id <= 16);
}

void BlockController::debugSetOccupied(uint8_t id, bool occ)
{
    if (!dbgIdOk(id))
        return;

    m_dbgActive[id] = true;
    m_dbgOcc[id]    = occ;

    if (!occ && !m_dbgStrom[id])
        m_dbgLastFreeMs[id] = millis();
    if (occ)
        m_dbgLastFreeMs[id] = 0;
}

void BlockController::debugSetStrom(uint8_t id, bool active)
{
    if (!dbgIdOk(id))
        return;

    m_dbgActive[id]     = true;
    m_dbgStrom[id]      = active;
    m_dbgStromShort[id] = false;

    if (!active && !m_dbgOcc[id])
        m_dbgLastFreeMs[id] = millis();
    if (active)
        m_dbgLastFreeMs[id] = 0;
}

void BlockController::debugSetStromShort(uint8_t id, bool active)
{
    if (!dbgIdOk(id))
        return;

    m_dbgActive[id] = true;

    if (active)
    {
        m_dbgStrom[id]      = true;
        m_dbgStromShort[id] = true;
        m_dbgLastFreeMs[id] = 0;
    }
    else
    {
        // komplett aus
        m_dbgStrom[id]      = false;
        m_dbgStromShort[id] = false;

        if (!m_dbgOcc[id])
            m_dbgLastFreeMs[id] = millis();
    }
}

void BlockController::debugClear(uint8_t id)
{
    if (!dbgIdOk(id))
        return;

    m_dbgActive[id]      = false;
    m_dbgOcc[id]         = false;
    m_dbgStrom[id]       = false;
    m_dbgStromShort[id]  = false;
    m_dbgLastFreeMs[id]  = 0;
}
#endif

void BlockController::onShortCircuit(uint8_t block)
{
    DBG_PRINTF("[BC] ShortCircuit on block %u\n", block);
    safetyTriggerBlockShort(block);
}
