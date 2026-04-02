#include "BlockController.h"

#include "Block.h"
#include "mega2_debug.h"
#include "safety.h"  // safetyTriggerBlockShort()

#ifndef MEGA2_DEBUG_BLOCK_GRANT
#define MEGA2_DEBUG_BLOCK_GRANT 0
#endif

#if MEGA2_DEBUG_BLOCK_GRANT
static uint32_t s_lastBgrantLogMs = 0;
static inline void logBgrantRateLimited(uint32_t nowMs,
                                        uint8_t fromBlock,
                                        uint8_t toBlock,
                                        bool granted,
                                        bool free,
                                        bool occupied,
                                        uint8_t grantTo4From)
{
    if ((uint32_t)(nowMs - s_lastBgrantLogMs) < 200u)
        return;

    s_lastBgrantLogMs = nowMs;

    Serial.print(F("[BGRANT] "));
    Serial.print(fromBlock);
    Serial.print(F("->"));
    Serial.print(toBlock);
    Serial.print(F(" granted="));
    Serial.print(granted ? 1 : 0);
    Serial.print(F(" free="));
    Serial.print(free ? 1 : 0);
    Serial.print(F(" occ="));
    Serial.print(occupied ? 1 : 0);
    Serial.print(F(" grant="));
    Serial.print(grantTo4From);
}
#endif


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
// Grant-Entscheidung – Block 4 Merge (Einfahrt aus Block 3 oder 6)
// ------------------------------------------------------------
void BlockController::updateGrantBlock4(uint32_t nowMs)
{
    (void)nowMs;

    const bool occ3 = isOccupied(3);
    const bool occ6 = isOccupied(6);
    Block* b4 = m_blocks[4];
    const bool free4 = b4 ? b4->isReallyFree(nowMs) : !isOccupied(4);

    // Zählregeln:
    // - occ123   = Anzahl belegter Blöcke 1..3
    // - occLower = Anzahl belegter Blöcke 5, SBHF1, SBHF2, SBHF3, 6
    uint8_t occ123 = 0;
    for (uint8_t b = 1; b <= 3 && b <= m_count; ++b)
        if (isOccupied(b)) occ123++;

    uint8_t occLower = 0;
    for (uint8_t b = 5; b <= 9 && b <= m_count; ++b)
        if (isOccupied(b)) occLower++;

    bool allow34 = false;
    bool allow64 = false;

    // Freigabe Block 3 -> 4:
    // - wenn kein Zug in Block 6 ist
    //   ODER
    // - wenn mehr als 2 Züge in 1..3 sind
    //   UND nicht mehr als 4 Züge in 5, SBHF1..3, 6 sind
    if (occ3 && free4)
    {
        if (!occ6)
        {
            allow34 = true;
        }
        else if ((occ123 > 2) && (occLower <= 4))
        {
            allow34 = true;
        }
    }

    // Freigabe Block 6 -> 4:
    // - wenn nicht mehr als 2 Züge in 1..3 sind
    if (occ6 && free4 && (occ123 <= 2))
    {
        allow64 = true;
    }

    // Vorfahrt:
    // Wenn Block 3 und 6 beide belegt sind, hat Block 6 Vorfahrt.
    uint8_t grant = 0;
    if (allow64)      grant = 6;
    else if (allow34) grant = 3;

    if (grant != m_grantTo4_from)
    {
        m_grantTo4_from = grant;
        m_grantTo4_ms   = nowMs;
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
        return isOccupied(4) && (isOccupied(3) || isOccupied(6));
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
#if MEGA2_SIM_MODE
        return m_dbgStromShort[id] ? DBG_SIM_CURRENT_SHORT_MA
                                   : DBG_SIM_CURRENT_NOMINAL_MA;
#else
        return 0;
#endif
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
    if (!idOk(toBlock, m_count))
        return false;

    // Merge/Arbitration: für Block 4 nur wenn Grant für den 'fromBlock' aktiv ist
    if (toBlock == 4)
    {
        const bool granted = entryGranted(fromBlock, toBlock);
        if (!granted)
        {
#if MEGA2_DEBUG_BLOCK_GRANT
            const uint32_t now = millis();
            logBgrantRateLimited(now,
                                 fromBlock,
                                 toBlock,
                                 false,
                                 false,
                                 isOccupied(4),
                                 m_grantTo4_from);
#endif
            return false;
        }
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
    const bool occ = isOccupied(toBlock);
    const uint32_t now = millis();
    const bool free = b ? b->isReallyFree(now) : !occ;

#if MEGA2_DEBUG_BLOCK_GRANT
    logBgrantRateLimited(now,
                         fromBlock,
                         toBlock,
                         true,
                         free,
                         occ,
                         m_grantTo4_from);
#endif

    return free;
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
