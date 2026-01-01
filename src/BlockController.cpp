#include "BlockController.h"

#include "Block.h"
#include "mega2_debug.h"
#include "safety.h"  // safetyTriggerBlockShort()

// ------------------------------------------------------------
// SIM-Currentwerte (nur Debug/Sim)
// ------------------------------------------------------------
static constexpr uint16_t DBG_SIM_CURRENT_NOMINAL_MA = 300;
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

        // In HW haben wir hier derzeit nur eine bool-Aussage (stromAktiv).
        // Für Anzeige/Diagnose mapen wir das heuristisch auf ~300mA.
        const bool active = b->stromAktiv();
        m_stromActive[id]   = active;
        m_stromFiltered[id] = active ? DBG_SIM_CURRENT_NOMINAL_MA : 0;

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

    // 500ms stabil "frei" (wichtig gegen Prellen/Jitter)
    static constexpr uint32_t STABLE_FREE_MS = 500;

    if (!idOk(toBlock, m_count))
        return false;

#if MEGA2_DEBUG
    // Debug: wenn wir den Block künstlich steuern, nehmen wir unseren "frei"-Timestamp
    if (m_dbgActive[toBlock])
    {
        if (isOccupied(toBlock))
            return false;

        uint32_t t = m_dbgLastFreeMs[toBlock];
        if (t == 0)
            return true; // gerade erst frei geworden oder nie gesetzt -> erlauben

        return (millis() - t) >= STABLE_FREE_MS;
    }
#endif

    return !isOccupied(toBlock);
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
