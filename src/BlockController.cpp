#include "BlockController.h"
#include "Block.h"
#include "mega2_debug.h"
#include "safety_error.h"
#include "safety.h"

#if MEGA2_DEBUG
static constexpr uint8_t  DBG_MIN_ID = 1;
static constexpr uint8_t  DBG_MAX_ID = 9;
static constexpr uint32_t DBG_STABLE_FREE_MS = 500;

static inline bool dbgIdOk(uint8_t id)
{
    return (id >= DBG_MIN_ID && id <= DBG_MAX_ID);
}
#endif

// ------------------------------------------------------------
// ctor
// ------------------------------------------------------------
BlockController::BlockController(Block** blocks, uint8_t count)
: m_blocks(blocks),
  m_count(count)
{
    m_stromFiltered = new uint16_t[count]();
    m_stromActive   = new bool[count]();

#if MEGA2_DEBUG
    for (uint8_t i = 0; i < 16; i++)
    {
        m_dbgActive[i]     = false;
        m_dbgOcc[i]        = false;
        m_dbgStrom[i]      = false;
        m_dbgLastFreeMs[i] = 0;
    }
#endif
}

// ------------------------------------------------------------
// update
// ------------------------------------------------------------
void BlockController::update(uint32_t nowMs)
{
    for (uint8_t i = 0; i < m_count; i++)
        if (m_blocks[i])
            m_blocks[i]->update(nowMs);
}

// ------------------------------------------------------------
// isOccupied
// ------------------------------------------------------------
bool BlockController::isOccupied(uint8_t id) const
{
#if MEGA2_DEBUG
    if (dbgIdOk(id) && m_dbgActive[id])
        return (m_dbgOcc[id] || m_dbgStrom[id]);
#endif

    if (!m_blocks || id >= m_count || !m_blocks[id])
        return false;

    return m_blocks[id]->besetzt();
}

// ------------------------------------------------------------
// canEnter
// ------------------------------------------------------------
bool BlockController::canEnter(uint8_t from, uint8_t to) const
{
    if (!m_blocks || from >= m_count || to >= m_count)
        return false;

    Block* fromBlock = m_blocks[from];
    Block* toBlock   = m_blocks[to];
    if (!fromBlock || !toBlock)
        return false;

    uint32_t now = millis();

#if MEGA2_DEBUG
    if (dbgIdOk(to) && m_dbgActive[to])
    {
        if (m_dbgOcc[to] || m_dbgStrom[to])
            return false;

        if ((now - m_dbgLastFreeMs[to]) < DBG_STABLE_FREE_MS)
            return false;
    }
    else
#endif
    {
        if (!toBlock->isReallyFree(now))
            return false;
    }

    // ---------------- BLOCK 4 Sonderregeln ----------------
    if (to == 4)
    {
        uint8_t occ123 = 0;
        for (uint8_t i = 1; i <= 3; i++)
            if (m_blocks[i] && isOccupied(i))
                occ123++;

        if (from == 6)
            return (occ123 <= 2);

        if (from == 3)
        {
            if (!isOccupied(6))
                return true;
            return (occ123 > 2);
        }

        return false;
    }

    // ---------------- Standardpfade ----------------
    if (from == 1 && to == 2) return true;
    if (from == 2 && to == 3) return true;
    if (from == 4 && to == 5) return true;
    if (from == 5 && (to == 7 || to == 8 || to == 9)) return true;
    if ((from == 7 || from == 8 || from == 9) && to == 6) return true;

    return false;
}

// ------------------------------------------------------------
// stromFiltered
// ------------------------------------------------------------
uint16_t BlockController::stromFiltered(uint8_t id) const
{
#if MEGA2_DEBUG
    if (dbgIdOk(id) && m_dbgActive[id])
        return m_dbgStrom[id] ? 1 : 0;
#endif

    if (!m_blocks || id >= m_count || !m_blocks[id])
        return 0;

    return m_blocks[id]->stromAktiv() ? 1 : 0;
}

bool BlockController::stromOverThreshold(uint8_t) const
{
    return false;
}

#if MEGA2_DEBUG
// ------------------------------------------------------------
// DEBUG API
// ------------------------------------------------------------
void BlockController::debugSetOccupied(uint8_t id, bool occ)
{
    if (!dbgIdOk(id)) return;

    m_dbgActive[id] = true;
    m_dbgOcc[id]    = occ;

    if (!m_dbgOcc[id] && !m_dbgStrom[id])
        m_dbgLastFreeMs[id] = millis();
}

void BlockController::debugSetStrom(uint8_t id, bool active)
{
    if (!dbgIdOk(id)) return;

    m_dbgActive[id] = true;
    m_dbgStrom[id]  = active;

    if (!m_dbgOcc[id] && !m_dbgStrom[id])
        m_dbgLastFreeMs[id] = millis();
}

void BlockController::debugClear(uint8_t id)
{
    if (!dbgIdOk(id)) return;

    m_dbgActive[id]     = false;
    m_dbgOcc[id]        = false;
    m_dbgStrom[id]      = false;
    m_dbgLastFreeMs[id] = millis();
}
#endif

// Wird später vom Stromsensor / Kurzschluss-Detektor aufgerufen
// Aktuell noch NICHT aktiv verdrahtet
void BlockController::onShortCircuit(uint8_t block)
{
    // Kurzschluss am Block: Safety-Lock + SSRs AUS
    safetyTriggerBlockShort(block);
}
