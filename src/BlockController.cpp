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

static bool isTopRoute(uint8_t fromBlock, uint8_t toBlock)
{
    return (fromBlock == 4 && toBlock == 1) ||
           (fromBlock == 1 && toBlock == 2) ||
           (fromBlock == 2 && toBlock == 3);
}

static bool isBottomRoute(uint8_t fromBlock, uint8_t toBlock)
{
    return (fromBlock == 3 && toBlock == 4) ||
           (fromBlock == 4 && toBlock == 5) ||
           (fromBlock == 5 && toBlock >= 7 && toBlock <= 9) ||
           (fromBlock >= 7 && fromBlock <= 9 && toBlock == 6) ||
           (fromBlock == 6 && toBlock == 4);
}

bool BlockController::routeUsesTopTrafo(uint8_t fromBlock, uint8_t toBlock)
{
    return isTopRoute(fromBlock, toBlock);
}

bool BlockController::routeUsesBottomTrafo(uint8_t fromBlock, uint8_t toBlock)
{
    return isBottomRoute(fromBlock, toBlock);
}

void BlockController::startGrantFreeze(uint32_t nowMs)
{
    startGrantFreezeTop(nowMs);
    startGrantFreezeBottom(nowMs);
}

void BlockController::startGrantFreezeTop(uint32_t nowMs)
{
    const bool wasActive = isGrantFreezeTopActive(nowMs);
    const uint32_t newUntil = nowMs + BlockController::GRANT_FREEZE_MS;

    if (!wasActive)
    {
        Serial.print(F("[BLK] freeze TOP start ("));
        Serial.print((unsigned long)BlockController::GRANT_FREEZE_MS);
        Serial.println(F(" ms)"));
    }

    if (newUntil > m_grantFreezeTopUntilMs)
        m_grantFreezeTopUntilMs = newUntil;
}

void BlockController::startGrantFreezeBottom(uint32_t nowMs)
{
    const bool wasActive = isGrantFreezeBottomActive(nowMs);
    const uint32_t newUntil = nowMs + BlockController::GRANT_FREEZE_MS;

    if (!wasActive)
    {
        Serial.print(F("[BLK] freeze BOTTOM start ("));
        Serial.print((unsigned long)BlockController::GRANT_FREEZE_MS);
        Serial.println(F(" ms)"));
    }

    if (newUntil > m_grantFreezeBottomUntilMs)
        m_grantFreezeBottomUntilMs = newUntil;
}

bool BlockController::isGrantFreezeActive(uint32_t nowMs) const
{
    return isGrantFreezeTopActive(nowMs) || isGrantFreezeBottomActive(nowMs);
}

bool BlockController::isGrantFreezeTopActive(uint32_t nowMs) const
{
    return nowMs < m_grantFreezeTopUntilMs;
}

bool BlockController::isGrantFreezeBottomActive(uint32_t nowMs) const
{
    return nowMs < m_grantFreezeBottomUntilMs;
}

bool BlockController::isPowerRecoveryBlockActive(uint32_t nowMs) const
{
    return isPowerRecoveryBlockTopActive(nowMs) || isPowerRecoveryBlockBottomActive(nowMs);
}

bool BlockController::isPowerRecoveryBlockTopActive(uint32_t nowMs) const
{
    return nowMs < m_powerRecoveryTopUntilMs;
}

bool BlockController::isPowerRecoveryBlockBottomActive(uint32_t nowMs) const
{
    return nowMs < m_powerRecoveryBottomUntilMs;
}

bool BlockController::isEntrySuppressedByPower(uint32_t nowMs) const
{
    return isLowerPathSuppressed(nowMs) ||
           m_powerTopUnavailable || isPowerRecoveryBlockTopActive(nowMs);
}

bool BlockController::isEntrySuppressedByPower(uint8_t fromBlock, uint8_t toBlock, uint32_t nowMs) const
{
    const bool topRoute = routeUsesTopTrafo(fromBlock, toBlock);
    const bool bottomRoute = routeUsesBottomTrafo(fromBlock, toBlock);

    if (topRoute && (m_powerTopUnavailable || isPowerRecoveryBlockTopActive(nowMs)))
        return true;

    if (bottomRoute && isLowerPathSuppressed(nowMs))
        return true;

    return false;
}

bool BlockController::isLowerPathSuppressed(uint32_t nowMs) const
{
    return m_powerBottomUnavailable || isPowerRecoveryBlockBottomActive(nowMs);
}

void BlockController::setPowerUnavailable(bool unavailable)
{
    setPowerUnavailableTop(unavailable);
    setPowerUnavailableBottom(unavailable);
}

void BlockController::setPowerUnavailableTop(bool unavailable)
{
    if (m_powerTopUnavailable == unavailable)
        return;

    m_powerTopUnavailable = unavailable;

    Serial.print(F("[BLK] power TOP "));
    Serial.println(unavailable ? F("unavailable -> block top-path entries") : F("available"));
}

void BlockController::setPowerUnavailableBottom(bool unavailable)
{
    if (m_powerBottomUnavailable == unavailable)
        return;

    m_powerBottomUnavailable = unavailable;

    Serial.print(F("[BLK] power BOTTOM "));
    Serial.println(unavailable ? F("unavailable -> block bottom-path entries") : F("available"));
}

void BlockController::startPowerRecoveryBlock(uint32_t nowMs)
{
    startPowerRecoveryBlockTop(nowMs);
    startPowerRecoveryBlockBottom(nowMs);
}

void BlockController::startPowerRecoveryBlockTop(uint32_t nowMs)
{
    const bool wasActive = isPowerRecoveryBlockTopActive(nowMs);
    const uint32_t newUntil = nowMs + BlockController::POWER_RECOVERY_BLOCK_MS;

    if (!wasActive)
    {
        Serial.print(F("[BLK] power recovery TOP start ("));
        Serial.print((unsigned long)BlockController::POWER_RECOVERY_BLOCK_MS);
        Serial.println(F(" ms)"));
    }

    if (newUntil > m_powerRecoveryTopUntilMs)
        m_powerRecoveryTopUntilMs = newUntil;
}

void BlockController::startPowerRecoveryBlockBottom(uint32_t nowMs)
{
    const bool wasActive = isPowerRecoveryBlockBottomActive(nowMs);
    const uint32_t newUntil = nowMs + BlockController::POWER_RECOVERY_BLOCK_MS;

    if (!wasActive)
    {
        Serial.print(F("[BLK] power recovery BOTTOM start ("));
        Serial.print((unsigned long)BlockController::POWER_RECOVERY_BLOCK_MS);
        Serial.println(F(" ms)"));
    }

    if (newUntil > m_powerRecoveryBottomUntilMs)
        m_powerRecoveryBottomUntilMs = newUntil;
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
    
    const bool freezeTopActive = isGrantFreezeTopActive(nowMs);
    const bool freezeBottomActive = isGrantFreezeBottomActive(nowMs);

    if (!freezeTopActive && m_grantFreezeTopUntilMs != 0)
    {
        Serial.println(F("[BLK] freeze TOP end"));
        m_grantFreezeTopUntilMs = 0;
    }

    if (!freezeBottomActive && m_grantFreezeBottomUntilMs != 0)
    {
        Serial.println(F("[BLK] freeze BOTTOM end"));
        m_grantFreezeBottomUntilMs = 0;
    }

    if (!isPowerRecoveryBlockTopActive(nowMs) && m_powerRecoveryTopUntilMs != 0)
    {
        Serial.println(F("[BLK] power recovery TOP end"));
        m_powerRecoveryTopUntilMs = 0;
    }

    if (!isPowerRecoveryBlockBottomActive(nowMs) && m_powerRecoveryBottomUntilMs != 0)
    {
        Serial.println(F("[BLK] power recovery BOTTOM end"));
        m_powerRecoveryBottomUntilMs = 0;
    }

    if (!freezeTopActive)
    {
        const uint8_t topIds[] = {1, 2, 3};
        for (uint8_t i = 0; i < (sizeof(topIds) / sizeof(topIds[0])); ++i)
        {
            const uint8_t id = topIds[i];
            if (id > m_count) continue;
            Block* b = m_blocks ? m_blocks[id] : nullptr;
            const bool occ = isOccupied(id);
            m_targetFreeCache[id] = b ? b->isReallyFree(nowMs) : !occ;
        }
    }

    if (!freezeBottomActive)
    {
        const uint8_t bottomIds[] = {4, 5, 6, 7, 8, 9};
        for (uint8_t i = 0; i < (sizeof(bottomIds) / sizeof(bottomIds[0])); ++i)
        {
            const uint8_t id = bottomIds[i];
            if (id > m_count) continue;
            Block* b = m_blocks ? m_blocks[id] : nullptr;
            const bool occ = isOccupied(id);
            m_targetFreeCache[id] = b ? b->isReallyFree(nowMs) : !occ;
        }

        updateGrantBlock4(nowMs);
    }
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

    const uint32_t now = millis();
    if (isEntrySuppressedByPower(fromBlock, toBlock, now))
    {
#if MEGA2_DEBUG_BLOCK_GRANT
        logBgrantRateLimited(now,
                             fromBlock,
                             toBlock,
                             false,
                             false,
                             isOccupied(toBlock),
                             m_grantTo4_from);
#endif
        return false;
    }

        // Merge/Arbitration: für Block 4 nur wenn Grant für den 'fromBlock' aktiv ist
    if (toBlock == 4)
    {
        const bool granted = entryGranted(fromBlock, toBlock);
        if (!granted)
        {
#if MEGA2_DEBUG_BLOCK_GRANT
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
    const bool freezeActive = routeUsesTopTrafo(fromBlock, toBlock)
                            ? isGrantFreezeTopActive(now)
                            : (routeUsesBottomTrafo(fromBlock, toBlock)
                                ? isGrantFreezeBottomActive(now)
                                : false);

    const bool free = freezeActive
                    ? m_targetFreeCache[toBlock]
                    : (b ? b->isReallyFree(now) : !occ);

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
