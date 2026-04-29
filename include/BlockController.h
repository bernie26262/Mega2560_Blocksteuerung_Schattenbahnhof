#pragma once

#include <Arduino.h>

class Block;

class BlockController
{
public:
    BlockController(Block** blocks, uint8_t count);

    static constexpr uint32_t GRANT_FREEZE_MS = 2000;
    static constexpr uint32_t POWER_RECOVERY_BLOCK_MS = 4000;

    void update(uint32_t nowMs);

    uint8_t count() const { return m_count; }

    // Status
    bool     isOccupied(uint8_t id) const;
    uint16_t stromFiltered(uint8_t id) const;      // mA (SIM) / heuristisch (HW)
    bool     stromOverThreshold(uint8_t id) const; // optional

    void startGrantFreeze(uint32_t nowMs);
    bool isGrantFreezeActive(uint32_t nowMs) const;
    void startGrantFreezeTop(uint32_t nowMs);
    void startGrantFreezeBottom(uint32_t nowMs);
    bool isGrantFreezeTopActive(uint32_t nowMs) const;
    bool isGrantFreezeBottomActive(uint32_t nowMs) const;

    void setPowerUnavailable(bool unavailable);
    void startPowerRecoveryBlock(uint32_t nowMs);
    bool isPowerRecoveryBlockActive(uint32_t nowMs) const;
    bool isEntrySuppressedByPower(uint32_t nowMs) const;
    void setPowerUnavailableTop(bool unavailable);
    void setPowerUnavailableBottom(bool unavailable);
    void startPowerRecoveryBlockTop(uint32_t nowMs);
    void startPowerRecoveryBlockBottom(uint32_t nowMs);
    bool isPowerRecoveryBlockTopActive(uint32_t nowMs) const;
    bool isPowerRecoveryBlockBottomActive(uint32_t nowMs) const;
    bool isEntrySuppressedByPower(uint8_t fromBlock, uint8_t toBlock, uint32_t nowMs) const;
    bool isLowerPathSuppressed(uint32_t nowMs) const;

    // Einfahrt in Block erlaubt? (hilft dem SBHF)
    bool canEnter(uint8_t fromBlock, uint8_t toBlock) const;

    bool entryGranted(uint8_t fromBlock, uint8_t toBlock) const;
    bool entryBlocked(uint8_t toBlock) const;

    // --------------------------------------------------------
    // Grant-State für Block 4 (Einfahrt aus 3 oder 6)
    // --------------------------------------------------------
    void updateGrantBlock4(uint32_t nowMs);
    uint8_t  m_grantTo4_from  = 0; // 0=none, 3 or 6
    uint32_t m_grantTo4_ms    = 0;
#if MEGA2_DEBUG
    void debugSetOccupied(uint8_t id, bool occ);
    void debugSetStrom(uint8_t id, bool active);      // 300 mA
    void debugSetStromShort(uint8_t id, bool active); // 2500 mA
    void debugClear(uint8_t id);
#endif

    // optional: direkte Meldung "Kurzschluss erkannt"
    void onShortCircuit(uint8_t block);

private:
    Block**  m_blocks = nullptr;
    uint8_t  m_count  = 0;

    // Cache aus update()
    uint16_t m_stromFiltered[16] = {0};
    bool     m_stromActive[16]   = {false};

    // Freeze / Cache
    static bool routeUsesTopTrafo(uint8_t fromBlock, uint8_t toBlock);
    static bool routeUsesBottomTrafo(uint8_t fromBlock, uint8_t toBlock);

    uint32_t m_grantFreezeTopUntilMs = 0;
    uint32_t m_grantFreezeBottomUntilMs = 0;
    bool     m_targetFreeCache[16] = {false};

    bool     m_powerTopUnavailable = false;
    bool     m_powerBottomUnavailable = false;
    uint32_t m_powerRecoveryTopUntilMs = 0;
    uint32_t m_powerRecoveryBottomUntilMs = 0;

#if MEGA2_DEBUG
    bool     m_dbgActive[16]     = {false};
    bool     m_dbgOcc[16]        = {false};
    bool     m_dbgStrom[16]      = {false};
    bool     m_dbgStromShort[16] = {false};
    uint32_t m_dbgLastFreeMs[16] = {0};
#endif
};
