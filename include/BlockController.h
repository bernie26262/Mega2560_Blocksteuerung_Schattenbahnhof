#pragma once

#include <Arduino.h>

class Block;

class BlockController
{
public:
    BlockController(Block** blocks, uint8_t count);

    void update(uint32_t nowMs);

    uint8_t count() const { return m_count; }

    // Status
    bool     isOccupied(uint8_t id) const;
    uint16_t stromFiltered(uint8_t id) const;      // mA (SIM) / heuristisch (HW)
    bool     stromOverThreshold(uint8_t id) const; // optional

    // Einfahrt in Block erlaubt? (hilft dem SBHF)
    bool canEnter(uint8_t fromBlock, uint8_t toBlock) const;

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

#if MEGA2_DEBUG
    bool     m_dbgActive[16]     = {false};
    bool     m_dbgOcc[16]        = {false};
    bool     m_dbgStrom[16]      = {false};
    bool     m_dbgStromShort[16] = {false};
    uint32_t m_dbgLastFreeMs[16] = {0};
#endif
};
