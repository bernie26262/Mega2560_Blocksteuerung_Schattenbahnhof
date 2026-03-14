#include "Mega2Status.h"

#include <Arduino.h>

#include "BlockController.h"
#include "ShadowYardController.h"
#include "Weiche.h"
#include "safety.h"
#include "safety_error.h"
#include "Mega2RunMode.h"

// globale Controller
extern BlockController      g_bc;
extern ShadowYardController g_sbhf;

// Weichen aus main.cpp
extern Weiche w12;
extern Weiche w13;
extern Weiche w14;
extern Weiche w15;

// g_bootId: weak definition, damit der Linker auch dann zufrieden ist,
// wenn (noch) keine andere starke Definition existiert.
// Falls du irgendwann woanders eine starke Definition anlegst, gewinnt diese automatisch.
__attribute__((weak)) uint16_t g_bootId = 0;

// --------------------------------------------------
// SAFETY
// --------------------------------------------------
void buildMega2SafetyStatus(Mega2SafetyStatus& out)
{
    out.notausActive = safetyIsEmergencyActive() ? 1 : 0;

    // SSR Ist-Zustand abbilden
    out.ssrMask =
        (safetyIsSSR(SSR_MAIN_ENABLE) ? 0x01 : 0) |
        (safetyIsSSR(SSR_TRAFO_A)     ? 0x02 : 0) |
        (safetyIsSSR(SSR_TRAFO_B)     ? 0x04 : 0);

    // globale Safety-Fehlerflags
    out.errorFlags = safetyIsEmergencyActive() ? 1 : 0;

    // NEU: Block-Grund (BOOT / EMERGENCY / NONE)
    out.blockReason = safetyGetBlockReason();
}

// --------------------------------------------------
// BLOCKS
// --------------------------------------------------
void buildMega2BlockStatus(BlockStatus* out,
                           const BlockController& bc)
{
    // BlockController arbeitet 1-basiert (Block 1..N).
    // Das übertragene Array ist aber 0-basiert (Index 0 == Block 1).
    memset(out, 0, sizeof(BlockStatus) * bc.count());
    for (uint8_t i = 0; i < bc.count(); i++)
    {
        const uint8_t blockId = (uint8_t)(i + 1u);
        out[i].besetzt  = bc.isOccupied(blockId);
        out[i].stromRaw = bc.stromFiltered(blockId);
    }
}

// --------------------------------------------------
// SHADOW YARD
// --------------------------------------------------
void buildMega2ShadowStatus(ShadowYardStatus& out,
                            const ShadowYardController& sy)
{
    memset(&out, 0, sizeof(out));

    out.state        = static_cast<uint8_t>(sy.state());
    out.ausfahrGleis = sy.ausfahrGleis();

    // Falls noch nicht belegt: 0xFF ist semantisch "keins"
    out.einfahrGleis = 0xFF;

    // Modus: aktuell nicht nach außen getterbar -> 0 (Sequential) als Default
    out.modus = 0;

    // Belegung SBHF-Gleise kann später sauber gesetzt werden, wenn gewünscht.
    // Für Startup-Flags reicht das hier:
    out.selftestFlags = 0;
    if (sy.isSelftestActive()) out.selftestFlags |= 0x01;
    if (sy.isSelftestDone())   out.selftestFlags |= 0x02;
}

// --------------------------------------------------
// Helper: Turnout masks (Bit0=W12..Bit3=W15)
// --------------------------------------------------
static uint16_t buildTurnoutSollMask()
{
    uint16_t m = 0;
    if (w12.getStellung() == Weiche::ABBIEGEN) m |= (1u << 0);
    if (w13.getStellung() == Weiche::ABBIEGEN) m |= (1u << 1);
    if (w14.getStellung() == Weiche::ABBIEGEN) m |= (1u << 2);
    if (w15.getStellung() == Weiche::ABBIEGEN) m |= (1u << 3);
    return m;
}

static uint16_t buildTurnoutIstMask()
{
    uint16_t m = 0;
    if (w12.rueckmeldungAbbiegen()) m |= (1u << 0);
    if (w13.rueckmeldungAbbiegen()) m |= (1u << 1);
    if (w14.rueckmeldungAbbiegen()) m |= (1u << 2);
    if (w15.rueckmeldungAbbiegen()) m |= (1u << 3);
    return m;
}

// --------------------------------------------------
// SYSTEM STATUS (v4, kompakt)
// --------------------------------------------------
void buildMega2SystemStatus(SystemStatus& out)
{
    out.version = SYSTEM_STATUS_VERSION;
    out.nodeId  = NODE_MEGA2;
    out.size    = sizeof(SystemStatus);

    out.uptimeMs = millis();
    out.bootId   = g_bootId;

    // -----------------------------
    // FLAGS
    // -----------------------------
    out.flags = SYS_OK;

    if (safetyIsEmergencyActive())
        out.flags |= SYS_NOTAUS_ACTIVE;

    if (safetyIsLocked())
        out.flags |= SYS_ERROR_PRESENT;

    if (safetyIsPowerOn())
        out.flags |= SYS_POWER_ON;

    if (mega2IsDiagTest())
        out.flags |= SYS_MODE_DIAG;

    // -----------------------------
    // SAFETY ERROR DETAILS
    // -----------------------------
    const SafetyErrorInfo& err = safetyErrorGet();
    out.safetyErrorType  = static_cast<uint8_t>(err.type);
    out.safetyErrorIndex = err.index;

    // -----------------------------
    // BLOCKS
    // -----------------------------
    out.blockOccupiedMask = 0;
    for (uint8_t i = 1; i <= 9; i++)
        if (g_bc.isOccupied(i))
            out.blockOccupiedMask |= (1 << (i - 1));

    // -----------------------------
    // SCHATTENBAHNHOF
    // -----------------------------
    out.sbhfState = static_cast<uint8_t>(g_sbhf.state());

    out.sbhfOccupiedMask = 0;
    for (uint8_t i = 0; i < 3; i++)
        if (g_bc.isOccupied(7 + i))
            out.sbhfOccupiedMask |= (1 << i);

    // META bits (without protocol bump):
    // 0x80 = SBHF selftest currently running.
    // 0x40 = SBHF selftest done.
    if (g_sbhf.isSelftestActive())
        out.sbhfOccupiedMask |= 0x80;
    if (g_sbhf.isSelftestDone())
        out.sbhfOccupiedMask |= 0x40;

    // aktuelles (ausgewähltes) Gleis 1..3 (0 = none)
    out.sbhfCurrentGleis = g_sbhf.ausfahrGleis();

    // Weichen Soll/Ist
    out.turnoutSollMask = buildTurnoutSollMask();
    out.turnoutIstMask  = buildTurnoutIstMask();

    // SBHF warnings/allowed mask -> reserved (Variant A)
    const uint8_t allowedMask = g_sbhf.allowedGleisMask();
    const uint8_t warningMask = g_sbhf.warningMask();

    if (warningMask != 0)
        out.flags |= SYS_WARNING_PRESENT;

    out.reserved = (static_cast<uint16_t>(allowedMask) << 8) | warningMask;

    // v3: no dedicated sbhfFlags field.
}
