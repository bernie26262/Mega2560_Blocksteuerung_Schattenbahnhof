#include "proto_mega2.h"
#include "BlockController.h"
#include "ShadowYardController.h"

// --------------------------------------------------
// SAFETY
// --------------------------------------------------
void buildMega2SafetyStatus(Mega2SafetyStatus& out)
{
    // Proto: nur notausActive
    out.notausActive = false;
}

// --------------------------------------------------
// BLOCKS
// --------------------------------------------------
void buildMega2BlockStatus(BlockStatus* out, const BlockController& ctrl)
{
    for (uint8_t i = 0; i < ctrl.count(); i++)
    {
        out[i].besetzt    = ctrl.isOccupied(i);
        out[i].stromRaw   = ctrl.stromFiltered(i);
    }
}

// --------------------------------------------------
// SHADOW YARD (Status-only, strikt Proto-konform)
// --------------------------------------------------
void buildMega2ShadowStatus(ShadowYardStatus& out,
                            const ShadowYardController& sy)
{
    // Zustand
    out.state = static_cast<uint8_t>(sy.state());

    // B2: einzig relevantes Gleis
    out.ausfahrGleis = sy.activeGleis();
}
