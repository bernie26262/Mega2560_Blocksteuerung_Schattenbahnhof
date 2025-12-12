#include "proto_mega2.h"
#include "BlockController.h"
#include "ShadowYardController.h"
#include "Weiche.h"

extern BlockController      g_bc;
extern ShadowYardController g_sbhf;
extern Weiche*              g_weichen[4];

void mega2_buildPayload(Mega2Payload& p)
{
    p.timestamp = millis();

    // ----------------------------------
    // BLOCKS (nur über Controller-Status)
    // ----------------------------------
    for (uint8_t i = 0; i < MEGA2_MAX_BLOCKS; i++)
    {
        p.blockOccupied[i]   = g_bc.isOccupied(i);
        p.blockStroeme_mA[i] = g_bc.stromFiltered(i);
    }

    // ----------------------------------
    // SCHATTENBAHNHOF-BELEGUNG
    // ----------------------------------
    p.sbhfOccupied[0] = g_bc.isOccupied(7);
    p.sbhfOccupied[1] = g_bc.isOccupied(8);
    p.sbhfOccupied[2] = g_bc.isOccupied(9);

    // ----------------------------------
    // WEICHEN
    // (Alt-Code bleibt vorerst)
    // ----------------------------------
    for (uint8_t w = 0; w < MEGA2_MAX_WEICHEN; w++)
    {
        p.weichenIst[w]  = g_weichen[w]->rueckmeldungAbbiegen();
        p.weichenSoll[w] =
            (g_weichen[w]->getStellung() == Weiche::ABBIEGEN);
    }

    // ----------------------------------
    // SCHATTENBAHNHOF-STATUS (Proto!)
    // ----------------------------------
    p.sbhfState = static_cast<uint8_t>(g_sbhf.state());

    // ⚠️ NICHT setzen (nicht im Proto):
    // p.nothaltAktiv
    // p.sbhfExit
    // p.sbhfTarget

    p.valid = true;
}
