#include "proto_mega2.h"
#include "BlockController.h"
#include "ShadowYardController.h"

extern BlockController      g_bc;
extern ShadowYardController g_sbhf;

void mega2_buildPayload(Mega2Payload& p)
{
    p.timestamp = millis();

    // ----------------------------------
    // BLOCKS (nur über Controller-Status)
    // ----------------------------------
    for (uint8_t i = 0; i < MEGA2_NUM_BLOCKS; i++)
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
    // WEICHEN (NEU: aus ShadowYardController, kein g_weichen mehr)
    // ----------------------------------
    const uint8_t n = g_sbhf.weichenCount();
    for (uint8_t w = 0; w < MEGA2_MAX_WEICHEN; w++)
    {
        if (w < n)
        {
            p.weichenIst[w]  = g_sbhf.weicheIst(w);
            p.weichenSoll[w] = g_sbhf.weicheSoll(w);
        }
        else
        {
            p.weichenIst[w]  = false;
            p.weichenSoll[w] = false;
        }
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
