#include "proto_mega2.h"
#include "BlockController.h"
#include "ShadowYardController.h"
#include "Weiche.h"

extern BlockController g_bc;
extern ShadowYardController g_sbhf;
extern Weiche* g_weichen[4];

void mega2_buildPayload(Mega2Payload& p)
{
    p.timestamp = millis();

    for (uint8_t i = 0; i < MEGA2_MAX_BLOCKS; i++)
    {
        Block* b = g_bc.block(i);
        if (b)
        {
            p.blockOccupied[i] = b->besetzt();
            p.blockStröme_mA[i] = b->strom_mA();
        }
        else
        {
            p.blockOccupied[i] = false;
            p.blockStröme_mA[i] = 0;
        }
    }

    p.sbhfOccupied[0] = g_bc.block(7)->besetzt();
    p.sbhfOccupied[1] = g_bc.block(8)->besetzt();
    p.sbhfOccupied[2] = g_bc.block(9)->besetzt();

    for (uint8_t w = 0; w < MEGA2_MAX_WEICHEN; w++)
    {
        p.weichenIst[w]  = g_weichen[w]->rueckmeldungAbbiegen();
        p.weichenSoll[w] = (g_weichen[w]->getStellung() == Weiche::ABBIEGEN);
    }

    p.nothaltAktiv = g_sbhf.nothaltAktiv();
    p.sbhfState    = (uint8_t)g_sbhf.state();
    p.sbhfExit     = g_sbhf.exitGleis();
    p.sbhfTarget   = g_sbhf.targetGleis();

    p.valid = true;
}
