#include "Mega2RunMode.h"

// Default: AUTOMATIK.
static volatile Mega2RunMode s_mode = Mega2RunMode::Automatik;

Mega2RunMode mega2RunMode()
{
    return s_mode;
}

void mega2SetRunMode(Mega2RunMode m)
{
    s_mode = m;
}