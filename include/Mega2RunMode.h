#pragma once
#include <stdint.h>

// Mega2 Betriebsmodus:
// - AUTOMATIK: normale Block-/SBHF-Automatik läuft.
// - DIAG_TEST: Automatik-Schaltpfade pausiert, Safety bleibt aktiv.
enum class Mega2RunMode : uint8_t
{
    Automatik = 0,
    DiagTest  = 1,
};

Mega2RunMode mega2RunMode();
void mega2SetRunMode(Mega2RunMode m);

inline bool mega2IsDiagTest()
{
    return mega2RunMode() == Mega2RunMode::DiagTest;
}