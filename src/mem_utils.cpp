#include "mem_utils.h"

extern unsigned int __heap_start;
extern void* __brkval;

namespace MemUtils {

static int s_minFree = 0;

int freeMemory()
{
  int v;
  return (int)&v - (__brkval == nullptr ? (int)&__heap_start : (int)__brkval);
}

void resetMinFree()
{
  s_minFree = freeMemory();
}

void updateMinFree()
{
  const int f = freeMemory();
  if (s_minFree == 0 || f < s_minFree) s_minFree = f;
}

int minFreeMemory()
{
  if (s_minFree == 0) s_minFree = freeMemory();
  return s_minFree;
}

} // namespace MemUtils