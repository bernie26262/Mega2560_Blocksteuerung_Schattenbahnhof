#pragma once
#include <Arduino.h>

namespace MemUtils {
  int freeMemory();
  void resetMinFree();
  void updateMinFree();
  int  minFreeMemory();
}