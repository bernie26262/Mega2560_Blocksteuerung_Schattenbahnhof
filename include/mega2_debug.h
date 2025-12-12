#pragma once

// Debug global ein/aus
#define MEGA2_DEBUG 1

#if MEGA2_DEBUG
  #define DBG_BEGIN(b)    Serial.begin(b)
  #define DBG_PRINT(x)    Serial.print(x)
  #define DBG_PRINTLN(x)  Serial.println(x)
#else
  #define DBG_BEGIN(b)
  #define DBG_PRINT(x)
  #define DBG_PRINTLN(x)
#endif
