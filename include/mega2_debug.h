#pragma once

// Debug global ein/aus
//  - MEGA2_DEBUG=1: Debug-Ausgaben aktiv
//  - MEGA2_DEBUG=0: Alles kompiliert weg

#include <Arduino.h>

#if MEGA2_DEBUG
  #include <stdarg.h>
  #include <stdio.h>

  #define DBG_BEGIN(b)    Serial.begin(b)
  #define DBG_PRINT(x)    Serial.print(x)
  #define DBG_PRINTLN(x)  Serial.println(x)

  // AVR hat i.d.R. kein Serial.printf(), daher eigener Formatter
  static inline void dbgPrintf_(const char* fmt, ...)
  {
      char buf[128];
      va_list ap;
      va_start(ap, fmt);
      vsnprintf(buf, sizeof(buf), fmt, ap);
      va_end(ap);
      Serial.print(buf);
  }

  #define DBG_PRINTF(...) dbgPrintf_(__VA_ARGS__)
#else
  #define DBG_BEGIN(b)
  #define DBG_PRINT(x)
  #define DBG_PRINTLN(x)
  #define DBG_PRINTF(...)
#endif
