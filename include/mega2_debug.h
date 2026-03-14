#pragma once

//  - MEGA2_DEBUG=1: Debug-Ausgaben aktiv
//  - MEGA2_DEBUG=0: Alles kompiliert weg

// Performance test helper:
//  - MEGA2_PERF_QUIET=1: suppresses DBG_PRINT/DBG_PRINTLN/DBG_PRINTF
//    (but keeps explicit Serial prints elsewhere) to measure Serial impact.
#ifndef MEGA2_PERF_QUIET
#define MEGA2_PERF_QUIET 0
#endif

#include <Arduino.h>

#if MEGA2_DEBUG
  #include <stdarg.h>
  #include <stdio.h>

  #define DBG_BEGIN(b)    Serial.begin(b)

  #if MEGA2_PERF_QUIET
    #define DBG_PRINT(x)    do{}while(0)
    #define DBG_PRINTLN(x)  do{}while(0)
  #else
    #define DBG_PRINT(x)    Serial.print(x)
    #define DBG_PRINTLN(x)  Serial.println(x)
  #endif

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

  #if MEGA2_PERF_QUIET
    #define DBG_PRINTF(...) do{}while(0)
  #else
    #define DBG_PRINTF(...) dbgPrintf_(__VA_ARGS__)
  #endif

#else
  #define DBG_BEGIN(b)
  #define DBG_PRINT(x)
  #define DBG_PRINTLN(x)
  #define DBG_PRINTF(...)
#endif
