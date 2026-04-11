#pragma once

#define DEBUG
// #define SIMPLEFOC_DISABLE_DEBUG

#ifdef DEBUG
  #define DPRINTF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
  #define DPRINT(fmt) Serial.print(fmt)
  #define DPRINTLN(fmt) Serial.println(fmt)
#else
  #define DPRINTF(fmt, ...)  // compiles to nothing
  #define DPRINT(fmt)  // compiles to nothing
  #define DPRINTLN(fmt)  // compiles to nothing
#endif