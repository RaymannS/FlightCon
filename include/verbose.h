#pragma once
// Enable verbose serial output by adding -DVERBOSE_SERIAL to build_flags.
// In flight builds, omit the flag — VLOG/VLOGF compile away to nothing.
#ifdef VERBOSE_SERIAL
  #define VLOG(...)  Serial.println(__VA_ARGS__)
  #define VLOGF(...) Serial.printf(__VA_ARGS__)
#else
  #define VLOG(...)  ((void)0)
  #define VLOGF(...) ((void)0)
#endif
