// arduino_shim.h
// Minimal Arduino API shim for host testing. The core headers in src/
// are pure C++ and do not need this, but the shim lets any small
// Arduino-dependent helper compile under g++ on a PC. It is NOT a
// substitute for the real Arduino core on hardware.
#pragma once

#include <cstdint>

#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return HIGH; }
inline void delay(unsigned long) {}
inline unsigned long millis() { return 0; }
