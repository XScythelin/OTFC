#ifndef _OTFC_DEBUG_H_
#define _OTFC_DEBUG_H_

#include <Arduino.h>

#ifdef OTFC_DEBUG_PIN
#include "Hal/Gpio.h"
#define PIN_DEBUG(v) ::Otfc::Hal::Gpio::digitalWrite(OTFC_DEBUG_PIN, v)
#define PIN_DEBUG_INIT() ::Otfc::Hal::Gpio::pinMode(OTFC_DEBUG_PIN, OUTPUT)
#else
#define PIN_DEBUG(v)
#define PIN_DEBUG_INIT()
#endif

namespace Otfc {

#ifdef OTFC_DEBUG_SERIAL
extern Stream* _debugStream;

static inline void initDebugStream(Stream* p)
{
  _debugStream = p;
}

#define LOG_SERIAL_INIT(p) _debugStream = p;
#define LOG_SERIAL_DEBUG(v)                                                                                            \
  if (_debugStream)                                                                                                    \
  {                                                                                                                    \
    _debugStream->print(v);                                                                                            \
  }
#define LOG_SERIAL_DEBUG_HEX(v)                                                                                        \
  if (_debugStream)                                                                                                    \
  {                                                                                                                    \
    _debugStream->print(v, HEX);                                                                                       \
  }

template<typename T>
void D(T t)
{
  if (!_debugStream) return;
  _debugStream->println(t);
}

template<typename T, typename... Args>
void D(T t, Args... args) // recursive variadic function
{
  if (!_debugStream) return;
  _debugStream->print(t);
  _debugStream->print(' ');
  D(args...);
}

#else

static inline void initDebugStream(Stream* p) {}

#define LOG_SERIAL_INIT(p)
#define LOG_SERIAL_DEBUG(v)
#define LOG_SERIAL_DEBUG_HEX(v)
#define D(...)

#endif

} // namespace Otfc

#endif
