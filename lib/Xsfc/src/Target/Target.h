#pragma once

#if defined(ESP32S3)
  #include "TargetESP32s3.h"
#elif defined(ESP32)
  #include "TargetESP32.h"
#else
  #error "Unsupported platform!"
#endif

#include "Queue.h"
#include "Utils/MemoryHelper.h"

#if defined(XSFC_I2C_0)
  #if defined(XSFC_I2C_0_SOFT)
    #include "EspWire.h"
    #define WireClass EspTwoWire
    #define WireInstance EspWire
  #else
    #include <Wire.h>
    #define WireClass TwoWire
    #define WireInstance Wire
  #endif
  #if defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_TWOWIRE)
    WireClass WireInstance;
  #endif
#endif

#if defined(XSFC_SPI_0)
  #include <SPI.h>
  #if !defined(XSFC_SPI_0_DEV)
    #define XSFC_SPI_0_DEV SPI
  #endif

  #if !defined(XSFC_SPI_0_DEV_T)
    #define XSFC_SPI_0_DEV_T SPIClass
  #endif
  #if defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_SPI)
    #if !defined(ARCH_RP2040)
      XSFC_SPI_0_DEV_T XSFC_SPI_0_DEV;
    #endif
  #endif
#endif

namespace Xsfc {

enum SerialPort {
#ifdef XSFC_SERIAL_USB
  SERIAL_USB,
#endif
#ifdef XSFC_SERIAL_0
  SERIAL_UART_0,
#endif
#ifdef XSFC_SERIAL_1
  SERIAL_UART_1,
#endif
#ifdef XSFC_SERIAL_2
  SERIAL_UART_2,
#endif
#ifdef XSFC_SERIAL_SOFT_0
  SERIAL_SOFT_0,
#endif
  SERIAL_UART_COUNT
};

}
