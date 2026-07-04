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

#if defined(OTFC_I2C_0)
  #if defined(OTFC_I2C_0_SOFT)
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

#if defined(OTFC_SPI_0)
  #include <SPI.h>
  #if !defined(OTFC_SPI_0_DEV)
    #define OTFC_SPI_0_DEV SPI
  #endif

  #if !defined(OTFC_SPI_0_DEV_T)
    #define OTFC_SPI_0_DEV_T SPIClass
  #endif
  #if defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_SPI)
    #if !defined(ARCH_RP2040)
      OTFC_SPI_0_DEV_T OTFC_SPI_0_DEV;
    #endif
  #endif
#endif

namespace Otfc {

enum SerialPort {
#ifdef OTFC_SERIAL_USB
  SERIAL_USB,
#endif
#ifdef OTFC_SERIAL_0
  SERIAL_UART_0,
#endif
#ifdef OTFC_SERIAL_1
  SERIAL_UART_1,
#endif
#ifdef OTFC_SERIAL_2
  SERIAL_UART_2,
#endif
#ifdef OTFC_SERIAL_SOFT_0
  SERIAL_SOFT_0,
#endif
  SERIAL_UART_COUNT
};

}
