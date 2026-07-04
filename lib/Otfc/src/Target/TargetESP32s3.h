#pragma once

#include "Esp.h"
#include "Debug_Otfc.h"

// pins to avoid:
// strapping: 0, 3, 45, 46
// flash/psram: 26-37 (reserved)
// usb/jtag: 19, 20

#define OTFC_INPUT
#define OTFC_INPUT_PIN 6 // ppm

#define OTFC_OUTPUT_COUNT 4
#define OTFC_OUTPUT_0 39
#define OTFC_OUTPUT_1 40
#define OTFC_OUTPUT_2 41
#define OTFC_OUTPUT_3 42

#define OTFC_SERIAL_0
#define OTFC_SERIAL_0_DEV Serial0
#define OTFC_SERIAL_0_DEV_T HardwareSerial
#define OTFC_SERIAL_0_TX 43
#define OTFC_SERIAL_0_RX 44
#define OTFC_SERIAL_0_FN (SERIAL_FUNCTION_MSP)
#define OTFC_SERIAL_0_BAUD (SERIAL_SPEED_115200)
#define OTFC_SERIAL_0_BBAUD (SERIAL_SPEED_NONE)

#define OTFC_SERIAL_1
#define OTFC_SERIAL_1_DEV Serial1
#define OTFC_SERIAL_1_DEV_T HardwareSerial
#define OTFC_SERIAL_1_TX 16
#define OTFC_SERIAL_1_RX 15
#define OTFC_SERIAL_1_FN (SERIAL_FUNCTION_MSP)
#define OTFC_SERIAL_1_BAUD (SERIAL_SPEED_115200)
#define OTFC_SERIAL_1_BBAUD (SERIAL_SPEED_NONE)

#define OTFC_SERIAL_2
#define OTFC_SERIAL_2_DEV Serial2
#define OTFC_SERIAL_2_DEV_T HardwareSerial
#define OTFC_SERIAL_2_TX 18
#define OTFC_SERIAL_2_RX 17
#define OTFC_SERIAL_2_FN (SERIAL_FUNCTION_RX_SERIAL)
#define OTFC_SERIAL_2_BAUD (SERIAL_SPEED_115200)
#define OTFC_SERIAL_2_BBAUD (SERIAL_SPEED_NONE)

#define OTFC_SERIAL_USB
#define OTFC_SERIAL_USB_DEV Serial
#define OTFC_SERIAL_USB_DEV_T HWCDC
#define OTFC_SERIAL_USB_FN (SERIAL_FUNCTION_MSP)

#define OTFC_SERIAL_SOFT_0
#define OTFC_SERIAL_SOFT_0_FN (SERIAL_FUNCTION_MSP)
#define OTFC_SERIAL_SOFT_0_WIFI

#define OTFC_SERIAL_REMAP_PINS
#define OTFC_SERIAL_DEBUG_PORT SERIAL_USB
#define SERIAL_TX_FIFO_SIZE 0xFF

#define OTFC_SPI_0
#define OTFC_SPI_0_DEV SPI1
#define OTFC_SPI_0_SCK 12
#define OTFC_SPI_0_MOSI 11
#define OTFC_SPI_0_MISO 13

#define OTFC_SPI_CS_GYRO 8
#define OTFC_SPI_CS_BARO 7

#define OTFC_I2C_0
#define OTFC_I2C_0_SCL 10
#define OTFC_I2C_0_SDA 9
#define OTFC_I2C_0_SOFT

#define OTFC_BUZZER_PIN 5
#define OTFC_BUTTON_PIN -1
#define OTFC_LED_PIN -1

#define OTFC_ADC_0
#define OTFC_ADC_0_PIN 1

#define OTFC_ADC_1
#define OTFC_ADC_1_PIN 4

#define OTFC_ADC_SCALE (3.3f / 4096)

#define OTFC_FEATURE_MASK (FEATURE_RX_SERIAL | FEATURE_DYNAMIC_FILTER)

#define OTFC_GYRO_I2C_RATE_MAX 2000
#define OTFC_GYRO_SPI_RATE_MAX 4000

#define OTFC_DSHOT_TELEMETRY

#define OTFC_FREE_RTOS
#ifndef CONFIG_FREERTOS_UNICORE
  #define OTFC_MULTI_CORE
#endif

//#define OTFC_FREE_RTOS_QUEUE
#define OTFC_ATOMIC_QUEUE

#define OTFC_DSP

#include "Device/SerialDevice.h"

#include "Target/TargetEsp32Common.h"

namespace Otfc {

template<>
inline int targetSerialInit(HWCDC& dev, const SerialDeviceConfig& conf)
{
  dev.begin(conf.baud);
  //dev.setTxTimeoutMs(10);
  //while(!dev) delay(10);
  return 1;
}

}
