#pragma once

#include <Arduino.h>

#define XSFC_SPI_0 1
#define XSFC_SPI_0_DEV_T TestSpi
#define FAST_CODE_ATTR
#define LOW 0
#define HIGH 1

inline void targetSPIInit(TestSpi&, int8_t, int8_t, int8_t, int8_t) {}
