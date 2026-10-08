#ifndef UNIT_TEST

#include "Utils/Storage.h"
#include "ModelConfig.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <cstddef>

#if defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_EEPROM)
static EEPROMClass EEPROM;
#endif

namespace Xsfc {

namespace Utils {

int Storage::begin()
{
  EEPROM.begin(EEPROM_SIZE);
  static_assert(sizeof(ModelConfig) <= EEPROM_SIZE, "ModelConfig Size too big");
  return 1;
}

StorageResult Storage::load(ModelConfig& config) const
{
  //return STORAGE_ERR_BAD_MAGIC;

  int addr = 0;
  uint8_t magic = EEPROM.read(addr++);
  if(EEPROM_MAGIC != magic)
  {
    return STORAGE_ERR_BAD_MAGIC;
  }

  uint8_t version = EEPROM.read(addr++);
  if (version != EEPROM_VERSION && version != 0x01 && version != 0x02 &&
      version != 0x03 && version != 0x04)
  {
    return STORAGE_ERR_BAD_VERSION;
  }

  uint16_t size = 0;
  size = EEPROM.read(addr++);
  size |= EEPROM.read(addr++) << 8;
  constexpr size_t prefixEnd = offsetof(ModelConfig, modelName) + MODEL_NAME_LEN + 1;
  constexpr size_t navigationEnd = offsetof(ModelConfig, navigation) + sizeof(NavigationSensorConfig);
  const auto aligned = [](size_t n) { return (n + alignof(ModelConfig) - 1) / alignof(ModelConfig) * alignof(ModelConfig); };
  // Autumn appended a 6-byte landing config, then a 2-byte polling limit.
  const bool summer = version == 0x01 && size == aligned(prefixEnd);
  const bool navigation = (version == EEPROM_VERSION || version == 0x02) && size == sizeof(ModelConfig);
  const bool autumn3 = version == 0x03 && size == aligned(navigationEnd + 6);
  const bool autumn4 = version == 0x04 && size == aligned(navigationEnd + 8);
  if (!summer && !navigation && !autumn3 && !autumn4)
  {
    return STORAGE_ERR_BAD_SIZE;
  }

  // Do not copy legacy tail padding onto newly appended navigation fields.
  auto* bytes = reinterpret_cast<uint8_t*>(&config);
  const size_t copySize = summer ? prefixEnd : navigationEnd;
  for (size_t i = 0; i < copySize; ++i) bytes[i] = EEPROM.read(addr + i);
  if (summer)
  {
    const ModelConfig defaults;
    config.navigation = defaults.navigation;
    // Summer navigation gains used different units; only these need conversion.
    config.altHold = defaults.altHold;
    config.posHold = defaults.posHold;
    for (const auto index : {FC_PID_ALT, FC_PID_VEL, FC_PID_POS, FC_PID_POSR})
      config.pid[index] = defaults.pid[index];
  }
  return STORAGE_LOAD_SUCCESS;
}

StorageResult Storage::save(const ModelConfig& config)
{
  int addr = 0;
  uint16_t size = sizeof(ModelConfig);
  EEPROM.write(addr++, EEPROM_MAGIC);
  EEPROM.write(addr++, EEPROM_VERSION);
  EEPROM.write(addr++, size & 0xFF);
  EEPROM.write(addr++, (size >> 8) & 0xFF);
  EEPROM.put(addr, config);
  bool ok = EEPROM.commit();
  if(!ok) return STORAGE_SAVE_ERROR;
  return STORAGE_SAVE_SUCCESS;
}

}

}

#endif
