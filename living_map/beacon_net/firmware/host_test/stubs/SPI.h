#pragma once
#include <Arduino.h>
class SPISettings { public: SPISettings() {} SPISettings(uint32_t, uint8_t, uint8_t) {} };
class SPIClass { public: void begin(int8_t sck = -1, int8_t miso = -1, int8_t mosi = -1, int8_t ss = -1); void beginTransaction(SPISettings); void endTransaction(); uint8_t transfer(uint8_t); void end(); };
extern SPIClass SPI;
