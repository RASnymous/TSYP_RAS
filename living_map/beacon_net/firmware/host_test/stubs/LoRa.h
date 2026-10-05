/* Host-test stand-in for "LoRa" by Sandeep Mistry: same signatures for the
 * members the firmware uses (checked against the real header, v0.8.0). */
#pragma once
#include <Arduino.h>
#include <SPI.h>
#define PA_OUTPUT_RFO_PIN 0
#define PA_OUTPUT_PA_BOOST_PIN 1
class LoRaClass : public Stream {
public:
  LoRaClass();
  int begin(long frequency);
  int beginPacket(int implicitHeader = false);
  int endPacket(bool async = false);
  int parsePacket(int size = 0);
  int packetRssi();
  float packetSnr();
  virtual size_t write(uint8_t byte);
  virtual size_t write(const uint8_t *buffer, size_t size);
  virtual int available();
  virtual int read();
  virtual int peek();
  virtual void flush();
  void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
  void setSpreadingFactor(int sf);
  void setSignalBandwidth(long sbw);
  void setCodingRate4(int denominator);
  void setPreambleLength(long length);
  void setSyncWord(int sw);
  void enableCrc();
  void setPins(int ss = 10, int reset = 9, int dio0 = 2);
};
extern LoRaClass LoRa;
