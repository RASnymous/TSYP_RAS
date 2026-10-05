/* Host-test stand-in for the ESP32 Preferences (NVS) library: same signatures
 * for the members the firmware uses. Backed by a file (env NVS_FILE). */
#pragma once
#include <Arduino.h>
class Preferences {
public:
  Preferences();
  ~Preferences();
  bool begin(const char *name, bool readOnly = false, const char *partition_label = NULL);
  void end();
  bool clear();
  size_t putBytes(const char *key, const void *value, size_t len);
  size_t getBytesLength(const char *key);
  size_t getBytes(const char *key, void *buf, size_t maxLen);
};
