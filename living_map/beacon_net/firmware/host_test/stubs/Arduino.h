/* Minimal Arduino-ESP32 API stub: enough to type-check the sketches on a PC. */
#pragma once
#define ARDUINO 10819
#define ESP32 1
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <string>
#include <algorithm>
typedef uint8_t byte;
typedef bool boolean;
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define MSBFIRST 1
#define SPI_MODE0 0
#define PI 3.14159265358979
unsigned long millis();
void delay(unsigned long);
void pinMode(int, int);
void digitalWrite(int, int);
uint32_t esp_random();
bool setCpuFrequencyMhz(uint32_t);
class String {
  std::string s;
public:
  String() {}
  String(const char *c) : s(c ? c : "") {}
  String(const std::string &x) : s(x) {}
  unsigned int length() const { return (unsigned)s.size(); }
  const char *c_str() const { return s.c_str(); }
  char operator[](unsigned i) const { return s[i]; }
  int indexOf(char c) const { size_t p = s.find(c); return p == std::string::npos ? -1 : (int)p; }
  String substring(unsigned a) const { return String(s.substr(a)); }
  String substring(unsigned a, unsigned b) const { return String(s.substr(a, b - a)); }
  void trim() { size_t a = s.find_first_not_of(" \t\r\n"); if (a == std::string::npos) { s.clear(); return; } s = s.substr(a, s.find_last_not_of(" \t\r\n") - a + 1); }
  void toUpperCase() { for (auto &c : s) c = (char)toupper(c); }
  bool equalsIgnoreCase(const String &o) const { return strcasecmp(s.c_str(), o.s.c_str()) == 0; }
  long toInt() const { return atol(s.c_str()); }
  bool operator==(const char *c) const { return s == c; }
  String &operator+=(char c) { s += c; return *this; }
  String &operator=(const char *c) { s = c; return *this; }
};
class Print {
public:
  virtual ~Print() {}
  virtual size_t write(uint8_t) = 0;
  virtual size_t write(const uint8_t *b, size_t n) { size_t i; for (i = 0; i < n; ++i) write(b[i]); return n; }
  size_t printf(const char *fmt, ...) __attribute__((format(printf, 2, 3)));
  size_t println(const char *);
  size_t println(const String &);
  size_t print(const char *);
};
class Stream : public Print {
public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() = 0;
  virtual void flush() {}
};
class HardwareSerial : public Stream {
public:
  void begin(unsigned long);
  size_t write(uint8_t) override;
  int available() override;
  int read() override;
  int peek() override;
};
extern HardwareSerial Serial;
struct EspClass { void restart(); };
extern EspClass ESP;
