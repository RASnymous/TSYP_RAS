/* Host implementation of the Arduino API subset used by the firmware, so the
 * REAL sketches run as PC processes (make -C host firmware-test):
 *   Serial = stdin/stdout, LoRa TX = "TX <len> B <hex>" on stderr (no RX),
 *   NVS = a file (env NVS_FILE), millis() = wall clock. */
#include <Arduino.h>
#include <LoRa.h>
#include <Preferences.h>
#include <SPI.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <map>
#include <vector>
static unsigned long t0;
unsigned long millis() { struct timeval tv; gettimeofday(&tv, 0); unsigned long ms = (unsigned long)(tv.tv_sec * 1000 + tv.tv_usec / 1000); if (!t0) t0 = ms; return ms - t0; }
void delay(unsigned long ms) { usleep((useconds_t)ms * 1000); }
void pinMode(int, int) {}
void digitalWrite(int, int) {}
uint32_t esp_random() { return (uint32_t)random(); }
bool setCpuFrequencyMhz(uint32_t) { return true; }
size_t Print::printf(const char *fmt, ...) { va_list a; va_start(a, fmt); int n = vprintf(fmt, a); va_end(a); fflush(stdout); return (size_t)n; }
size_t Print::println(const char *s) { return (size_t)::printf("%s\n", s); }
size_t Print::println(const String &s) { return println(s.c_str()); }
size_t Print::print(const char *s) { return (size_t)::printf("%s", s); }
HardwareSerial Serial;
static int peeked = -2;
void HardwareSerial::begin(unsigned long) { fcntl(0, F_SETFL, O_NONBLOCK); }
size_t HardwareSerial::write(uint8_t c) { putchar(c); return 1; }
int HardwareSerial::available() { if (peeked == -2) { unsigned char c; ssize_t r = ::read(0, &c, 1); if (r == 1) peeked = c; else { if (r == 0) { static int eof_at = 0; if (!eof_at) eof_at = (int)millis(); if ((int)millis() - eof_at > 1500) exit(0); } return 0; } } return 1; }
int HardwareSerial::read() { if (!available()) return -1; int c = peeked; peeked = -2; return c; }
int HardwareSerial::peek() { return available() ? peeked : -1; }
EspClass ESP; void EspClass::restart() { ::printf("[restart]\n"); exit(0); }
SPIClass SPI; void SPIClass::begin(int8_t, int8_t, int8_t, int8_t) {}
/* LoRa mock */
LoRaClass LoRa;
static std::vector<uint8_t> txb;
LoRaClass::LoRaClass() {}
int LoRaClass::begin(long) { return 1; }
void LoRaClass::setPins(int, int, int) {}
void LoRaClass::setSpreadingFactor(int) {} void LoRaClass::setSignalBandwidth(long) {} void LoRaClass::setCodingRate4(int) {}
void LoRaClass::setPreambleLength(long) {} void LoRaClass::setSyncWord(int) {} void LoRaClass::enableCrc() {} void LoRaClass::setTxPower(int, int) {}
/* v9: optional RX injection for tests: env LORA_RX_FILE = a file of hex frames, one per line,
 * delivered one by one (every 100 ms) as if heard over the air */
static std::vector<uint8_t> rxb; static size_t rxi = 0; static FILE *rxf = nullptr; static unsigned long rx_next = 0;
int LoRaClass::parsePacket(int) {
  const char *fn = getenv("LORA_RX_FILE");
  if (!fn) return 0;
  if (!rxf) { rxf = fopen(fn, "r"); if (!rxf) return 0; }
  if (millis() < rx_next) return 0;
  char line[600];
  if (!fgets(line, sizeof line, rxf)) return 0;
  rx_next = millis() + 100;
  rxb.clear(); rxi = 0;
  for (char *c = line; c[0] && c[1] && c[0] != '\n'; c += 2) { unsigned v; if (sscanf(c, "%2x", &v) != 1) break; rxb.push_back((uint8_t)v); }
  return (int)rxb.size();
}
int LoRaClass::packetRssi() { return -90; } float LoRaClass::packetSnr() { return 7.5f; }
int LoRaClass::available() { return rxi < rxb.size() ? 1 : 0; } int LoRaClass::read() { return rxi < rxb.size() ? rxb[rxi++] : -1; } int LoRaClass::peek() { return rxi < rxb.size() ? rxb[rxi] : -1; } void LoRaClass::flush() {}
int LoRaClass::beginPacket(int) { txb.clear(); return 1; }
size_t LoRaClass::write(uint8_t b) { txb.push_back(b); return 1; }
size_t LoRaClass::write(const uint8_t *b, size_t n) { txb.insert(txb.end(), b, b + n); return n; }
int LoRaClass::endPacket(bool) { fprintf(stderr, "TX %3zu B ", txb.size()); for (auto c : txb) fprintf(stderr, "%02x", c); fprintf(stderr, "\n"); return 1; }
/* Preferences mock (file-backed) */
static std::map<std::string, std::vector<uint8_t>> nvs;
static const char *NVS_FILE = getenv("NVS_FILE") ? getenv("NVS_FILE") : "nvs_test.bin";
static void nvs_load() { FILE *f = fopen(NVS_FILE, "rb"); if (!f) return; char k[64]; uint32_t n; while (fread(k, 64, 1, f) == 1 && fread(&n, 4, 1, f) == 1) { std::vector<uint8_t> v(n); if (fread(v.data(), 1, n, f) != n) break; nvs[k] = v; } fclose(f); }
static void nvs_save() { FILE *f = fopen(NVS_FILE, "wb"); for (auto &kv : nvs) { char k[64] = {0}; strncpy(k, kv.first.c_str(), 63); uint32_t n = (uint32_t)kv.second.size(); fwrite(k, 64, 1, f); fwrite(&n, 4, 1, f); fwrite(kv.second.data(), 1, n, f); } fclose(f); }
Preferences::Preferences() {} Preferences::~Preferences() {}
bool Preferences::begin(const char *, bool, const char *) { nvs_load(); return true; }
void Preferences::end() { nvs_save(); }
bool Preferences::clear() { nvs.clear(); nvs_save(); return true; }
size_t Preferences::putBytes(const char *k, const void *v, size_t n) { nvs[k] = std::vector<uint8_t>((const uint8_t *)v, (const uint8_t *)v + n); return n; }
size_t Preferences::getBytesLength(const char *k) { return nvs.count(k) ? nvs[k].size() : 0; }
size_t Preferences::getBytes(const char *k, void *b, size_t n) { if (!nvs.count(k)) return 0; size_t m = std::min(n, nvs[k].size()); memcpy(b, nvs[k].data(), m); return m; }
void setup(); void loop();
int main() { setup(); for (;;) { loop(); usleep(1000); } }
