/*
 * bp_arduino.h - ESP32 + SX1276 glue for the Living Map beacon protocol.
 * ==========================================================================
 * Shared by firmware/beacon_node and firmware/gateway_node. Everything that
 * is protocol logic lives in the portable C core (beacon_proto / bp_gossip /
 * bp_json); this header only connects it to the hardware:
 *
 *   radio    SX1276 / RFM95 through the "LoRa" library (Sandeep Mistry)
 *   clock    millis()           random  esp_random() (hardware RNG)
 *   storage  NVS via Preferences: key, net id, node id, own record + seq
 *   console  line-based serial commands (115200 baud), replies "OK ..." /
 *            "ERR ..." or one JSON object per line
 *
 * Override any BP_* setting with a #define BEFORE including this header.
 * Only compiled on Arduino targets.
 */
#ifndef BP_ARDUINO_H
#define BP_ARDUINO_H
#ifdef ARDUINO

#include <Arduino.h>
#include <LoRa.h>
#include <Preferences.h>
#include <SPI.h>

#include "beacon_proto.h"
#include "bp_gossip.h"
#include "bp_json.h"

/* ---- SX1276 wiring: defaults = TTGO LoRa32 (V1 / V2.1). RFM95 on a bare
 *      ESP32 devkit: wire it and change these six. ---------------------- */
#ifndef BP_PIN_SCK
#define BP_PIN_SCK 5
#endif
#ifndef BP_PIN_MISO
#define BP_PIN_MISO 19
#endif
#ifndef BP_PIN_MOSI
#define BP_PIN_MOSI 27
#endif
#ifndef BP_PIN_SS
#define BP_PIN_SS 18
#endif
#ifndef BP_PIN_RST
#define BP_PIN_RST 14
#endif
#ifndef BP_PIN_DIO0
#define BP_PIN_DIO0 26
#endif
#ifndef BP_PIN_LED
#define BP_PIN_LED -1           /* e.g. 25 on TTGO LoRa32 V2.1, 2 on many devkits */
#endif

/* ---- Radio: EU868 sub-band g1 (868.0-868.6 MHz, 25 mW ERP, 1 % duty) ---- */
#ifndef BP_LORA_FREQ_HZ
#define BP_LORA_FREQ_HZ 868100000L
#endif
#ifndef BP_LORA_SF
#define BP_LORA_SF 9            /* 44 B -> 288 ms on air                         */
#endif
#ifndef BP_LORA_BW_HZ
#define BP_LORA_BW_HZ 125000L
#endif
#ifndef BP_LORA_CR
#define BP_LORA_CR 5            /* 4/5 */
#endif
#ifndef BP_LORA_SYNC
#define BP_LORA_SYNC 0x12       /* private network (0x34 = LoRaWAN public)       */
#endif
#ifndef BP_LORA_TX_DBM
#define BP_LORA_TX_DBM 14       /* + ~2 dBi antenna stays within 25 mW ERP       */
#endif

#define BP_DEMO_KEY "4c6976696e674d61702d44454d4f2121"   /* "LivingMap-DEMO!!" */

namespace bpa {

/* what survives a power cycle */
struct Persist {
    uint32_t    magic;          /* 'LMB2' */
    uint8_t     key[BP_KEY_LEN];
    uint8_t     key_set;
    uint8_t     net;
    uint16_t    id;
    uint8_t     has_own;
    bp_record_t own;
    char        name[12];
};
static const uint32_t PERSIST_MAGIC = 0x324D424CUL;

class App;
typedef void (*EventFn)(App &app, int ev, const bp_entry_t *e, int info);
typedef bool (*CommandFn)(App &app, const String &cmd, const String &arg);  /* true = handled */
/* v9: every packet heard, before the gossip engine sees it (the gateway passes them to the ONA) */
typedef void (*RawRxFn)(App &app, const uint8_t *buf, int len, int rssi, float snr);

class App {
public:
    bp_node_t   node;
    bp_config_t cfg;
    Persist     p;
    bool        radio_ok = false;
    EventFn     on_event = nullptr;
    CommandFn   on_command = nullptr;
    RawRxFn     on_raw_rx = nullptr;

    /* ---------------------------------------------------------------- setup */
    void begin(uint16_t default_id, bool relay, uint32_t digest_period_ms) {
        default_id_ = default_id;
        relay_ = relay;
        digest_ms_ = digest_period_ms;
        if (BP_PIN_LED >= 0) pinMode(BP_PIN_LED, OUTPUT);
        load();
        SPI.begin(BP_PIN_SCK, BP_PIN_MISO, BP_PIN_MOSI, BP_PIN_SS);
        LoRa.setPins(BP_PIN_SS, BP_PIN_RST, BP_PIN_DIO0);
        radio_ok = LoRa.begin(BP_LORA_FREQ_HZ) == 1;
        if (radio_ok) {
            LoRa.setSpreadingFactor(BP_LORA_SF);
            LoRa.setSignalBandwidth(BP_LORA_BW_HZ);
            LoRa.setCodingRate4(BP_LORA_CR);
            LoRa.setPreambleLength(8);
            LoRa.setSyncWord(BP_LORA_SYNC);
            LoRa.enableCrc();
            LoRa.setTxPower(BP_LORA_TX_DBM);
        }
        restart_engine();
        Serial.printf("# LMB2 node %u net 0x%02X %s | radio %s | %.1f MHz SF%d BW%ldk CR4/%d | key %s\n",
                      (unsigned)p.id, (unsigned)p.net, relay_ ? "beacon" : "gateway",
                      radio_ok ? "ok" : "NOT FOUND", BP_LORA_FREQ_HZ / 1e6, BP_LORA_SF,
                      (long)(BP_LORA_BW_HZ / 1000), BP_LORA_CR, p.key_set ? "set" : "MISSING (send KEY)");
    }

    /* (re)build the gossip engine from the persisted settings */
    void restart_engine() {
        bp_hal_t h;
        bp_config_defaults(&cfg);
        memcpy(cfg.key, p.key, BP_KEY_LEN);
        cfg.net_id = p.net;
        cfg.my_id = p.id;
        cfg.relay_enabled = relay_ ? 1 : 0;
        cfg.digest_period_ms = digest_ms_;
        cfg.lora_sf = BP_LORA_SF;
        cfg.lora_bw_hz = (uint32_t)BP_LORA_BW_HZ;
        cfg.lora_cr_denom = BP_LORA_CR;
        h.now_ms = hal_now;
        h.radio_tx = hal_tx;
        h.rand32 = hal_rand;
        h.on_event = hal_event;
        h.ctx = this;
        bp_node_init(&node, &cfg, &h);
        if (clock_unix_) bp_node_set_time(&node, clock_unix_ + (millis() - clock_millis_) / 1000);
        if (p.has_own) bp_node_set_own(&node, &p.own);
    }

    /* ---------------------------------------------------------------- loop */
    void loop() {
        poll_serial();
        if (!radio_ok) return;
        int sz = LoRa.parsePacket();          /* polls RX-done, re-arms RX when idle */
        if (sz > 0) {
            uint8_t buf[256];
            int n = 0;
            while (LoRa.available() && n < (int)sizeof buf) buf[n++] = (uint8_t)LoRa.read();
            int rssi = LoRa.packetRssi();
            float snr = LoRa.packetSnr();
            if (on_raw_rx) on_raw_rx(*this, buf, n, rssi, snr);
            if (p.key_set)
                bp_node_on_rx(&node, buf, (uint8_t)n, (int16_t)rssi, (int16_t)(snr * 10.0f));
        }
        if (p.key_set) bp_node_tick(&node);  /* never transmit without a key */
    }

    /* ---------------------------------------------------------------- helpers */
    void save() {
        prefs_.begin("lmb2", false);
        prefs_.putBytes("cfg", &p, sizeof p);
        prefs_.end();
    }

    void set_clock(uint32_t unix_s) {
        clock_unix_ = unix_s;
        clock_millis_ = millis();
        bp_node_set_time(&node, unix_s);
    }

    void print_entry(const bp_entry_t *e, const char *ev) {
        char json[640];
        if (bp_entry_to_json(&node, e, ev, json, sizeof json) > 0) Serial.println(json);
    }

    void print_status() {
        const bp_stats_t &s = node.st;
        Serial.printf("{\"ev\":\"status\",\"id\":%u,\"net\":%u,\"role\":\"%s\",\"name\":\"%s\",\"key\":%s,"
                      "\"radio\":%s,\"clock\":%s,\"unix\":%lu,\"known\":%d,\"duty\":%.4f,\"budget_ms\":%.0f,"
                      "\"round_s\":%.1f,\"tx\":{\"own\":%lu,\"relay\":%lu,\"ae\":%lu,\"sync\":%lu,\"digest\":%lu,"
                      "\"retries\":%lu},\"rx\":{\"ok\":%lu,\"new\":%lu,\"update\":%lu,\"dup\":%lu,\"bad_mac\":%lu,"
                      "\"bad\":%lu,\"stale\":%lu},\"suppressed\":%lu}\n",
                      (unsigned)p.id, (unsigned)p.net, relay_ ? "beacon" : "gateway", p.name,
                      p.key_set ? "true" : "false", radio_ok ? "true" : "false",
                      node.clock_set ? "true" : "false", (unsigned long)bp_node_unix(&node),
                      bp_node_count(&node), (double)bp_node_duty_used(&node), (double)node.budget_ms,
                      node.cfg.round_ms / 1000.0, (unsigned long)s.tx_own, (unsigned long)s.tx_relay,
                      (unsigned long)s.tx_ae, (unsigned long)s.tx_sync, (unsigned long)s.tx_digest,
                      (unsigned long)s.ack_retries, (unsigned long)s.rx_ok, (unsigned long)s.rx_new,
                      (unsigned long)s.rx_update, (unsigned long)s.rx_dup, (unsigned long)s.rx_bad_mac,
                      (unsigned long)s.rx_bad_other, (unsigned long)s.rx_stale, (unsigned long)s.suppressed);
    }

    /* v9: send a frame that does not come from the gossip engine (a mission briefing
     * from the ONA). Paid for from the same duty-cycle bucket: refused (-2) when the
     * credit is short, so the 1 % rule still holds. */
    int transmit_raw(const uint8_t *buf, uint8_t len) {
        if (!radio_ok) return -1;
        float air = (float)bp_lora_airtime_ms(len, (uint8_t)BP_LORA_SF, (uint32_t)BP_LORA_BW_HZ,
                                              (uint8_t)BP_LORA_CR, 8, 1, 0);
        if (node.budget_ms < air) return -2;
        node.budget_ms -= air;
        return hal_tx(this, buf, len);      /* parsePacket() re-arms RX, as after a gossip TX */
    }

    void blink() {
        if (BP_PIN_LED >= 0) { digitalWrite(BP_PIN_LED, HIGH); led_off_ms_ = millis() + 60; }
    }

private:
    Preferences prefs_;
    uint16_t default_id_ = 1;
    bool     relay_ = true;
    uint32_t digest_ms_ = 0;
    uint32_t clock_unix_ = 0, clock_millis_ = 0;
    uint32_t led_off_ms_ = 0;
    String   line_;

    void load() {
        memset(&p, 0, sizeof p);
        prefs_.begin("lmb2", true);
        size_t n = prefs_.getBytesLength("cfg");
        if (n == sizeof p) prefs_.getBytes("cfg", &p, sizeof p);
        prefs_.end();
        if (p.magic != PERSIST_MAGIC) {                 /* first boot / factory reset */
            memset(&p, 0, sizeof p);
            p.magic = PERSIST_MAGIC;
            p.net = 0x2A;
            p.id = default_id_;
            snprintf(p.name, sizeof p.name, relay_ ? "b%u" : "gw%u", (unsigned)(relay_ ? p.id : 1));
        }
    }

    /* ---- HAL ---- */
    static uint32_t hal_now(void *) { return millis(); }
    static uint32_t hal_rand(void *) { return esp_random(); }
    static int hal_tx(void *ctx, const uint8_t *buf, uint8_t len) {
        App *a = static_cast<App *>(ctx);
        if (!LoRa.beginPacket()) return -1;
        LoRa.write(buf, len);
        if (!LoRa.endPacket()) return -1;     /* blocking: returns after time-on-air */
        a->blink();
        return 0;
    }
    static void hal_event(void *ctx, int ev, const bp_node *, const bp_entry_t *e, int info) {
        App *a = static_cast<App *>(ctx);
        if (a->on_event) a->on_event(*a, ev, e, info);
    }

    /* ---- serial console ---- */
    void poll_serial() {
        if (led_off_ms_ && (int32_t)(millis() - led_off_ms_) >= 0) {
            digitalWrite(BP_PIN_LED, LOW);
            led_off_ms_ = 0;
        }
        while (Serial.available()) {
            char c = (char)Serial.read();
            if (c == '\r') continue;
            if (c == '\n') { handle_line(line_); line_ = ""; }
            else if (line_.length() < 600) line_ += c;
        }
    }

    void handle_line(String l) {
        l.trim();
        if (!l.length() || l[0] == '#') return;
        int sp = l.indexOf(' ');
        String cmd = sp < 0 ? l : l.substring(0, sp);
        String arg = sp < 0 ? String("") : l.substring(sp + 1);
        cmd.toUpperCase();
        arg.trim();

        if (cmd == "KEY") {
            if (arg.equalsIgnoreCase("demo")) arg = BP_DEMO_KEY;
            if (bp_key_from_hex(arg.c_str(), p.key) != BP_OK) { Serial.println("ERR KEY needs 32 hex chars"); return; }
            p.key_set = 1;
            save();
            restart_engine();
            Serial.println("OK KEY");
        } else if (cmd == "NET") {
            long v = arg.toInt();
            if (v < 0 || v > 255 || !arg.length()) { Serial.println("ERR NET 0..255"); return; }
            p.net = (uint8_t)v;
            save();
            restart_engine();
            Serial.printf("OK NET %u\n", (unsigned)p.net);
        } else if (cmd == "ID") {
            long v = arg.toInt();
            if (v < 1 || v > 65534) { Serial.println("ERR ID 1..65534"); return; }
            if (p.has_own && p.own.origin_id != (uint16_t)v) p.has_own = 0;   /* record belonged to old id */
            p.id = (uint16_t)v;
            save();
            restart_engine();
            Serial.printf("OK ID %u\n", (unsigned)p.id);
        } else if (cmd == "NAME") {
            snprintf(p.name, sizeof p.name, "%s", arg.c_str());
            save();
            Serial.printf("OK NAME %s\n", p.name);
        } else if (cmd == "TIME") {
            unsigned long v = strtoul(arg.c_str(), nullptr, 10);
            if (v < 1600000000UL) { Serial.println("ERR TIME <unix seconds>"); return; }
            set_clock((uint32_t)v);
            Serial.printf("OK TIME %lu\n", v);
        } else if (cmd == "STATUS") {
            print_status();
        } else if (cmd == "DUMP") {
            for (int i = 0; i < BP_CACHE_MAX; ++i)
                if (node.cache[i].used) print_entry(&node.cache[i], node.cache[i].is_own ? "own" : "dump");
            Serial.println("OK DUMP");
        } else if (cmd == "REBOOT") {
            Serial.println("OK REBOOT");
            delay(50);
            ESP.restart();
        } else if (cmd == "FACTORY") {
            prefs_.begin("lmb2", false);
            prefs_.clear();
            prefs_.end();
            Serial.println("OK FACTORY (rebooting)");
            delay(50);
            ESP.restart();
        } else if (on_command && on_command(*this, cmd, arg)) {
            /* role-specific */
        } else if (cmd == "HELP") {
            Serial.println("# KEY <32hex>|demo  NET <n>  ID <n>  NAME <s>  TIME <unix>  STATUS  DUMP  REBOOT  FACTORY");
        } else {
            Serial.printf("ERR unknown command %s (HELP)\n", cmd.c_str());
        }
    }
};

}  // namespace bpa

#endif /* ARDUINO */
#endif /* BP_ARDUINO_H */
