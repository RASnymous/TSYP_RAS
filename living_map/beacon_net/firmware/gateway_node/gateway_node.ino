/* ============================================================================
 * Living Map GATEWAY firmware - TSYP14 IEEE RAS x AESS
 * ESP32 + SX1276/RFM95 on USB, one of the three gateways of the Outside
 * Network Area (ONA), or on the Executor robot. Same radio settings and
 * protocol as the beacons.
 * ============================================================================
 * The gateway is a passive member of the gossip network:
 *   - verifies every frame (HMAC, net id, replay/version, timestamp sanity)
 *     and keeps its own replica of the map,
 *   - never relays (relay_enabled = 0), but every ~90 s broadcasts a signed
 *     DIGEST of the (origin, seq) pairs it holds: beacons in range push
 *     exactly what is missing (pull gossip). A gateway switched on late, or
 *     rebooted, rebuilds the whole map in about a minute instead of waiting
 *     for anti-entropy to cycle through every record,
 *   - prints one JSON object per line on USB serial:
 *       {"ev":"new"|"update"|"silent"|"revived"|"expired", <record>, rx meta}
 *       {"ev":"reject","reason":"bad_mac"|...}
 *       {"ev":"hb","gw":"gw1",...}                      every 5 s
 *       {"ev":"rx","gw":"gw1","frame":"<88 hex>","rssi":..,"snr":..}   (v9)
 *          every packet heard, as received: records, robot position reports
 *          (0x23) and briefings (0x24). The ONA re-checks the signature itself
 *          and votes across its three gateways (2 of 3 must agree).
 *     ona/ (python -m ona --serial gw1=COM3 ...) or, for one gateway without
 *     the ONA, python/beaconnet/gateway_bridge.py forward them to the Command Post.
 *
 * Commands (115200 baud): KEY <32hex>|demo, NET, ID, NAME gw1, TIME <unix>
 * (the ONA / bridge send TIME automatically), STATUS, DUMP, HELP,
 * TX <88 hex>  (v9) transmit a 44-byte frame (the ONA's mission briefing for
 *              the Executor), paid from the 1 % duty-cycle budget,
 * RAW 0|1      (v9) the "rx" lines on (default) or off.
 *
 * Ranging (v9): the ONA locates robots from each gateway's distance to them.
 * An SX1276 cannot measure distance; a gateway with a ranging radio (SX1280
 * time of flight, or UWB) adds "range_m" and "range_sd" to the rx line of a
 * robot's frame. Without it the ONA falls back on the RSSI (much rougher).
 * ==========================================================================*/

#include <bp_arduino.h>

#ifndef GW_DIGEST_MS
#define GW_DIGEST_MS 90000UL
#endif
#ifndef GW_HB_MS
#define GW_HB_MS 5000UL
#endif

static bpa::App app;
static uint32_t next_hb = 0;
static bool raw_lines = true;

static const char *ev_name(int ev) {
    switch (ev) {
        case BP_EV_NEW: return "new";
        case BP_EV_UPDATE: return "update";
        case BP_EV_SILENT: return "silent";
        case BP_EV_REVIVED: return "revived";
        case BP_EV_EXPIRED: return "expired";
        default: return nullptr;
    }
}

static void on_event(bpa::App &a, int ev, const bp_entry_t *e, int info) {
    if (ev == BP_EV_REJECT) {
        Serial.printf("{\"ev\":\"reject\",\"reason\":\"%s\",\"rx_node\":%u,\"ts\":%lu}\n",
                      info == -10 ? "future_timestamp" : bp_status_str(info), (unsigned)a.p.id,
                      (unsigned long)bp_node_unix(&a.node));
        return;
    }
    const char *name = ev_name(ev);
    if (name && e) a.print_entry(e, name);
}

/* v9: every 44-byte LMB2 frame heard goes to the ONA as it was received */
static void on_raw_rx(bpa::App &a, const uint8_t *buf, int len, int rssi, float snr) {
    static const char hx[] = "0123456789abcdef";
    char frame[2 * BP_FRAME_LEN + 1];
    if (!raw_lines || len != BP_FRAME_LEN || (buf[0] >> 4) != BP_VERSION) return;
    for (int i = 0; i < len; ++i) {
        frame[2 * i] = hx[buf[i] >> 4];
        frame[2 * i + 1] = hx[buf[i] & 15];
    }
    frame[2 * len] = 0;
    Serial.printf("{\"ev\":\"rx\",\"gw\":\"%s\",\"frame\":\"%s\",\"rssi\":%d,\"snr\":%.1f,\"ts\":%lu}\n",
                  a.p.name, frame, rssi, (double)snr, (unsigned long)bp_node_unix(&a.node));
}

static int hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static bool on_command(bpa::App &a, const String &cmd, const String &arg) {
    if (cmd == "TX") {                       /* the ONA's briefing for the Executor */
        uint8_t buf[BP_FRAME_LEN];
        if ((int)arg.length() != 2 * BP_FRAME_LEN) { Serial.println("ERR TX needs 88 hex chars (44 bytes)"); return true; }
        for (int i = 0; i < BP_FRAME_LEN; ++i) {
            int h = hexval(arg[2 * i]), l = hexval(arg[2 * i + 1]);
            if (h < 0 || l < 0) { Serial.println("ERR TX bad hex"); return true; }
            buf[i] = (uint8_t)(h << 4 | l);
        }
        if ((buf[0] >> 4) != BP_VERSION) { Serial.println("ERR TX not an LMB2 frame"); return true; }
        int r = a.transmit_raw(buf, BP_FRAME_LEN);
        if (r == -2) Serial.println("ERR TX duty-cycle budget exhausted, retry later");
        else if (r != 0) Serial.println("ERR TX radio");
        else Serial.printf("OK TX type 0x%02x\n", (unsigned)buf[0]);
        return true;
    }
    if (cmd == "RAW") {
        raw_lines = arg.toInt() != 0;
        Serial.printf("OK RAW %d\n", raw_lines ? 1 : 0);
        return true;
    }
    return false;
}

void setup() {
    Serial.begin(115200);
    delay(200);
    app.on_event = on_event;
    app.on_raw_rx = on_raw_rx;
    app.on_command = on_command;
    app.begin(/*default_id=*/900, /*relay=*/false, /*digest_period_ms=*/GW_DIGEST_MS);
    Serial.println("{\"ev\":\"boot\",\"role\":\"gateway\"}");
}

void loop() {
    app.loop();
    if ((int32_t)(millis() - next_hb) >= 0) {
        next_hb = millis() + GW_HB_MS;
        const bp_stats_t &s = app.node.st;
        Serial.printf("{\"ev\":\"hb\",\"gw\":\"%s\",\"id\":%u,\"ts\":%lu,\"clock\":%s,\"key\":%s,\"radio\":%s,"
                      "\"rx_ok\":%lu,\"rx_bad\":%lu,\"known\":%d,\"digests\":%lu,\"duty\":%.4f}\n",
                      app.p.name, (unsigned)app.p.id, (unsigned long)bp_node_unix(&app.node),
                      app.node.clock_set ? "true" : "false", app.p.key_set ? "true" : "false",
                      app.radio_ok ? "true" : "false", (unsigned long)s.rx_ok,
                      (unsigned long)(s.rx_bad_mac + s.rx_bad_other), bp_node_count(&app.node),
                      (unsigned long)s.tx_digest, (double)bp_node_duty_used(&app.node));
    }
}
