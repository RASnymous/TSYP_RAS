/* ============================================================================
 * Living Map BEACON firmware - TSYP14 IEEE RAS x AESS
 * ESP32 + SX1276/RFM95, 868.1 MHz LoRa (SF9 / 125 kHz / 4:5), protocol LMB2
 * ============================================================================
 * One of these is dropped by the Writer at every event and along the trail.
 * The beacon:
 *   - holds ONE record of its own (44-byte frame: id, kind, GPS position +
 *     error, severity, confidence, next-hop vector, timestamp, TTL, half-life),
 *     sealed with a truncated HMAC-SHA256 under the mission key,
 *   - announces it every gossip round (~96 s at SF9, sized to 60 % of the
 *     1 % duty budget), and re-issues it as a STALE route marker when the
 *     event ages out, so the evacuation route stays intact,
 *   - stores and re-broadcasts its neighbours' records (gossip): reactive
 *     relay of new versions with directional suppression + passive acks,
 *     anti-entropy of the most valuable cached record, targeted sync pushes
 *     when a gateway's digest shows something missing. If this beacon is
 *     crushed, its record survives in its neighbours and keeps flowing,
 *   - flags a neighbour that went silent (ORIGIN_SILENT) -> lost beacon alert.
 *
 * Arduino IDE: install "LoRa" by Sandeep Mistry, copy lib/BeaconProto to
 * ~/Arduino/libraries/BeaconProto, board "ESP32 Dev Module" (or TTGO LoRa32).
 * PlatformIO: `pio run -e beacon -t upload` in this folder.
 *
 * Provisioning (the Writer's MCU, or you, over USB serial at 115200):
 *   KEY <32 hex>                 mission key (once per mission, all nodes)
 *   TIME <unix>                  clock (the beacon has no RTC)
 *   PROV {json}                  this beacon's record; beacon_id becomes the
 *                                node id; seq is bumped past anything this
 *                                beacon announced before; persisted in NVS
 *   SEED {json}                  a record the Writer already knows (the route
 *                                so far) -> known route tree from second one
 *   RETRACT                      withdraw the event (hazard cleared)
 *   STATUS | DUMP | VERBOSE 0/1 | HELP
 * Example:
 *   PROV {"beacon_id":7,"kind":"VICTIM","gps":{"lat":34.4312,"lon":8.7845,"err_m":2.1},"severity":15,"next":{"id":6,"dist_m":8.5},"ttl_s":7200}
 * ==========================================================================*/

// #define BP_PIN_LED 25          // uncomment / adapt for your board (see bp_arduino.h)
#include <bp_arduino.h>

static bpa::App app;
static bool verbose = true;

static const char *ev_name(int ev) {
    switch (ev) {
        case BP_EV_NEW: return "new";
        case BP_EV_UPDATE: return "update";
        case BP_EV_SILENT: return "silent";
        case BP_EV_REVIVED: return "revived";
        case BP_EV_EXPIRED: return "expired";
        case BP_EV_OWN_RENEWED: return "own_renewed";
        default: return "event";
    }
}

static void on_event(bpa::App &a, int ev, const bp_entry_t *e, int info) {
    if (ev == BP_EV_OWN_RENEWED && e) {          /* keep NVS in step with the air */
        a.p.own = e->rec;
        a.save();
    }
    if (!verbose) return;
    if (ev == BP_EV_REJECT) {
        Serial.printf("{\"ev\":\"reject\",\"reason\":\"%s\"}\n",
                      info == -10 ? "future_timestamp" : bp_status_str(info));
    } else if (ev == BP_EV_TX) {
        static const char *k[] = {"?", "own", "relay", "ae", "sync", "digest"};
        Serial.printf("# tx %s origin %u\n", (info >= 1 && info <= 5) ? k[info] : "?",
                      e ? (unsigned)e->rec.origin_id : 0u);
    } else if (e) {
        a.print_entry(e, ev_name(ev));
    }
}

static bool provision_own(bpa::App &a, const String &arg) {
    bp_record_t r;
    uint32_t now = a.node.clock_set ? bp_node_unix(&a.node) : 0;
    if (bp_record_from_json(arg.c_str(), &r, now) != BP_OK) {
        Serial.println("ERR PROV invalid record JSON (need beacon_id, gps.lat, gps.lon)");
        return true;
    }
    if (!a.node.clock_set) {
        if (!r.timestamp) { Serial.println("ERR PROV send TIME first, or include \"ts\""); return true; }
        a.set_clock(r.timestamp);                  /* the Writer's timestamp is our clock */
    }
    if (r.origin_id != a.p.id) {                   /* the Writer assigns identities */
        a.p.id = r.origin_id;
        a.p.has_own = 0;
        a.restart_engine();
    }
    if (a.p.has_own) {                             /* never re-use a version number */
        uint16_t min_seq = (uint16_t)(a.p.own.seq + 1);
        if ((int16_t)(uint16_t)(r.seq - min_seq) < 0) r.seq = min_seq;
    }
    if (bp_node_set_own(&a.node, &r) != BP_OK) { Serial.println("ERR PROV rejected"); return true; }
    a.p.own = r;
    a.p.has_own = 1;
    a.save();
    char json[512];
    bp_record_to_json(&r, json, sizeof json);
    Serial.printf("OK PROV id=%u seq=%u %s\n", (unsigned)r.origin_id, (unsigned)r.seq, json);
    return true;
}

static bool on_command(bpa::App &a, const String &cmd, const String &arg) {
    if (cmd == "PROV") return provision_own(a, arg);
    if (cmd == "SEED") {
        bp_record_t r;
        uint32_t now = a.node.clock_set ? bp_node_unix(&a.node) : 0;
        if (bp_record_from_json(arg.c_str(), &r, now) != BP_OK || bp_node_seed(&a.node, &r) != BP_OK)
            Serial.println("ERR SEED");
        else
            Serial.printf("OK SEED %u\n", (unsigned)r.origin_id);
        return true;
    }
    if (cmd == "RETRACT") {
        if (!a.p.has_own) { Serial.println("ERR RETRACT no record"); return true; }
        a.p.own.flags |= BP_RF_RETRACTED;
        a.p.own.seq = (uint16_t)(a.p.own.seq + 1);
        if (a.node.clock_set) a.p.own.timestamp = bp_node_unix(&a.node);
        bp_node_set_own(&a.node, &a.p.own);
        a.save();
        Serial.printf("OK RETRACT seq=%u\n", (unsigned)a.p.own.seq);
        return true;
    }
    if (cmd == "VERBOSE") {
        verbose = arg.toInt() != 0;
        Serial.printf("OK VERBOSE %d\n", verbose ? 1 : 0);
        return true;
    }
    if (cmd == "HELP") {
        Serial.println("# beacon: PROV {json}  SEED {json}  RETRACT  VERBOSE 0|1");
        return false;                              /* let the common HELP print too */
    }
    return false;
}

void setup() {
    Serial.begin(115200);
    delay(200);
    setCpuFrequencyMhz(80);                        /* ~30 mA instead of ~70 mA; SPI unaffected */
    app.on_event = on_event;
    app.on_command = on_command;
    app.begin(/*default_id=*/1, /*relay=*/true, /*digest_period_ms=*/0);
    if (app.p.has_own) Serial.printf("# own record restored: id %u seq %u (%s)\n",
                                     (unsigned)app.p.own.origin_id, (unsigned)app.p.own.seq,
                                     bp_kind_name(app.p.own.kind));
}

void loop() {
    app.loop();
}
