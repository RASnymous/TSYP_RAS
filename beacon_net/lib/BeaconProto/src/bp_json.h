/*
 * bp_json.h - parse a beacon record from the mission-log JSON shape.
 * ==========================================================================
 * The Writer provisions each beacon over serial with one line of JSON, the
 * same shape the gateway prints and the Command Post shows:
 *
 *   {"beacon_id":7,"kind":"VICTIM","gps":{"lat":34.4312,"lon":8.7845,"err_m":2.1},
 *    "severity":15,"next":{"id":6,"dist_m":8.5},"ttl_s":7200}
 *
 * Required: beacon_id, gps.lat, gps.lon.  Everything else has a default:
 *   seq 1 | kind WAYPOINT | err_m unknown | severity 0 | confidence 1.0 |
 *   next none | ts = now_unix | ttl_s / half_life_s = per-kind defaults |
 *   gps.src "slam" | retracted/stale false.
 * next.bearing_deg is optional (sets HAS_BEARING).  Unknown keys are ignored,
 * so a gateway JSON line (with ev, rx_node, hops, ...) parses back too.
 *
 * Tiny, allocation-free, no recursion deeper than the JSON nesting (max 8):
 * runs on the ESP32 and in the host tools (tested against the Python mirror).
 */
#ifndef BP_JSON_H
#define BP_JSON_H

#include "beacon_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* BP_OK, or BP_ERR_FIELD (malformed JSON / missing or out-of-range field) */
int bp_record_from_json(const char *js, bp_record_t *out, uint32_t now_unix);

/* 32 hex chars -> 16-byte key. BP_OK or BP_ERR_FIELD. */
int bp_key_from_hex(const char *hex, uint8_t key[BP_KEY_LEN]);

#ifdef __cplusplus
}
#endif
#endif /* BP_JSON_H */
