/*
 * beacon_proto.h - Living Map beacon protocol v2 ("LMB2")
 * ==========================================================================
 * One 44-byte LoRa frame per beacon record. Authenticated with a truncated
 * HMAC-SHA256 under a per-mission network key. Shared, byte-for-byte, by:
 *   - the beacon firmware (ESP32 + SX1276, 868 MHz)
 *   - the gateway / Executor receiver firmware
 *   - the host tools and network simulator
 *   - the Python mirror implementation (python/beaconnet/proto.py)
 *
 * FRAME LAYOUT (little-endian)                               covered by MAC
 * ---------------------------------------------------------------------------
 *  off size field            meaning
 *   0   1   ver_type         0x21 = version 2 (hi nibble), type 1 RECORD  yes
 *   1   1   net_id           mission / team network id                   yes
 *   2   2   relay_id         node that transmitted THIS copy              no
 *   4   1   hops             hi nibble = hops so far, lo = hop limit      no
 *   5   1   hdr_flags        bit0 ORIGIN_SILENT, bit1 ORIGIN_ALIVE        no
 *  ---- signed record (30 bytes) ---------------------------------------
 *   6   2   origin_id        beacon that owns this record                yes
 *   8   2   seq              per-origin version counter (wraps)          yes
 *  10   1   kind             bp_kind_t (VICTIM, GAS, ...)                yes
 *  11   1   flags            BP_RF_* (has next, gnss fix, retracted...)  yes
 *  12   4   lat_e7           latitude  x 1e7 (int32)                     yes
 *  16   4   lon_e7           longitude x 1e7 (int32)                     yes
 *  20   1   err_dm           position error, 0.1 m (255 = unknown)       yes
 *  21   1   severity         0..255 (0..100 recommended)                 yes
 *  22   1   confidence       0..255  -> 0.0..1.0 at timestamp            yes
 *  23   1   next_bearing     direction to next hop, 360/256 deg steps    yes
 *  24   2   next_id          next beacon on the route (0xFFFF = none)    yes
 *  26   2   next_dist_dm     distance to next hop, 0.1 m                 yes
 *  28   4   timestamp        unix seconds when the record was made       yes
 *  32   2   ttl_10s          hard lifetime, 10 s units (max 7.5 days)    yes
 *  34   2   half_life_10s    confidence half-life, 10 s units            yes
 *  ---- authentication -------------------------------------------------
 *  36   8   mac              HMAC-SHA256(key, "LMB2"|b[0..1]|b[6..35])[:8]
 * ---------------------------------------------------------------------------
 * Total 44 bytes.  relay_id / hops / hdr_flags change at every relay, so they
 * are deliberately outside the MAC; the record itself is end-to-end sealed.
 */
#ifndef BEACON_PROTO_H
#define BEACON_PROTO_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BP_VERSION        2
#define BP_TYPE_RECORD    1
#define BP_TYPE_DIGEST    2
#define BP_VER_TYPE       ((BP_VERSION << 4) | BP_TYPE_RECORD)   /* 0x21 */
#define BP_VER_DIGEST     ((BP_VERSION << 4) | BP_TYPE_DIGEST)   /* 0x22 */
/* v9: frames for the Outside Network Area, same 44-byte size, header and MAC
 * rule as a RECORD (see living_map/ona/ona/lmb2.py for the layouts):
 *   0x23 ROBOT    a robot's position report, heard by the ONA's gateways
 *   0x24 MISSION  a briefing from the Command Post, sent by the gateways to the Executor
 * The gossip engine ignores both (no relay, no reject event); a gateway passes
 * every frame it hears to the ONA as one "rx" JSON line. */
#define BP_TYPE_ROBOT     3
#define BP_TYPE_MISSION   4
#define BP_VER_ROBOT      ((BP_VERSION << 4) | BP_TYPE_ROBOT)    /* 0x23 */
#define BP_VER_MISSION    ((BP_VERSION << 4) | BP_TYPE_MISSION)  /* 0x24 */

/*
 * DIGEST FRAME (pull side of push-pull gossip; sent by gateways / receivers)
 *   0  1  ver_type 0x22        4  1  count (entries)
 *   1  1  net_id               5  1  page
 *   2  2  sender_id            6  4n entries: origin_id u16, seq u16
 *   6+4n 8  mac = HMAC-SHA256(key, "LMB2" | all preceding bytes)[:8]
 * "Here is every (origin, seq) I hold."  Neighbours that hold something missing
 * or newer push it (BP_TX_SYNC).  Max 60 entries per page (255-byte LoRa limit).
 * Entries are sorted by origin. page = index (bits 0-6) | FINAL (bit 7); page
 * k>0 repeats the last origin of page k-1, so the pages' origin ranges touch
 * and "not listed within [first, last]" reliably means "missing".
 */
#define BP_DIGEST_MAX_ENTRIES 60
#define BP_DIGEST_LEN(n)      (6 + 4 * (n) + BP_MAC_LEN)
#define BP_DIGEST_PAGE_MASK   0x7F
#define BP_DIGEST_FINAL       0x80
#define BP_HDR_LEN        6
#define BP_REC_LEN        30
#define BP_MAC_LEN        8
#define BP_FRAME_LEN      (BP_HDR_LEN + BP_REC_LEN + BP_MAC_LEN)  /* 44 */
#define BP_KEY_LEN        16
#define BP_ID_NONE        0xFFFF
#define BP_ERR_UNKNOWN    255
#define BP_MAX_HOP_LIMIT  15
#define BP_HOPS_UNSCOPED  15     /* hop_limit 15 = no scope, counter saturates */

/* ---- record flags (signed) ---- */
#define BP_RF_HAS_NEXT    0x01   /* next_id / next_dist are valid            */
#define BP_RF_HAS_BEARING 0x02   /* next_bearing is valid                    */
#define BP_RF_POS_GNSS    0x04   /* position from a GNSS fix (else SLAM)     */
#define BP_RF_RETRACTED   0x08   /* hazard cleared / record withdrawn        */
#define BP_RF_STALE       0x10   /* event aged out; record kept alive by its
                                    beacon as a ROUTE MARKER only (position +
                                    next hop stay valid, kind kept for history) */

/* ---- header flags (unsigned, set by the relaying node) ---- */
#define BP_HF_ORIGIN_SILENT 0x01 /* relay (or its sources) lost the origin     */
#define BP_HF_ORIGIN_ALIVE  0x02 /* relay hears the origin itself, good link   */

typedef enum {
    BP_KIND_WAYPOINT = 0,   /* trail breadcrumb                            */
    BP_KIND_VICTIM = 1,
    BP_KIND_GAS = 2,
    BP_KIND_RADIATION = 3,
    BP_KIND_THERMAL = 4,    /* fire / heat source                          */
    BP_KIND_OBSTRUCTION = 5,
    BP_KIND_STRUCTURAL = 6, /* collapse risk                               */
    BP_KIND_EXIT = 7,       /* safe exit / staging area                    */
    /* resources (mine scenario): marked, never avoided; low radio priority  */
    BP_KIND_PHOSPHATE = 8,  /* phosphate seam; severity = grade, % P2O5 x 4 */
    BP_KIND_GOLD = 9,       /* gold / precious-metal find                  */
    BP_KIND_GEMSTONE = 10,  /* precious-stone find                         */
    /* victim search: an area searched with a thermal camera, like the "X"
     * search marking rescue teams paint at a door. severity = % of the area
     * covered; any victim found there has its own VICTIM record             */
    BP_KIND_SEARCHED = 11,
    BP_KIND_COUNT
} bp_kind_t;

typedef enum {
    BP_OK = 0,
    BP_ERR_LEN = -1,
    BP_ERR_VERSION = -2,
    BP_ERR_NET = -3,
    BP_ERR_MAC = -4,
    BP_ERR_FIELD = -5
} bp_status_t;

typedef struct {
    uint16_t origin_id;
    uint16_t seq;
    uint8_t  kind;
    uint8_t  flags;
    int32_t  lat_e7;
    int32_t  lon_e7;
    uint8_t  err_dm;
    uint8_t  severity;
    uint8_t  confidence;
    uint8_t  next_bearing;
    uint16_t next_id;
    uint16_t next_dist_dm;
    uint32_t timestamp;
    uint16_t ttl_10s;
    uint16_t half_life_10s;
} bp_record_t;

typedef struct {
    uint8_t  net_id;
    uint16_t relay_id;
    uint8_t  hops;       /* 0..15 */
    uint8_t  hop_limit;  /* 0..15 */
    uint8_t  hdr_flags;
    bp_record_t rec;
} bp_frame_t;

/* ---- codec ---- */
int  bp_encode(const bp_frame_t *f, const uint8_t key[BP_KEY_LEN], uint8_t out[BP_FRAME_LEN]);
int  bp_decode(const uint8_t *buf, size_t len, const uint8_t key[BP_KEY_LEN],
               uint8_t expect_net, bp_frame_t *out);
int  bp_validate_record(const bp_record_t *r);          /* BP_OK or BP_ERR_FIELD */

/* digest codec: returns frame length (>0) or a negative bp_status_t */
int  bp_encode_digest(uint8_t net_id, uint16_t sender, const uint16_t *origins,
                      const uint16_t *seqs, uint8_t count, uint8_t page,
                      const uint8_t key[BP_KEY_LEN], uint8_t *out, size_t out_max);
int  bp_decode_digest(const uint8_t *buf, size_t len, const uint8_t key[BP_KEY_LEN],
                      uint8_t expect_net, uint16_t *sender, uint16_t *origins,
                      uint16_t *seqs, uint8_t *count, uint8_t max_entries);
const char *bp_status_str(int status);

/* ---- kinds / adaptive aging ---- */
const char *bp_kind_name(uint8_t kind);                  /* "VICTIM", ...       */
int      bp_kind_from_name(const char *name);            /* -1 if unknown       */
uint32_t bp_default_half_life_s(uint8_t kind);
uint32_t bp_default_ttl_s(uint8_t kind);
float    bp_kind_weight(uint8_t kind);                   /* gossip priority     */

/* effective confidence (0..1) at unix time `now`: c0 * 2^(-age/half_life) */
float bp_effective_confidence(const bp_record_t *r, uint32_t now);
/* 1 if the record must be forgotten: past TTL or confidence under min_conf */
int   bp_is_expired(const bp_record_t *r, uint32_t now, float min_conf);

/* ---- unit helpers ---- */
int32_t  bp_deg_to_e7(double deg);
double   bp_e7_to_deg(int32_t e7);
uint8_t  bp_bearing_to_u8(double deg);
double   bp_u8_to_bearing(uint8_t b);
uint16_t bp_seconds_to_10s(uint32_t s);                  /* rounds, clamps     */

/* ---- LoRa time-on-air (Semtech AN1200.13), milliseconds, rounded up ---- */
uint32_t bp_lora_airtime_ms(uint8_t payload_len, uint8_t sf, uint32_t bw_hz,
                            uint8_t cr_denom /*5..8*/, uint8_t preamble,
                            int crc_on, int implicit_header);

/* ---- JSON (same shape as the mission-log example) ----
 * {"beacon_id":7,"seq":1,"kind":"VICTIM","gps":{"lat":..,"lon":..,"err_m":2.1,"src":"slam"},
 *  "severity":15,"confidence":1.00,"next":{"id":6,"dist_m":8.5,"bearing_deg":212.3},
 *  "ts":1758500000,"ttl_s":7200,"half_life_s":3600,"retracted":false,"stale":false}
 * Returns chars written (excluding NUL) or -1 if the buffer is too small. */
int bp_record_to_json(const bp_record_t *r, char *buf, size_t n);

#ifdef __cplusplus
}
#endif
#endif /* BEACON_PROTO_H */
