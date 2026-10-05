/*
 * bp_gossip.h - epidemic replication engine for Living Map beacons.
 * ==========================================================================
 * Portable C99, no globals, no malloc: one bp_node_t per radio node. The
 * firmware (ESP32), the gateway and the host simulator all run THIS code;
 * only the tiny HAL (clock, radio TX, random, event sink) differs.
 *
 * Every node keeps a cache of the records it has heard. Three mechanisms
 * keep that cache replicated across neighbouring beacons:
 *
 *  1. OWN ANNOUNCE     each gossip round a beacon re-broadcasts its own
 *                      record (hops = 0).
 *  2. REACTIVE RELAY   a NEW or UPDATED record (never-seen seq) is re-sent
 *                      after a random back-off, hop count +1, up to the hop
 *                      limit. Trickle-style suppression: if k copies of the
 *                      same (origin, seq) are overheard during the back-off,
 *                      the relay is cancelled -> no broadcast storm.
 *  3. ANTI-ENTROPY     each round, one cached record chosen by
 *                      priority x staleness is re-announced. Records of a
 *                      destroyed beacon therefore live on in its neighbours
 *                      and keep circulating until their TTL / half-life ends.
 *  4. PULL (DIGEST)    a gateway / receiver periodically broadcasts a signed
 *                      digest of the (origin, seq) pairs it holds; beacons in
 *                      range push exactly what it is missing (BP_TX_SYNC).
 *                      Gateway convergence no longer depends on how many
 *                      records compete for anti-entropy slots.
 *
 * Liveness: a node with a STRONG first-hand link to origin X (smoothed RSSI
 * >= sensitivity + 12 dB and X heard in >= 75 % of recent rounds) that then hears
 * nothing at all from X for BP_SILENT_ROUNDS (5) rounds marks X "silent" and
 * sets ORIGIN_SILENT when it gossips X's record; other nodes adopt that as
 * hearsay unless they have a strong link to X themselves. Such a node sets
 * ORIGIN_ALIVE instead, which clears stale hearsay. Anything X transmits (its
 * own record, or a relay of someone else's) counts as proof of life. Weak
 * links never judge -> no false "beacon lost" alarms from fading.
 *
 * Regulatory: every TX is paid for from a duty-cycle token bucket
 * (1 % in the EU 868.0-868.6 MHz sub-band). Priority when the budget is
 * short: reactive relays > sync pushes > own record > anti-entropy.
 */
#ifndef BP_GOSSIP_H
#define BP_GOSSIP_H

#include "beacon_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef BP_CACHE_MAX
#define BP_CACHE_MAX 48
#endif
#define BP_SILENT_ROUNDS 5   /* empty rounds before a strong neighbour is "silent" */
#ifndef BP_JUDGE_MARGIN_DB
#define BP_JUDGE_MARGIN_DB 12 /* link margin above sensitivity to judge liveness */
#endif
#ifndef BP_ACK_RETRIES
#define BP_ACK_RETRIES 2     /* passive-ack retransmissions per record version */
#endif

enum {
    BP_EV_NEW = 1,      /* first time this origin is heard               */
    BP_EV_UPDATE = 2,   /* newer seq for a known origin                  */
    BP_EV_SILENT = 3,   /* first-hand neighbour stopped transmitting     */
    BP_EV_REVIVED = 4,  /* silent neighbour heard again                  */
    BP_EV_EXPIRED = 5,  /* record aged out (TTL or half-life)            */
    BP_EV_REJECT = 6,   /* frame failed auth/validation (info = status)  */
    BP_EV_TX = 7,       /* we transmitted (info = BP_TX_*)               */
    BP_EV_OWN_RENEWED = 8 /* own event aged out -> record re-issued as a
                             STALE route marker (firmware persists it)   */
};
/* transmission classes, in priority order when airtime is short */
enum { BP_TX_OWN = 1, BP_TX_RELAY = 2, BP_TX_AE = 3, BP_TX_SYNC = 4, BP_TX_DIGEST = 5 };

typedef struct {
    bp_record_t rec;
    uint8_t  used;
    uint8_t  is_own;
    uint8_t  hops;           /* hops of the best copy we received (0 = own) */
    uint8_t  hop_limit;
    uint8_t  relay_pending;
    uint8_t  relay_kind;     /* BP_TX_* of the pending transmission         */
    uint8_t  dup_heard;      /* copies overheard during the back-off        */
    uint8_t  heard_direct;   /* ever heard first-hand from the origin       */
    uint8_t  silent;             /* WE heard it first-hand, then lost it    */
    uint8_t  origin_silent_flag; /* ORIGIN_SILENT seen on received copies   */
    uint8_t  reported_silent;    /* what we tell others / the gateway       */
    uint16_t last_relay_id;
    int16_t  rssi;           /* of the last first-hand reception            */
    int16_t  snr_x10;
    uint16_t tx_count;       /* transmissions of the current version        */
    uint8_t  ack_wait;       /* relayed, waiting to overhear a copy ahead   */
    uint8_t  retries;        /* passive-ack retries spent on this version   */
    uint32_t first_seen_ms, last_heard_ms, last_direct_ms, last_sent_ms, relay_at_ms;
    uint32_t ack_deadline_ms;
    uint16_t any_rounds;     /* anything from X heard first-hand, 1 bit/round */
    uint8_t  direct_n;       /* first-hand receptions (saturating)            */
    int16_t  link_rssi;      /* smoothed RSSI of X's transmissions, dBm       */
    uint8_t  known_rounds;   /* rounds since first first-hand contact         */
} bp_entry_t;

typedef struct {
    uint16_t my_id;
    uint8_t  net_id;
    uint8_t  key[BP_KEY_LEN];
    uint8_t  hop_limit;         /* global cap on how far a record floods        */
    uint8_t  relay_enabled;     /* 0 for gateways / passive listeners           */
    uint8_t  suppress_k;        /* cancel a relay after k duplicates            */
    uint32_t round_ms;          /* gossip round period (jittered +-20 %)        */
    uint32_t relay_imin_ms;     /* reactive back-off window [imin, 2*imin)      */
    uint32_t frame_airtime_ms;  /* time-on-air of one frame (0 = from sf/bw/cr) */
    uint8_t  lora_sf;           /* radio settings, used for airtime maths       */
    uint32_t lora_bw_hz;
    uint8_t  lora_cr_denom;
    uint32_t digest_period_ms;  /* 0 = never (beacons); gateways: e.g. 90000    */
    uint8_t  sync_max;          /* records pushed in answer to one digest       */
    uint32_t duty_cap_ms;       /* max accumulated airtime credit               */
    float    duty_cycle;        /* 0.01 = 1 %                                   */
    float    min_conf;          /* forget records below this confidence         */
    uint32_t max_future_s;      /* reject records timestamped this far ahead    */
} bp_config_t;

struct bp_node;
typedef struct {
    uint32_t (*now_ms)(void *ctx);
    int      (*radio_tx)(void *ctx, const uint8_t *buf, uint8_t len); /* 0 = ok */
    uint32_t (*rand32)(void *ctx);
    void     (*on_event)(void *ctx, int ev, const struct bp_node *n,
                         const bp_entry_t *e, int info);               /* optional */
    void *ctx;
} bp_hal_t;

typedef struct {
    uint32_t tx_own, tx_relay, tx_ae, tx_sync, tx_digest, tx_airtime_ms;
    uint32_t rx_digest;
    uint32_t rx_ok, rx_new, rx_update, rx_dup, rx_stale, rx_echo;
    uint32_t rx_bad_mac, rx_bad_other, rx_future, rx_conflict;
    uint32_t rx_foreign;             /* v9: ROBOT / MISSION frames (not for gossip), ignored */
    uint32_t suppressed, deferred_budget, evicted, expired, ack_retries;
} bp_stats_t;

typedef struct bp_node {
    bp_config_t cfg;
    bp_hal_t    hal;
    bp_entry_t  cache[BP_CACHE_MAX];
    bp_stats_t  st;
    int64_t     unix_offset_ms;  /* unix_ms = now_ms + offset                   */
    uint8_t     clock_set;
    uint32_t    boot_ms;
    uint32_t    next_round_ms;
    uint32_t    next_digest_ms;
    uint32_t    last_digest_ms;
    uint8_t     digest_page;
    uint32_t    busy_until_ms;   /* radio busy transmitting                     */
    float       budget_ms;       /* duty-cycle credit                           */
    uint32_t    budget_t_ms;
} bp_node_t;

void bp_config_defaults(bp_config_t *c);
void bp_node_init(bp_node_t *n, const bp_config_t *cfg, const bp_hal_t *hal);

/* clock: unix seconds, set by the Writer at provisioning or by the host */
void     bp_node_set_time(bp_node_t *n, uint32_t unix_s);
uint32_t bp_node_unix(const bp_node_t *n);

/* provisioning: install/replace this beacon's own record (origin forced to
 * my_id). If the record changed, seq should be bumped by the caller.       */
int  bp_node_set_own(bp_node_t *n, const bp_record_t *rec);
const bp_entry_t *bp_node_own(const bp_node_t *n);

/* provisioning: prime the cache with a record the Writer already knows (the
 * route so far). A fresh beacon then knows the route tree from its first
 * second, which the relay-suppression logic relies on, instead of learning
 * it record by record from anti-entropy. Not re-broadcast (neighbours have
 * it); newer versions heard on air replace it as usual. BP_OK or error.   */
int  bp_node_seed(bp_node_t *n, const bp_record_t *rec);

void bp_node_on_rx(bp_node_t *n, const uint8_t *buf, uint8_t len, int16_t rssi, int16_t snr_x10);
void bp_node_tick(bp_node_t *n);        /* call every few ms */

/* introspection */
const bp_entry_t *bp_node_find(const bp_node_t *n, uint16_t origin);
int   bp_node_count(const bp_node_t *n);
float bp_entry_priority(const bp_node_t *n, const bp_entry_t *e);
float bp_node_duty_used(const bp_node_t *n);  /* fraction of airtime since boot */

/* JSON for gateways: record + reception metadata (age, eff_conf, hops, ...) */
int bp_entry_to_json(const bp_node_t *n, const bp_entry_t *e, const char *event,
                     char *buf, size_t len);

#ifdef __cplusplus
}
#endif
#endif /* BP_GOSSIP_H */
