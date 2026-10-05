/* bp_gossip.c - epidemic replication engine (see bp_gossip.h). */
#include "bp_gossip.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* signed difference of two wrapping millisecond clocks */
#define TDIFF(a, b) ((int32_t)((uint32_t)(a) - (uint32_t)(b)))

/* ------------------------------------------------------------ small helpers */
static uint32_t now_ms(const bp_node_t *n) { return n->hal.now_ms(n->hal.ctx); }

static uint32_t rnd(bp_node_t *n, uint32_t range) {
    return range ? n->hal.rand32(n->hal.ctx) % range : 0;
}

static void emit(bp_node_t *n, int ev, const bp_entry_t *e, int info) {
    if (n->hal.on_event) n->hal.on_event(n->hal.ctx, ev, n, e, info);
}

static bp_entry_t *find_mut(bp_node_t *n, uint16_t origin) {
    int i;
    for (i = 0; i < BP_CACHE_MAX; ++i)
        if (n->cache[i].used && n->cache[i].rec.origin_id == origin) return &n->cache[i];
    return NULL;
}

static int rec_equal(const bp_record_t *a, const bp_record_t *b) {
    return a->origin_id == b->origin_id && a->seq == b->seq && a->kind == b->kind &&
           a->flags == b->flags && a->lat_e7 == b->lat_e7 && a->lon_e7 == b->lon_e7 &&
           a->err_dm == b->err_dm && a->severity == b->severity && a->confidence == b->confidence &&
           a->next_bearing == b->next_bearing && a->next_id == b->next_id &&
           a->next_dist_dm == b->next_dist_dm && a->timestamp == b->timestamp &&
           a->ttl_10s == b->ttl_10s && a->half_life_10s == b->half_life_10s;
}

/* ---- liveness, first-hand -------------------------------------------------
 * We JUDGE the liveness of origin X only through a STRONG link: smoothed RSSI
 * >= receiver sensitivity + BP_JUDGE_MARGIN_DB AND X heard in >= 75 % of the
 * recent rounds. On such a link every frame X sends is received, so
 * BP_SILENT_ROUNDS (5) gossip rounds without a single frame from X (its own
 * announcement, or any relay it makes) means X is gone - not unlucky. Weak or
 * fading links never make us a judge: in simulation they raised false "beacon
 * lost" alarms (a burst of relays heard through a fade looks like a good
 * link; later a quiet X is simply not heard). Measuring by frame counts fails
 * too: under load X's own announcements are rightly deferred behind relays.
 * any_rounds: 1 bit per gossip round (bit0 = current), anything heard from X. */
static int16_t sensitivity_dbm(const bp_node_t *n) {
    static const int16_t s125[13] = {0, 0, 0, 0, 0, 0, -118, -123, -126, -129, -132, -134, -137};
    int16_t s = s125[n->cfg.lora_sf <= 12 ? n->cfg.lora_sf : 12];
    if (n->cfg.lora_bw_hz >= 500000u) s = (int16_t)(s + 6);
    else if (n->cfg.lora_bw_hz >= 250000u) s = (int16_t)(s + 3);
    return s;
}
static int popcount16(uint32_t v) {
    int c = 0;
    v &= 0xFFFFu;
    while (v) { c += (int)(v & 1u); v >>= 1; }
    return c;
}
/* strong over the (up to 8, at least 3) rounds of history that end `skip`
 * rounds ago: margin OK and X heard in >= 75 % of those rounds. (The RSSI of
 * received frames alone is biased on a marginal link - only the lucky ones
 * are received - hence the reception-rate condition as well.)              */
static int strong_history(const bp_node_t *n, const bp_entry_t *e, int skip) {
    int h = (int)e->known_rounds - skip;
    if (!e->heard_direct || e->direct_n < 3 || e->link_rssi < sensitivity_dbm(n) + BP_JUDGE_MARGIN_DB) return 0;
    if (h > 8) h = 8;
    if (h < 3) return 0;
    return 4 * popcount16(((uint32_t)e->any_rounds >> skip) & ((1u << h) - 1u)) >= 3 * h;
}
static int strong_link(const bp_node_t *n, const bp_entry_t *e) {
    return strong_history(n, e, 0) || strong_history(n, e, 1);   /* current round may be young */
}
/* we may speak first-hand about X: strong link, or we already judged it */
static int first_hand(const bp_node_t *n, const bp_entry_t *e) { return e->silent || strong_link(n, e); }
/* X transmitted within the last two rounds (reachable neighbour right now) */
static int recently_heard(const bp_entry_t *e) { return (e->any_rounds & 0x3u) != 0; }

static uint32_t frame_airtime(const bp_node_t *n, uint8_t len) {
    return bp_lora_airtime_ms(len, n->cfg.lora_sf, n->cfg.lora_bw_hz, n->cfg.lora_cr_denom, 8, 1, 0);
}

static int record_expired(const bp_node_t *n, const bp_record_t *r) {
    if (!n->clock_set) return 0;          /* no clock yet -> can't judge age */
    return bp_is_expired(r, bp_node_unix(n), n->cfg.min_conf);
}

/* ------------------------------------------------------------ config / init */
void bp_config_defaults(bp_config_t *c) {
    memset(c, 0, sizeof *c);
    c->my_id = 1;
    c->net_id = 0x2A;
    c->hop_limit = BP_HOPS_UNSCOPED; /* no scope limit: the flood is already bounded by
                                        once-per-version relays + (origin, seq) dedup */
    c->relay_enabled = 1;
    c->suppress_k = 1;              /* one copy from a node AHEAD of us is enough */
    c->round_ms = 0;                /* 0 = auto: background (own + AE) <= 60 % of duty */
    c->relay_imin_ms = 800;
    c->lora_sf = 9;                 /* SF9 / 125 kHz / 4:5: ~1.5 km LOS, ~60 m rubble */
    c->lora_bw_hz = 125000;
    c->lora_cr_denom = 5;
    c->frame_airtime_ms = 0;        /* 0 -> computed from sf/bw/cr at init (288 ms) */
    c->digest_period_ms = 0;        /* beacons don't pull; gateways set e.g. 90 s   */
    c->sync_max = 2;                /* frames pushed in answer to one digest        */
    c->duty_cap_ms = 3000;
    c->duty_cycle = 0.01f;
    c->min_conf = 0.05f;
    c->max_future_s = 300;
}

void bp_node_init(bp_node_t *n, const bp_config_t *cfg, const bp_hal_t *hal) {
    uint32_t t;
    memset(n, 0, sizeof *n);
    n->cfg = *cfg;
    n->hal = *hal;
    if (n->cfg.hop_limit > BP_MAX_HOP_LIMIT) n->cfg.hop_limit = BP_MAX_HOP_LIMIT;
    if (n->cfg.suppress_k == 0) n->cfg.suppress_k = 1;
    if (n->cfg.lora_sf < 6 || n->cfg.lora_sf > 12) n->cfg.lora_sf = 9;
    if (n->cfg.lora_bw_hz == 0) n->cfg.lora_bw_hz = 125000;
    if (n->cfg.lora_cr_denom < 5 || n->cfg.lora_cr_denom > 8) n->cfg.lora_cr_denom = 5;
    if (n->cfg.frame_airtime_ms == 0) n->cfg.frame_airtime_ms = frame_airtime(n, BP_FRAME_LEN);
    if (n->cfg.duty_cycle <= 0.0f || n->cfg.duty_cycle > 1.0f) n->cfg.duty_cycle = 0.01f;
    /* Background traffic (1 own + 1 anti-entropy frame per round) is sized to
     * 60 % of the duty budget, the other 40 % stays free for reactive relays
     * and sync pushes: round = 2 frames / (0.6 x duty).
     * SF7 ~31 s, SF9 ~96 s, SF10 ~ 3 min, SF12 ~12 min.                    */
    if (n->cfg.round_ms == 0) {
        float r = 2.0f * (float)n->cfg.frame_airtime_ms / (0.6f * n->cfg.duty_cycle);
        n->cfg.round_ms = r < 20000.0f ? 20000u : (uint32_t)r;
    }
    /* the credit bucket must hold a few frames (priority reserve below) and,
     * on a digest sender, one full-size digest                               */
    {
        uint32_t need = 4u * n->cfg.frame_airtime_ms;
        if (n->cfg.digest_period_ms) {
            uint32_t d = frame_airtime(n, BP_DIGEST_LEN(BP_CACHE_MAX < BP_DIGEST_MAX_ENTRIES ?
                                                        BP_CACHE_MAX : BP_DIGEST_MAX_ENTRIES)) +
                         n->cfg.frame_airtime_ms;
            if (d > need) need = d;
        }
        if (n->cfg.duty_cap_ms < need) n->cfg.duty_cap_ms = need;
    }
    t = now_ms(n);
    n->boot_ms = t;
    n->budget_t_ms = t;
    n->busy_until_ms = t;
    n->budget_ms = (float)n->cfg.duty_cap_ms;              /* start full: still provably
                                                               within duty (see tick)      */
    n->next_round_ms = t + rnd(n, n->cfg.round_ms);          /* desynchronise rounds         */
    n->next_digest_ms = t + n->cfg.digest_period_ms / 4 + rnd(n, n->cfg.digest_period_ms / 2 + 1);
}

void bp_node_set_time(bp_node_t *n, uint32_t unix_s) {
    n->unix_offset_ms = (int64_t)unix_s * 1000 - (int64_t)now_ms(n);
    n->clock_set = 1;
}

uint32_t bp_node_unix(const bp_node_t *n) {
    int64_t u = (int64_t)now_ms(n) + n->unix_offset_ms;
    return u > 0 ? (uint32_t)(u / 1000) : 0;
}

/* ------------------------------------------------------------ priority */
float bp_entry_priority(const bp_node_t *n, const bp_entry_t *e) {
    float conf = n->clock_set ? bp_effective_confidence(&e->rec, bp_node_unix(n))
                              : (float)e->rec.confidence / 255.0f;
    float p;
    if (e->rec.flags & BP_RF_STALE)        /* route marker only: baseline value */
        p = bp_kind_weight(BP_KIND_WAYPOINT) * conf;
    else
        p = bp_kind_weight(e->rec.kind) * (1.0f + (float)e->rec.severity / 64.0f) * conf;
    if (e->reported_silent) p *= 1.5f;   /* a lost beacon's record is worth more */
    return p;
}

/* free slot, else evict the least valuable non-own record (only if the newcomer
 * is worth more - or unconditionally when `force`, used for the own record) */
static bp_entry_t *alloc_slot(bp_node_t *n, float incoming, int force) {
    bp_entry_t *worst = NULL;
    float wp = 1e30f;
    int i;
    for (i = 0; i < BP_CACHE_MAX; ++i)
        if (!n->cache[i].used) { memset(&n->cache[i], 0, sizeof n->cache[i]); return &n->cache[i]; }
    for (i = 0; i < BP_CACHE_MAX; ++i) {
        bp_entry_t *e = &n->cache[i];
        float p;
        if (e->is_own) continue;
        p = record_expired(n, &e->rec) ? -1.0f : bp_entry_priority(n, e);
        if (p < wp) { wp = p; worst = e; }
    }
    if (worst && (force || wp < incoming)) {
        n->st.evicted++;
        memset(worst, 0, sizeof *worst);
        return worst;
    }
    return NULL;
}

/* ------------------------------------------------------------ topology */
/* "Is the node that sent this copy AHEAD of us?"  i.e. farther from the
 * record's origin, on our side of it, so its transmission covers the ground
 * we would cover. Used for relay suppression, anti-entropy suppression and
 * passive acknowledgements.
 *
 * Primary source: the ROUTE TREE. Every record carries next_id (the previous
 * beacon, toward the exit), so each node knows the tree the Writer laid down,
 * which follows the corridors. R is ahead of me w.r.t. origin O exactly when
 * the tree path O -> R runs through me:  d(O,R) = d(O,me) + d(me,R), d(me,R)>0.
 * Straight-line distance gets this wrong in winding corridors (a beacon up a
 * side passage can be "farther" from the origin yet cover nothing we cover);
 * simulation showed it silencing the only bridge node toward the gateway.
 * If the tree has a gap (a record on the path not received yet) the answer
 * is -1 "unknown" and callers act conservatively (no suppression, keep
 * waiting for an ack): gaps are transient and a wrong "ahead" is what stalls
 * a flood. Networks laid without next-hop links (no tree at all) fall back
 * to flat-earth geometry from the beacons' own positions.
 * Result: 1 = ahead, 0 = not ahead, -1 = unknown.                          */
typedef struct { uint16_t id[BP_CACHE_MAX + 1]; int len; } chain_t;

static int chain_build(bp_node_t *n, uint16_t id, chain_t *c) {
    c->len = 0;
    while (c->len <= BP_CACHE_MAX) {                     /* bound also breaks loops */
        const bp_entry_t *e = find_mut(n, id);
        c->id[c->len++] = id;
        if (!e) return c->len = -1;                       /* record unknown: gap      */
        if (!(e->rec.flags & BP_RF_HAS_NEXT) || e->rec.next_id == BP_ID_NONE) return c->len; /* root */
        id = e->rec.next_id;
    }
    return c->len = -1;
}

static int chain_dist(const chain_t *a, const chain_t *b) {  /* hops via the common ancestor */
    int i, j;
    if (a->len < 0 || b->len < 0) return -1;
    for (i = 0; i < a->len; ++i)
        for (j = 0; j < b->len; ++j)
            if (a->id[i] == b->id[j]) return i + j;
    return -1;                                            /* different trees          */
}

static int geo_ahead(bp_node_t *n, const bp_record_t *o, uint16_t relay_id) {
    const bp_entry_t *me = find_mut(n, n->cfg.my_id), *r = find_mut(n, relay_id);
    double k, ax, ay, bx, by;
    if (!me || !r) return -1;
    k = cos(bp_e7_to_deg(o->lat_e7) * 0.017453292519943295);
    ax = ((double)me->rec.lon_e7 - (double)o->lon_e7) * k;
    ay = (double)me->rec.lat_e7 - (double)o->lat_e7;
    bx = ((double)r->rec.lon_e7 - (double)o->lon_e7) * k;
    by = (double)r->rec.lat_e7 - (double)o->lat_e7;
    if (ax * bx + ay * by <= 0.0) return 0;              /* other side of the origin */
    return (bx * bx + by * by) > (ax * ax + ay * ay);     /* strictly farther along   */
}

/* co / cm: chains of the origin and of this node, built once by the caller */
static int ahead_c(bp_node_t *n, const bp_record_t *o, const chain_t *co, const chain_t *cm,
                   uint16_t relay_id) {
    chain_t cr;
    int d_om, d_or, d_mr;
    if (relay_id == o->origin_id) return 0;
    chain_build(n, relay_id, &cr);
    d_om = chain_dist(co, cm);
    d_or = chain_dist(co, &cr);
    d_mr = chain_dist(cm, &cr);
    if (d_om >= 0 && d_or >= 0 && d_mr >= 0) return d_mr > 0 && d_or == d_om + d_mr;
    if (!(o->flags & BP_RF_HAS_NEXT)) return geo_ahead(n, o, relay_id);   /* no route tree */
    return -1;
}

static int relay_is_ahead(bp_node_t *n, const bp_record_t *o, uint16_t relay_id) {
    chain_t co, cm;
    if (relay_id == o->origin_id) return 0;
    if (!find_mut(n, n->cfg.my_id)) return -1;           /* we have no position yet */
    chain_build(n, o->origin_id, &co);
    chain_build(n, n->cfg.my_id, &cm);
    return ahead_c(n, o, &co, &cm, relay_id);
}

/* Passive acknowledgement. After relaying, a node expects to OVERHEAR the
 * next node ahead of it relay the same version (that node heard us, so the
 * link is very likely symmetric). If no copy from ahead arrives in time and
 * we do know a live first-hand neighbour ahead of us, the frame was probably
 * lost (collision, fading, busy receiver) -> retry, at most BP_ACK_RETRIES
 * times. Same for the origin's first announce of a new version: any relay
 * of it is the ack. Without this a single collision stalls the flood front
 * until anti-entropy notices (tens of minutes in a 30-beacon chain).     */
static int live_neighbour_ahead(bp_node_t *n, const bp_record_t *o) {
    chain_t co, cm;
    int i;
    if (!find_mut(n, n->cfg.my_id)) return 0;
    chain_build(n, o->origin_id, &co);
    chain_build(n, n->cfg.my_id, &cm);
    for (i = 0; i < BP_CACHE_MAX; ++i) {
        const bp_entry_t *r = &n->cache[i];
        if (!r->used || r->is_own || !recently_heard(r)) continue;
        if (r->rec.origin_id == o->origin_id) continue;
        if (o->origin_id == n->cfg.my_id) return 1;       /* own record: any neighbour */
        if (ahead_c(n, o, &co, &cm, r->rec.origin_id) != 0) return 1;  /* ahead, or maybe */
    }
    return 0;
}

static int may_flood(uint8_t hops, uint8_t limit) {
    return limit == BP_HOPS_UNSCOPED || hops < limit;
}

static void schedule(bp_node_t *n, bp_entry_t *e, uint8_t kind, uint32_t delay_min, uint32_t window) {
    e->relay_pending = 1;
    e->relay_kind = kind;
    e->dup_heard = 0;
    e->relay_at_ms = now_ms(n) + delay_min + rnd(n, window);
}

/* reported silence = first-hand evidence if we have it, hearsay otherwise */
static void update_silence(bp_node_t *n, bp_entry_t *e) {
    uint8_t rep = first_hand(n, e) ? e->silent : e->origin_silent_flag;
    if (rep != e->reported_silent) {
        e->reported_silent = rep;
        emit(n, rep ? BP_EV_SILENT : BP_EV_REVIVED, e, 0);
    }
}

/* ------------------------------------------------------------ transmit */
static int transmit(bp_node_t *n, bp_entry_t *e, uint8_t kind) {
    bp_frame_t f;
    uint8_t buf[BP_FRAME_LEN];
    uint32_t t = now_ms(n);
    float need = (float)n->cfg.frame_airtime_ms;

    /* priority reserve: background traffic may only spend credit above a
     * floor, so a reactive relay or a sync push always finds a frame's worth */
    if (kind == BP_TX_OWN && e->tx_count > e->retries) need *= 2.0f;  /* periodic re-announce;
                                                    a NEW version goes out like a relay */
    else if (kind == BP_TX_AE) need *= 3.0f;
    if (n->budget_ms < need) { n->st.deferred_budget++; return -1; }

    memset(&f, 0, sizeof f);
    f.net_id = n->cfg.net_id;
    f.relay_id = n->cfg.my_id;
    f.hop_limit = e->hop_limit > n->cfg.hop_limit ? n->cfg.hop_limit : e->hop_limit;
    f.hops = e->is_own ? 0 : (uint8_t)(e->hops + 1);
    if (f.hops > f.hop_limit) {
        /* Unscoped records (limit 15): the 4-bit counter just saturates.
         * Scoped records: reactive flooding stops at the limit; anti-entropy
         * and sync may still hand the record to direct neighbours (saturated,
         * so they store it but never re-flood it).                          */
        if (f.hop_limit != BP_HOPS_UNSCOPED && kind != BP_TX_AE && kind != BP_TX_SYNC) return -2;
        f.hops = f.hop_limit;
    }
    /* liveness hints for the receivers: SILENT = we (or our sources) lost X;
     * ALIVE = we hear X ourselves over a good link right now -> clears any
     * stale "silent" hearsay downstream (stops a false alarm spreading).   */
    f.hdr_flags = e->reported_silent ? BP_HF_ORIGIN_SILENT : 0;
    if (!e->is_own && !e->silent && strong_link(n, e) && recently_heard(e)) f.hdr_flags = BP_HF_ORIGIN_ALIVE;
    f.rec = e->rec;
    if (bp_encode(&f, n->cfg.key, buf) != BP_OK) return -3;
    if (n->hal.radio_tx(n->hal.ctx, buf, BP_FRAME_LEN) != 0) return -4;

    n->budget_ms -= (float)n->cfg.frame_airtime_ms;
    n->busy_until_ms = t + n->cfg.frame_airtime_ms;
    n->st.tx_airtime_ms += n->cfg.frame_airtime_ms;
    if (kind == BP_TX_OWN) n->st.tx_own++;
    else if (kind == BP_TX_RELAY) n->st.tx_relay++;
    else if (kind == BP_TX_SYNC) n->st.tx_sync++;
    else n->st.tx_ae++;
    e->last_sent_ms = t;
    e->ack_wait = 0;
    if ((kind == BP_TX_RELAY || (kind == BP_TX_OWN && e->tx_count <= e->retries)) &&
        e->retries < BP_ACK_RETRIES && live_neighbour_ahead(n, &e->rec)) {
        e->ack_wait = 1;                  /* expect to overhear a copy from ahead */
        e->ack_deadline_ms = t + 2u * n->cfg.frame_airtime_ms + 3u * n->cfg.relay_imin_ms;
    }
    e->tx_count++;
    emit(n, BP_EV_TX, e, kind);
    return 0;
}

/* ------------------------------------------------------------ provisioning */
int bp_node_set_own(bp_node_t *n, const bp_record_t *rec) {
    bp_entry_t *e;
    bp_record_t r = *rec;
    r.origin_id = n->cfg.my_id;
    if (bp_validate_record(&r) != BP_OK) return BP_ERR_FIELD;
    e = find_mut(n, n->cfg.my_id);
    if (!e) e = alloc_slot(n, 0.0f, 1);
    if (!e) return BP_ERR_FIELD;
    memset(e, 0, sizeof *e);
    e->used = 1;
    e->is_own = 1;
    e->rec = r;
    e->hops = 0;
    e->hop_limit = n->cfg.hop_limit;
    e->first_seen_ms = e->last_heard_ms = e->last_sent_ms = now_ms(n);
    schedule(n, e, BP_TX_OWN, 0, n->cfg.relay_imin_ms);   /* announce promptly */
    return BP_OK;
}

const bp_entry_t *bp_node_own(const bp_node_t *n) { return bp_node_find(n, n->cfg.my_id); }

int bp_node_seed(bp_node_t *n, const bp_record_t *rec) {
    bp_entry_t *e;
    uint32_t t = now_ms(n);
    if (rec->origin_id == n->cfg.my_id) return BP_ERR_FIELD;   /* use set_own */
    if (bp_validate_record(rec) != BP_OK) return BP_ERR_FIELD;
    if (record_expired(n, rec)) return BP_ERR_FIELD;
    e = find_mut(n, rec->origin_id);
    if (e) {
        if ((int16_t)(uint16_t)(rec->seq - e->rec.seq) <= 0) return BP_OK;   /* have same/newer */
        e->rec = *rec;
        return BP_OK;
    }
    {
        bp_entry_t tmp;
        memset(&tmp, 0, sizeof tmp);
        tmp.rec = *rec;
        e = alloc_slot(n, bp_entry_priority(n, &tmp), 0);
    }
    if (!e) return BP_ERR_FIELD;
    e->used = 1;
    e->rec = *rec;
    e->hops = BP_MAX_HOP_LIMIT;             /* unknown distance */
    e->hop_limit = n->cfg.hop_limit;
    e->first_seen_ms = e->last_heard_ms = e->last_sent_ms = t;
    return BP_OK;
}

/* ------------------------------------------------------------ receive */
/* Reactive-relay back-off weighted by received power: nodes that heard the
 * sender WEAKLY (i.e. far away) fire first, so every hop covers the most new
 * ground and the closer nodes get suppressed by them.                      */
static uint32_t relay_delay_ms(const bp_node_t *n, int16_t rssi) {
    float frac = ((float)rssi + 126.0f) / 80.0f;          /* -126 dBm -> 0, -46 -> 1 */
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    return (uint32_t)((float)n->cfg.relay_imin_ms * (0.25f + 1.5f * frac));
}

/* first-hand proof of life: the node itself transmitted and we heard it */
static void note_direct(bp_entry_t *e, uint32_t t, int16_t rssi) {
    if (!e->heard_direct) e->known_rounds = 1;
    e->link_rssi = e->direct_n ? (int16_t)((3 * (int)e->link_rssi + (int)rssi) / 4) : rssi;
    if (e->direct_n < 255) e->direct_n++;
    e->any_rounds |= 1u;
    e->heard_direct = 1;
    e->last_direct_ms = t;
    e->silent = 0;
    e->origin_silent_flag = 0;
}

static void note_neighbour_alive(bp_node_t *n, uint16_t id, uint32_t t, int16_t rssi) {
    bp_entry_t *r = find_mut(n, id);
    if (!r || r->is_own) return;
    note_direct(r, t, rssi);                  /* it relayed something: alive */
    update_silence(n, r);
}

/* PULL side of push-pull gossip. A digest lists every (origin, seq) the sender
 * holds, sorted by origin. Pages overlap by one entry, so page k covers the
 * origin range [first, last] (page 0 from 0, the final page up to 0xFFFF) with
 * no gaps between pages. Anything we hold inside that range that the sender
 * lacks, or holds an older version of, is a candidate; the `sync_max` most
 * valuable ones are pushed after a short random back-off. Several neighbours
 * usually hear the same digest: a copy pushed by a peer cancels ours.      */
static void handle_digest(bp_node_t *n, const uint8_t *buf, uint8_t len, int16_t rssi) {
    uint16_t sender, org[BP_DIGEST_MAX_ENTRIES], sq[BP_DIGEST_MAX_ENTRIES];
    uint8_t cnt = 0, page, i;
    uint16_t lo, hi;
    bp_entry_t *cand[BP_CACHE_MAX];
    float pri[BP_CACHE_MAX];
    int nc = 0, j, k;
    uint32_t t = now_ms(n);
    int st = bp_decode_digest(buf, len, n->cfg.key, n->cfg.net_id, &sender, org, sq, &cnt,
                              BP_DIGEST_MAX_ENTRIES);
    if (st != BP_OK) {
        if (st == BP_ERR_MAC) n->st.rx_bad_mac++;
        else n->st.rx_bad_other++;
        emit(n, BP_EV_REJECT, NULL, st);
        return;
    }
    n->st.rx_ok++;
    n->st.rx_digest++;
    if (sender == n->cfg.my_id) return;
    note_neighbour_alive(n, sender, t, rssi);
    if (!n->cfg.relay_enabled || n->cfg.sync_max == 0) return;   /* passive listeners don't push */

    for (i = 1; i < cnt; ++i)
        if (org[i] <= org[i - 1]) { n->st.rx_bad_other++; return; } /* must be strictly sorted */
    page = buf[5];
    lo = ((page & BP_DIGEST_PAGE_MASK) == 0 || cnt == 0) ? 0 : org[0];
    hi = ((page & BP_DIGEST_FINAL) || cnt == 0) ? 0xFFFF : org[cnt - 1];

    for (j = 0; j < BP_CACHE_MAX; ++j) {
        bp_entry_t *e = &n->cache[j];
        int want = 1;
        uint16_t o;
        if (!e->used || record_expired(n, &e->rec)) continue;
        o = e->rec.origin_id;
        if (o < lo || o > hi || o == sender) continue;
        if (e->relay_pending && (e->relay_kind == BP_TX_RELAY || e->relay_kind == BP_TX_SYNC)) continue;
        for (i = 0; i < cnt; ++i) {
            if (org[i] == o) { want = (int16_t)(uint16_t)(e->rec.seq - sq[i]) > 0; break; }
            if (org[i] > o) break;                          /* sorted: not listed */
        }
        if (!want) continue;
        pri[nc] = bp_entry_priority(n, e);
        cand[nc++] = e;
    }
    /* partial selection sort: the sync_max most valuable candidates */
    for (k = 0; k < nc && k < (int)n->cfg.sync_max; ++k) {
        int b = k;
        bp_entry_t *te;
        float tp;
        for (j = k + 1; j < nc; ++j) if (pri[j] > pri[b]) b = j;
        te = cand[k]; cand[k] = cand[b]; cand[b] = te;
        tp = pri[k]; pri[k] = pri[b]; pri[b] = tp;
        /* staggered: the most valuable first, then the next one imin later */
        schedule(n, cand[k], BP_TX_SYNC, (uint32_t)k * n->cfg.relay_imin_ms, 2u * n->cfg.relay_imin_ms);
    }
}

void bp_node_on_rx(bp_node_t *n, const uint8_t *buf, uint8_t len, int16_t rssi, int16_t snr_x10) {
    bp_frame_t f;
    bp_entry_t *e;
    uint32_t t = now_ms(n);
    int ev = 0;
    int st;

    if (len > 0 && buf[0] == BP_VER_DIGEST) { handle_digest(n, buf, len, rssi); return; }
    /* v9: robot position reports and mission briefings are for the ONA and the
     * Executor: a beacon neither relays nor rejects them */
    if (len > 0 && (buf[0] == BP_VER_ROBOT || buf[0] == BP_VER_MISSION)) { n->st.rx_foreign++; return; }
    st = bp_decode(buf, len, n->cfg.key, n->cfg.net_id, &f);
    if (st != BP_OK) {
        if (st == BP_ERR_MAC) n->st.rx_bad_mac++;
        else n->st.rx_bad_other++;
        emit(n, BP_EV_REJECT, NULL, st);
        return;
    }
    n->st.rx_ok++;

    /* any authentic frame proves its transmitter is alive and in range */
    if (f.relay_id != f.rec.origin_id) note_neighbour_alive(n, f.relay_id, t, rssi);

    if (f.rec.origin_id == n->cfg.my_id) {            /* our own record echoed back */
        e = find_mut(n, n->cfg.my_id);
        if (e && e->relay_pending) e->dup_heard++;
        if (e && f.rec.seq == e->rec.seq) e->ack_wait = 0;   /* a neighbour relayed us */
        n->st.rx_echo++;
        return;
    }
    if (f.hop_limit > n->cfg.hop_limit) f.hop_limit = n->cfg.hop_limit;   /* enforce global cap */
    if (f.hops > f.hop_limit) { n->st.rx_bad_other++; return; }
    if (n->clock_set && f.rec.timestamp > bp_node_unix(n) + n->cfg.max_future_s) {
        n->st.rx_future++;
        emit(n, BP_EV_REJECT, NULL, -10);
        return;
    }
    if (record_expired(n, &f.rec)) { n->st.rx_stale++; return; }

    e = find_mut(n, f.rec.origin_id);
    if (!e) {
        bp_entry_t tmp;
        memset(&tmp, 0, sizeof tmp);
        tmp.rec = f.rec;
        e = alloc_slot(n, bp_entry_priority(n, &tmp), 0);
        if (!e) return;                                   /* cache full of better data */
        e->used = 1;
        e->rec = f.rec;
        e->hops = f.hops;
        e->hop_limit = f.hop_limit;
        e->first_seen_ms = e->last_heard_ms = e->last_sent_ms = t;
        n->st.rx_new++;
        ev = BP_EV_NEW;
        if (n->cfg.relay_enabled && may_flood(f.hops, f.hop_limit))
            schedule(n, e, BP_TX_RELAY, relay_delay_ms(n, rssi), n->cfg.relay_imin_ms / 2);
    } else {
        int16_t d = (int16_t)(uint16_t)(f.rec.seq - e->rec.seq);
        if (d > 0) {                                      /* newer version: replace */
            e->rec = f.rec;
            e->hops = f.hops;
            e->hop_limit = f.hop_limit;
            e->origin_silent_flag = 0;
            e->last_heard_ms = t;
            e->first_seen_ms = t;                         /* this VERSION is new */
            e->retries = 0;
            e->ack_wait = 0;
            e->tx_count = 0;
            n->st.rx_update++;
            ev = BP_EV_UPDATE;
            if (n->cfg.relay_enabled && may_flood(f.hops, f.hop_limit))
                schedule(n, e, BP_TX_RELAY, relay_delay_ms(n, rssi), n->cfg.relay_imin_ms / 2);
        } else if (d == 0) {
            if (!rec_equal(&e->rec, &f.rec)) {            /* same seq, different body */
                n->st.rx_conflict++;
                return;
            }
            n->st.rx_dup++;
            e->last_heard_ms = t;
            /* Suppression input. For a reactive relay or an anti-entropy push,
             * only a copy sent by a node AHEAD of us (farther from the origin,
             * on our side of it) counts: copies from nodes behind us say
             * nothing about whether the nodes ahead of us have it. Counting
             * them stalls the flood front in a corridor, and let the far side
             * of a chain silence the one bridge node toward the gateway (both
             * seen in simulation). Unknown (tree gap) -> no suppression.
             * A SYNC push answers a digest: any peer copy counts.            */
            if (e->ack_wait) {                            /* weak ack when unknown: a copy */
                int ahead = relay_is_ahead(n, &f.rec, f.relay_id);   /* 2 hops past ours */
                if (ahead == 1 || (ahead < 0 && f.hops >= e->hops + 2)) e->ack_wait = 0;
            }
            if (e->relay_pending) {
                if (e->relay_kind == BP_TX_RELAY || e->relay_kind == BP_TX_AE) {
                    if (relay_is_ahead(n, &f.rec, f.relay_id) == 1) e->dup_heard++;
                } else if (e->relay_kind == BP_TX_SYNC && f.relay_id != f.rec.origin_id) {
                    e->dup_heard++;
                }
            }
            if (f.hops < e->hops) e->hops = f.hops;
        } else {
            n->st.rx_stale++;                             /* old version / replay */
            return;
        }
    }
    e->last_relay_id = f.relay_id;
    if (f.relay_id == f.rec.origin_id) {                 /* first-hand from the origin */
        note_direct(e, t, rssi);                  /* its own announcement */
        e->rssi = rssi;
        e->snr_x10 = snr_x10;
    } else if (f.hdr_flags & BP_HF_ORIGIN_ALIVE) {
        e->origin_silent_flag = 0;                       /* a first-hand witness says alive */
    } else if (f.hdr_flags & BP_HF_ORIGIN_SILENT) {
        e->origin_silent_flag = 1;
    }
    if (ev) emit(n, ev, e, 0);
    update_silence(n, e);
    /* digest sender: something new right after our digest is most likely a
     * sync answer -> there may be more to fetch, pull again soon            */
    if (ev && n->cfg.digest_period_ms && n->st.tx_digest &&
        (uint32_t)TDIFF(t, n->last_digest_ms) < 10000u) {
        uint32_t soon = t + 4u * n->cfg.relay_imin_ms + 2000u;
        if (TDIFF(n->next_digest_ms, soon) > 0) n->next_digest_ms = soon;
    }
}

/* ------------------------------------------------------------ periodic */
static void gossip_round(bp_node_t *n, uint32_t t) {
    uint32_t jitter = n->cfg.round_ms / 5;
    bp_entry_t *own = NULL, *best = NULL;
    float best_score = -1.0f;
    int i;

    n->next_round_ms = t + n->cfg.round_ms - jitter + rnd(n, 2 * jitter);

    for (i = 0; i < BP_CACHE_MAX; ++i) {
        bp_entry_t *e = &n->cache[i];
        if (!e->used) continue;
        if (e->is_own) { own = e; continue; }
        if (record_expired(n, &e->rec)) {                 /* aged out: forget it */
            emit(n, BP_EV_EXPIRED, e, 0);
            n->st.expired++;
            memset(e, 0, sizeof *e);
            continue;
        }
        if (!e->silent && (e->any_rounds & ((1u << BP_SILENT_ROUNDS) - 1u)) == 0 &&
            strong_history(n, e, BP_SILENT_ROUNDS))
            e->silent = 1;                                /* strong neighbour went quiet */
        e->any_rounds = (uint16_t)(e->any_rounds << 1);  /* next round */
        if (e->heard_direct && e->known_rounds < 255) e->known_rounds++;
        update_silence(n, e);
    }

    /* 1) own record. A living beacon never goes quiet: when its EVENT ages out
     *    (TTL or half-life) it re-issues the record as a STALE route marker -
     *    new seq, fresh timestamp, waypoint lifetime, same position / next hop /
     *    kind - so the evacuation route through it stays intact.            */
    if (own && record_expired(n, &own->rec)) {
        own->rec.seq = (uint16_t)(own->rec.seq + 1);
        own->rec.flags |= BP_RF_STALE;
        own->rec.timestamp = bp_node_unix(n);
        own->rec.confidence = 255;
        own->rec.ttl_10s = bp_seconds_to_10s(bp_default_ttl_s(BP_KIND_WAYPOINT));
        own->rec.half_life_10s = bp_seconds_to_10s(bp_default_half_life_s(BP_KIND_WAYPOINT));
        own->relay_pending = 0;
        own->tx_count = 0;                                /* new version: ack its announce */
        own->retries = 0;
        own->ack_wait = 0;
        emit(n, BP_EV_OWN_RENEWED, own, 0);
    }
    if (own && !own->relay_pending)
        schedule(n, own, BP_TX_OWN, 0, n->cfg.round_ms / 10);

    /* 2) anti-entropy: re-announce the most valuable, least recently circulated record */
    if (!n->cfg.relay_enabled) return;
    for (i = 0; i < BP_CACHE_MAX; ++i) {
        bp_entry_t *e = &n->cache[i];
        float score;
        if (!e->used || e->is_own || e->relay_pending) continue;
        /* staleness = time since WE last pushed it. (Hearing it ourselves does
         * not mean our other neighbours have it: in a chain we may be the only
         * bridge - learned the hard way in simulation.) */
        score = bp_entry_priority(n, e) * ((float)TDIFF(t, e->last_sent_ms) / 1000.0f + 1.0f);
        /* freshness boost: a version learned in the last few rounds is pushed
         * harder, so a reactive flood that lost a frame is repaired quickly */
        if ((uint32_t)TDIFF(t, e->first_seen_ms) < 5u * n->cfg.round_ms) score *= 4.0f;
        if (score > best_score) { best_score = score; best = e; }
    }
    if (best) schedule(n, best, BP_TX_AE, n->cfg.round_ms / 10, n->cfg.round_ms / 3);
}

static int tx_rank(uint8_t kind) {
    switch (kind) {
    case BP_TX_RELAY: return 0;   /* a flood front in progress: never hold it up   */
    case BP_TX_SYNC:  return 1;   /* someone asked for exactly this record         */
    case BP_TX_OWN:   return 2;
    default:          return 3;   /* anti-entropy: background repair               */
    }
}

/* Broadcast one page of our (origin, seq) digest. Returns 0 if a frame went
 * out, -1 if deferred (duty-cycle credit) - retried on a later tick.       */
static int send_digest(bp_node_t *n, uint32_t t) {
    uint16_t org[BP_CACHE_MAX], sq[BP_CACHE_MAX];
    uint8_t buf[BP_DIGEST_LEN(BP_DIGEST_MAX_ENTRIES)];
    int total = 0, i, j, start, cnt, fin, len;
    uint8_t page = n->digest_page;
    uint32_t air;

    for (i = 0; i < BP_CACHE_MAX; ++i) {                 /* insertion sort by origin */
        const bp_entry_t *e = &n->cache[i];
        uint16_t o, s;
        if (!e->used || record_expired(n, &e->rec)) continue;
        o = e->rec.origin_id;
        s = e->rec.seq;
        for (j = total; j > 0 && org[j - 1] > o; --j) { org[j] = org[j - 1]; sq[j] = sq[j - 1]; }
        org[j] = o;
        sq[j] = s;
        total++;
    }
    start = page * (BP_DIGEST_MAX_ENTRIES - 1);           /* pages overlap by one entry */
    if (start >= total && page) { page = 0; start = 0; }  /* cache shrank since last page */
    cnt = total - start;
    if (cnt > BP_DIGEST_MAX_ENTRIES) cnt = BP_DIGEST_MAX_ENTRIES;
    fin = start + cnt >= total;

    len = bp_encode_digest(n->cfg.net_id, n->cfg.my_id, org + start, sq + start, (uint8_t)cnt,
                           (uint8_t)((page & BP_DIGEST_PAGE_MASK) | (fin ? BP_DIGEST_FINAL : 0)),
                           n->cfg.key, buf, sizeof buf);
    if (len <= 0) { n->next_digest_ms = t + n->cfg.digest_period_ms; return -1; }
    air = frame_airtime(n, (uint8_t)len);
    if (n->budget_ms < (float)air) { n->st.deferred_budget++; n->next_digest_ms = t + 1000; return -1; }
    if (n->hal.radio_tx(n->hal.ctx, buf, (uint8_t)len) != 0) { n->next_digest_ms = t + 1000; return -1; }

    n->budget_ms -= (float)air;
    n->busy_until_ms = t + air;
    n->st.tx_airtime_ms += air;
    n->st.tx_digest++;
    n->last_digest_ms = t;
    emit(n, BP_EV_TX, NULL, BP_TX_DIGEST);
    if (fin) {
        n->digest_page = 0;
        n->next_digest_ms = t + n->cfg.digest_period_ms - n->cfg.digest_period_ms / 10 +
                            rnd(n, n->cfg.digest_period_ms / 5 + 1);
    } else {
        n->digest_page = (uint8_t)(page + 1);
        n->next_digest_ms = t + air + 4u * n->cfg.relay_imin_ms;  /* answers to this page first */
    }
    return 0;
}

void bp_node_tick(bp_node_t *n) {
    uint32_t t = now_ms(n);
    bp_entry_t *pick = NULL;
    int i, r;

    /* Duty-cycle token bucket, provably compliant: the regulation limits airtime
     * per rolling hour (1 % -> 36 s). A bucket of capacity C refilled at rate r
     * allows at most C + r*3600 s in any hour, so r = duty - C/3600 s.       */
    {
        float rate = n->cfg.duty_cycle - (float)n->cfg.duty_cap_ms / 3600000.0f;
        if (rate < 0.0f) rate = 0.0f;
        n->budget_ms += (float)(uint32_t)(t - n->budget_t_ms) * rate;
    }
    if (n->budget_ms > (float)n->cfg.duty_cap_ms) n->budget_ms = (float)n->cfg.duty_cap_ms;
    n->budget_t_ms = t;

    if (TDIFF(t, n->busy_until_ms) < 0) return;          /* radio still transmitting */

    /* due transmissions, by class: relay > sync > own > anti-entropy */
    for (i = 0; i < BP_CACHE_MAX; ++i) {
        bp_entry_t *e = &n->cache[i];
        if (e->used && e->ack_wait && TDIFF(t, e->ack_deadline_ms) >= 0) {
            e->ack_wait = 0;                              /* nobody ahead repeated it: retry */
            e->retries++;
            n->st.ack_retries++;
            schedule(n, e, e->is_own ? BP_TX_OWN : BP_TX_RELAY, 0, n->cfg.relay_imin_ms);
        }
        if (!e->used || !e->relay_pending || TDIFF(t, e->relay_at_ms) < 0) continue;
        if (e->relay_kind == BP_TX_RELAY && e->dup_heard >= n->cfg.suppress_k) {
            e->relay_pending = 0;                         /* enough neighbours already did it */
            n->st.suppressed++;
            continue;
        }
        if ((e->relay_kind == BP_TX_AE || e->relay_kind == BP_TX_SYNC) && e->dup_heard >= 1) {
            e->relay_pending = 0;                         /* a peer pushed it meanwhile */
            e->last_sent_ms = t;                          /* count it as done for us too */
            n->st.suppressed++;
            continue;
        }
        if (!pick || tx_rank(e->relay_kind) < tx_rank(pick->relay_kind) ||
            (e->relay_kind == pick->relay_kind && TDIFF(e->relay_at_ms, pick->relay_at_ms) < 0))
            pick = e;
    }
    if (pick) {
        r = transmit(n, pick, pick->relay_kind);
        if (r == -1 && (pick->relay_kind == BP_TX_RELAY || pick->relay_kind == BP_TX_OWN))
            pick->relay_at_ms = t + n->cfg.round_ms / 4;  /* out of airtime: retry later */
        else
            pick->relay_pending = 0;                      /* AE / SYNC: next round / digest */
        if (r == 0) return;                               /* one frame per tick */
    }

    if (n->cfg.digest_period_ms && TDIFF(t, n->next_digest_ms) >= 0 && send_digest(n, t) == 0) return;
    if (TDIFF(t, n->next_round_ms) >= 0) gossip_round(n, t);
}

/* ------------------------------------------------------------ introspection */
const bp_entry_t *bp_node_find(const bp_node_t *n, uint16_t origin) {
    int i;
    for (i = 0; i < BP_CACHE_MAX; ++i)
        if (n->cache[i].used && n->cache[i].rec.origin_id == origin) return &n->cache[i];
    return NULL;
}

int bp_node_count(const bp_node_t *n) {
    int i, c = 0;
    for (i = 0; i < BP_CACHE_MAX; ++i) c += n->cache[i].used ? 1 : 0;
    return c;
}

float bp_node_duty_used(const bp_node_t *n) {
    uint32_t up = now_ms(n) - n->boot_ms;
    return up ? (float)n->st.tx_airtime_ms / (float)up : 0.0f;
}

int bp_entry_to_json(const bp_node_t *n, const bp_entry_t *e, const char *event, char *buf, size_t len) {
    int w = bp_record_to_json(&e->rec, buf, len), w2;
    uint32_t unow = bp_node_unix(n);
    uint32_t age = unow > e->rec.timestamp ? unow - e->rec.timestamp : 0;
    float conf = n->clock_set ? bp_effective_confidence(&e->rec, unow) : (float)e->rec.confidence / 255.0f;
    char rs[48];

    if (w < 2) return -1;
    if (e->heard_direct)
        snprintf(rs, sizeof rs, "%d,\"snr\":%.1f", (int)e->rssi, (double)e->snr_x10 / 10.0);
    else
        snprintf(rs, sizeof rs, "null,\"snr\":null");
    w2 = snprintf(buf + w - 1, len - (size_t)(w - 1),
                  ",\"ev\":\"%s\",\"rx_node\":%u,\"age_s\":%lu,\"eff_conf\":%.3f,\"hops\":%u,"
                  "\"relay\":%u,\"rssi\":%s,\"silent\":%s}",
                  event, (unsigned)n->cfg.my_id, (unsigned long)age, (double)conf, (unsigned)e->hops,
                  (unsigned)e->last_relay_id, rs, e->reported_silent ? "true" : "false");
    if (w2 < 0 || (size_t)(w - 1 + w2) >= len) return -1;
    return w - 1 + w2;
}
