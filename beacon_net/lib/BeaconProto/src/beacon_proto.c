/* beacon_proto.c - Living Map beacon protocol v2 codec (see beacon_proto.h). */
#include "beacon_proto.h"
#include "bp_sha256.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static const uint8_t DOMAIN_TAG[4] = {'L', 'M', 'B', '2'};

/* ------------------------------------------------------------ byte helpers */
static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ------------------------------------------------------------ record <-> bytes */
static void pack_record(const bp_record_t *r, uint8_t *p) {
    put16(p + 0, r->origin_id);
    put16(p + 2, r->seq);
    p[4] = r->kind;
    p[5] = r->flags;
    put32(p + 6, (uint32_t)r->lat_e7);
    put32(p + 10, (uint32_t)r->lon_e7);
    p[14] = r->err_dm;
    p[15] = r->severity;
    p[16] = r->confidence;
    p[17] = r->next_bearing;
    put16(p + 18, r->next_id);
    put16(p + 20, r->next_dist_dm);
    put32(p + 22, r->timestamp);
    put16(p + 26, r->ttl_10s);
    put16(p + 28, r->half_life_10s);
}

static void unpack_record(const uint8_t *p, bp_record_t *r) {
    r->origin_id = get16(p + 0);
    r->seq = get16(p + 2);
    r->kind = p[4];
    r->flags = p[5];
    r->lat_e7 = (int32_t)get32(p + 6);
    r->lon_e7 = (int32_t)get32(p + 10);
    r->err_dm = p[14];
    r->severity = p[15];
    r->confidence = p[16];
    r->next_bearing = p[17];
    r->next_id = get16(p + 18);
    r->next_dist_dm = get16(p + 20);
    r->timestamp = get32(p + 22);
    r->ttl_10s = get16(p + 26);
    r->half_life_10s = get16(p + 28);
}

static void compute_mac(const uint8_t key[BP_KEY_LEN], const uint8_t *frame, uint8_t mac[BP_MAC_LEN]) {
    uint8_t msg[4 + 2 + BP_REC_LEN];
    uint8_t full[BP_SHA256_LEN];
    memcpy(msg, DOMAIN_TAG, 4);
    msg[4] = frame[0];                          /* ver_type */
    msg[5] = frame[1];                          /* net_id   */
    memcpy(msg + 6, frame + BP_HDR_LEN, BP_REC_LEN);
    bp_hmac_sha256(key, BP_KEY_LEN, msg, sizeof msg, full);
    memcpy(mac, full, BP_MAC_LEN);
}

int bp_validate_record(const bp_record_t *r) {
    if (r->lat_e7 < -900000000 || r->lat_e7 > 900000000) return BP_ERR_FIELD;
    if (r->lon_e7 < -1800000000 || r->lon_e7 > 1800000000) return BP_ERR_FIELD;
    if (r->ttl_10s == 0 || r->half_life_10s == 0) return BP_ERR_FIELD;
    if (r->origin_id == BP_ID_NONE) return BP_ERR_FIELD;
    if ((r->flags & BP_RF_HAS_NEXT) && r->next_id == BP_ID_NONE) return BP_ERR_FIELD;
    return BP_OK;
}

int bp_encode(const bp_frame_t *f, const uint8_t key[BP_KEY_LEN], uint8_t out[BP_FRAME_LEN]) {
    int st = bp_validate_record(&f->rec);
    if (st != BP_OK) return st;
    if (f->hops > BP_MAX_HOP_LIMIT || f->hop_limit > BP_MAX_HOP_LIMIT || f->hops > f->hop_limit)
        return BP_ERR_FIELD;
    out[0] = BP_VER_TYPE;
    out[1] = f->net_id;
    put16(out + 2, f->relay_id);
    out[4] = (uint8_t)((f->hops << 4) | (f->hop_limit & 0x0F));
    out[5] = f->hdr_flags;
    pack_record(&f->rec, out + BP_HDR_LEN);
    compute_mac(key, out, out + BP_HDR_LEN + BP_REC_LEN);
    return BP_OK;
}

int bp_decode(const uint8_t *buf, size_t len, const uint8_t key[BP_KEY_LEN], uint8_t expect_net,
              bp_frame_t *out) {
    uint8_t mac[BP_MAC_LEN];
    if (len != BP_FRAME_LEN) return BP_ERR_LEN;
    if (buf[0] != BP_VER_TYPE) return BP_ERR_VERSION;
    if (buf[1] != expect_net) return BP_ERR_NET;
    compute_mac(key, buf, mac);
    if (!bp_ct_equal(mac, buf + BP_HDR_LEN + BP_REC_LEN, BP_MAC_LEN)) return BP_ERR_MAC;

    out->net_id = buf[1];
    out->relay_id = get16(buf + 2);
    out->hops = (uint8_t)(buf[4] >> 4);
    out->hop_limit = (uint8_t)(buf[4] & 0x0F);
    out->hdr_flags = buf[5];
    unpack_record(buf + BP_HDR_LEN, &out->rec);
    if (out->hops > out->hop_limit) return BP_ERR_FIELD;
    return bp_validate_record(&out->rec);
}

/* ------------------------------------------------------------ digest frames */
static void digest_mac(const uint8_t key[BP_KEY_LEN], const uint8_t *frame, size_t body_len,
                       uint8_t mac[BP_MAC_LEN]) {
    uint8_t msg[4 + BP_DIGEST_LEN(BP_DIGEST_MAX_ENTRIES)];
    uint8_t full[BP_SHA256_LEN];
    memcpy(msg, DOMAIN_TAG, 4);
    memcpy(msg + 4, frame, body_len);
    bp_hmac_sha256(key, BP_KEY_LEN, msg, 4 + body_len, full);
    memcpy(mac, full, BP_MAC_LEN);
}

int bp_encode_digest(uint8_t net_id, uint16_t sender, const uint16_t *origins, const uint16_t *seqs,
                     uint8_t count, uint8_t page, const uint8_t key[BP_KEY_LEN], uint8_t *out,
                     size_t out_max) {
    size_t body, i;
    if (count > BP_DIGEST_MAX_ENTRIES) return BP_ERR_FIELD;
    body = 6 + 4u * count;
    if (out_max < body + BP_MAC_LEN) return BP_ERR_LEN;
    out[0] = BP_VER_DIGEST;
    out[1] = net_id;
    put16(out + 2, sender);
    out[4] = count;
    out[5] = page;
    for (i = 0; i < count; ++i) {
        put16(out + 6 + 4 * i, origins[i]);
        put16(out + 8 + 4 * i, seqs[i]);
    }
    digest_mac(key, out, body, out + body);
    return (int)(body + BP_MAC_LEN);
}

int bp_decode_digest(const uint8_t *buf, size_t len, const uint8_t key[BP_KEY_LEN], uint8_t expect_net,
                     uint16_t *sender, uint16_t *origins, uint16_t *seqs, uint8_t *count,
                     uint8_t max_entries) {
    uint8_t mac[BP_MAC_LEN];
    size_t n, body, i;
    if (len < BP_DIGEST_LEN(0)) return BP_ERR_LEN;
    if (buf[0] != BP_VER_DIGEST) return BP_ERR_VERSION;
    if (buf[1] != expect_net) return BP_ERR_NET;
    n = buf[4];
    if (n > BP_DIGEST_MAX_ENTRIES || len != BP_DIGEST_LEN(n)) return BP_ERR_LEN;
    body = 6 + 4 * n;
    digest_mac(key, buf, body, mac);
    if (!bp_ct_equal(mac, buf + body, BP_MAC_LEN)) return BP_ERR_MAC;
    if (n > max_entries) n = max_entries;
    *sender = get16(buf + 2);
    for (i = 0; i < n; ++i) {
        origins[i] = get16(buf + 6 + 4 * i);
        seqs[i] = get16(buf + 8 + 4 * i);
    }
    *count = (uint8_t)n;
    return BP_OK;
}

const char *bp_status_str(int s) {
    switch (s) {
        case BP_OK: return "ok";
        case BP_ERR_LEN: return "bad_length";
        case BP_ERR_VERSION: return "bad_version";
        case BP_ERR_NET: return "foreign_network";
        case BP_ERR_MAC: return "bad_mac";
        case BP_ERR_FIELD: return "bad_field";
        default: return "unknown";
    }
}

/* ------------------------------------------------------------ kinds & aging */
static const char *const KIND_NAMES[BP_KIND_COUNT] = {
    "WAYPOINT", "VICTIM", "GAS", "RADIATION", "THERMAL", "OBSTRUCTION", "STRUCTURAL", "EXIT",
    "PHOSPHATE", "GOLD", "GEMSTONE", "SEARCHED"};

/* half-life / TTL per kind: gas information rots in minutes, an obstruction
 * stays true for days.  Seconds. */
static const uint32_t KIND_HALF_LIFE[BP_KIND_COUNT] = {
    43200,  /* WAYPOINT    12 h  */
    3600,   /* VICTIM       1 h  (condition changes, location stays useful) */
    600,    /* GAS         10 min (disperses / drifts)                      */
    21600,  /* RADIATION    6 h  (source is persistent)                     */
    1200,   /* THERMAL     20 min (fires grow or die out)                   */
    172800, /* OBSTRUCTION 48 h                                             */
    86400,  /* STRUCTURAL  24 h                                             */
    172800, /* EXIT        48 h                                             */
    259200, /* PHOSPHATE   72 h (geology does not move: the record ages     */
    259200, /* GOLD        72 h  slowly, but resources stay low priority)    */
    259200, /* GEMSTONE    72 h                                             */
    7200    /* SEARCHED     2 h  (people may move into a searched area)      */
};
static const uint32_t KIND_TTL[BP_KIND_COUNT] = {
    86400, 7200, 3600, 43200, 3600, 604800, 259200, 604800, 604800, 604800, 604800, 43200};
/* gossip priority: rescue first, resources last */
static const float KIND_WEIGHT[BP_KIND_COUNT] = {1.0f, 3.0f, 2.5f, 2.5f, 2.0f, 1.5f, 2.0f, 1.5f, 0.8f, 0.8f, 0.8f, 1.2f};

const char *bp_kind_name(uint8_t k) { return k < BP_KIND_COUNT ? KIND_NAMES[k] : "UNKNOWN"; }

static int ieq(const char *a, const char *b) {
    while (*a && *b) {
        char ca = *a, cb = *b;
        if (ca >= 'a' && ca <= 'z') ca = (char)(ca - 32);
        if (cb >= 'a' && cb <= 'z') cb = (char)(cb - 32);
        if (ca != cb) return 0;
        ++a; ++b;
    }
    return *a == *b;
}

int bp_kind_from_name(const char *name) {
    int i;
    if (!name) return -1;
    for (i = 0; i < BP_KIND_COUNT; ++i)
        if (ieq(name, KIND_NAMES[i])) return i;
    if (ieq(name, "FIRE") || ieq(name, "HEAT")) return BP_KIND_THERMAL;
    if (ieq(name, "TRAIL")) return BP_KIND_WAYPOINT;
    if (ieq(name, "RUBBLE") || ieq(name, "BLOCKED")) return BP_KIND_OBSTRUCTION;
    if (ieq(name, "ROOF") || ieq(name, "COLLAPSE")) return BP_KIND_STRUCTURAL;
    if (ieq(name, "ORE") || ieq(name, "SEAM")) return BP_KIND_PHOSPHATE;
    if (ieq(name, "GEM")) return BP_KIND_GEMSTONE;
    if (ieq(name, "CLEAR") || ieq(name, "NO_VICTIM")) return BP_KIND_SEARCHED;
    return -1;
}

uint32_t bp_default_half_life_s(uint8_t k) { return k < BP_KIND_COUNT ? KIND_HALF_LIFE[k] : 3600; }
uint32_t bp_default_ttl_s(uint8_t k) { return k < BP_KIND_COUNT ? KIND_TTL[k] : 7200; }
float bp_kind_weight(uint8_t k) { return k < BP_KIND_COUNT ? KIND_WEIGHT[k] : 1.0f; }

float bp_effective_confidence(const bp_record_t *r, uint32_t now) {
    float c0 = (float)r->confidence / 255.0f;
    float hl = (float)r->half_life_10s * 10.0f;
    float age = now > r->timestamp ? (float)(now - r->timestamp) : 0.0f;
    if (hl <= 0.0f) return 0.0f;
    return c0 * powf(2.0f, -age / hl);
}

int bp_is_expired(const bp_record_t *r, uint32_t now, float min_conf) {
    uint32_t age = now > r->timestamp ? now - r->timestamp : 0;
    if (age > (uint32_t)r->ttl_10s * 10u) return 1;
    return bp_effective_confidence(r, now) < min_conf;
}

/* ------------------------------------------------------------ unit helpers */
int32_t bp_deg_to_e7(double d) { return (int32_t)(d >= 0 ? d * 1e7 + 0.5 : d * 1e7 - 0.5); }
double bp_e7_to_deg(int32_t e7) { return (double)e7 / 1e7; }

uint8_t bp_bearing_to_u8(double deg) {
    double d = fmod(deg, 360.0);
    long v;
    if (d < 0) d += 360.0;
    v = (long)(d * 256.0 / 360.0 + 0.5);
    return (uint8_t)(v & 0xFF);
}
double bp_u8_to_bearing(uint8_t b) { return (double)b * 360.0 / 256.0; }

uint16_t bp_seconds_to_10s(uint32_t s) {
    uint32_t v = (s + 5u) / 10u;
    if (v == 0) v = 1;
    if (v > 0xFFFFu) v = 0xFFFFu;
    return (uint16_t)v;
}

/* ------------------------------------------------------------ LoRa airtime */
uint32_t bp_lora_airtime_ms(uint8_t pl, uint8_t sf, uint32_t bw, uint8_t cr_denom, uint8_t preamble,
                            int crc_on, int implicit_header) {
    double tsym = (double)(1UL << sf) / (double)bw * 1000.0; /* ms */
    int de = tsym > 16.0 ? 1 : 0;                              /* low data rate optimise */
    int cr = cr_denom - 4;
    double num = 8.0 * pl - 4.0 * sf + 28 + 16 * (crc_on ? 1 : 0) - 20 * (implicit_header ? 1 : 0);
    double den = 4.0 * (sf - 2 * de);
    double n = ceil(num / den) * (cr + 4);
    double payload_syms = 8 + (n > 0 ? n : 0);
    double total = ceil((preamble + 4.25 + payload_syms) * tsym);
    return (uint32_t)total;
}

/* ------------------------------------------------------------ JSON */
int bp_record_to_json(const bp_record_t *r, char *buf, size_t n) {
    char err[16], next[96];
    int w;
    if (r->err_dm == BP_ERR_UNKNOWN) snprintf(err, sizeof err, "null");
    else snprintf(err, sizeof err, "%.1f", r->err_dm / 10.0);

    if (r->flags & BP_RF_HAS_NEXT) {
        if (r->flags & BP_RF_HAS_BEARING)
            snprintf(next, sizeof next, "{\"id\":%u,\"dist_m\":%.1f,\"bearing_deg\":%.1f}",
                     (unsigned)r->next_id, r->next_dist_dm / 10.0, bp_u8_to_bearing(r->next_bearing));
        else
            snprintf(next, sizeof next, "{\"id\":%u,\"dist_m\":%.1f}", (unsigned)r->next_id,
                     r->next_dist_dm / 10.0);
    } else {
        snprintf(next, sizeof next, "null");
    }

    w = snprintf(buf, n,
                 "{\"beacon_id\":%u,\"seq\":%u,\"kind\":\"%s\","
                 "\"gps\":{\"lat\":%.7f,\"lon\":%.7f,\"err_m\":%s,\"src\":\"%s\"},"
                 "\"severity\":%u,\"confidence\":%.2f,\"next\":%s,"
                 "\"ts\":%lu,\"ttl_s\":%lu,\"half_life_s\":%lu,\"retracted\":%s,\"stale\":%s}",
                 (unsigned)r->origin_id, (unsigned)r->seq, bp_kind_name(r->kind),
                 bp_e7_to_deg(r->lat_e7), bp_e7_to_deg(r->lon_e7), err,
                 (r->flags & BP_RF_POS_GNSS) ? "gnss" : "slam",
                 (unsigned)r->severity, r->confidence / 255.0, next,
                 (unsigned long)r->timestamp, (unsigned long)r->ttl_10s * 10UL,
                 (unsigned long)r->half_life_10s * 10UL,
                 (r->flags & BP_RF_RETRACTED) ? "true" : "false",
                 (r->flags & BP_RF_STALE) ? "true" : "false");
    if (w < 0 || (size_t)w >= n) return -1;
    return w;
}
