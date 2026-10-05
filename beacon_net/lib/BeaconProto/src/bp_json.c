/* bp_json.c - mission-log JSON -> bp_record_t (see bp_json.h). */
#include "bp_json.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define MAX_DEPTH 8
#define PATH_LEN  48
#define STR_LEN   24

typedef struct {
    const char *p;
    int bad;
    /* collected values (have_* = seen) */
    int have_id, have_lat, have_lon, have_next, have_bearing;
    double id, seq, lat, lon, err_m, sev, conf, next_id, next_dist, bearing, ts, ttl, hl;
    int err_null, kind, gnss, retracted, stale;
    int have_seq, have_err, have_sev, have_conf, have_ts, have_ttl, have_hl, have_dist;
} jctx_t;

static void ws(jctx_t *c) {
    while (*c->p == ' ' || *c->p == '\t' || *c->p == '\n' || *c->p == '\r') c->p++;
}

static int ieq(const char *a, const char *b) {
    while (*a && *b) {
        char x = *a++, y = *b++;
        if (x >= 'a' && x <= 'z') x = (char)(x - 32);
        if (y >= 'a' && y <= 'z') y = (char)(y - 32);
        if (x != y) return 0;
    }
    return *a == *b;
}

/* reads a JSON string at c->p (must start with '"'), keeps up to cap-1 chars */
static void read_string(jctx_t *c, char *dst, size_t cap) {
    size_t n = 0;
    if (*c->p != '"') { c->bad = 1; return; }
    c->p++;
    while (*c->p && *c->p != '"') {
        char ch = *c->p++;
        if (ch == '\\') {
            ch = *c->p;
            if (!ch) break;
            c->p++;
            if (ch == 'u') {                         /* \uXXXX: not needed for our keys */
                int k;
                for (k = 0; k < 4 && *c->p; ++k) c->p++;
                ch = '?';
            } else if (ch == 'n') ch = '\n';
            else if (ch == 't') ch = '\t';
        }
        if (n + 1 < cap) dst[n++] = ch;
    }
    if (*c->p != '"') { c->bad = 1; return; }
    c->p++;
    if (cap) dst[n] = 0;
}

static void on_number(jctx_t *c, const char *path, double v) {
    if (!strcmp(path, "beacon_id") || !strcmp(path, "id")) { c->id = v; c->have_id = 1; }
    else if (!strcmp(path, "seq")) { c->seq = v; c->have_seq = 1; }
    else if (!strcmp(path, "kind") || !strcmp(path, "type")) {
        if (v >= 0 && v < BP_KIND_COUNT && v == floor(v)) c->kind = (int)v; else c->bad = 1;
    }
    else if (!strcmp(path, "gps.lat") || !strcmp(path, "lat")) { c->lat = v; c->have_lat = 1; }
    else if (!strcmp(path, "gps.lon") || !strcmp(path, "lon")) { c->lon = v; c->have_lon = 1; }
    else if (!strcmp(path, "gps.err_m") || !strcmp(path, "err_m")) { c->err_m = v; c->have_err = 1; }
    else if (!strcmp(path, "severity")) { c->sev = v; c->have_sev = 1; }
    else if (!strcmp(path, "confidence")) { c->conf = v; c->have_conf = 1; }
    else if (!strcmp(path, "next.id")) { c->next_id = v; c->have_next = 1; }
    else if (!strcmp(path, "next.dist_m")) { c->next_dist = v; c->have_dist = 1; }
    else if (!strcmp(path, "next.bearing_deg")) { c->bearing = v; c->have_bearing = 1; }
    else if (!strcmp(path, "ts")) { c->ts = v; c->have_ts = 1; }
    else if (!strcmp(path, "ttl_s")) { c->ttl = v; c->have_ttl = 1; }
    else if (!strcmp(path, "half_life_s")) { c->hl = v; c->have_hl = 1; }
}

static void on_string(jctx_t *c, const char *path, const char *s) {
    if (!strcmp(path, "kind") || !strcmp(path, "type")) {
        int k = bp_kind_from_name(s);
        if (k < 0) c->bad = 1; else c->kind = k;
    } else if (!strcmp(path, "gps.src")) {
        c->gnss = ieq(s, "gnss") || ieq(s, "gps");
    }
}

static void on_literal(jctx_t *c, const char *path, int lit /* 0 false, 1 true, 2 null */) {
    if (!strcmp(path, "retracted")) c->retracted = lit == 1;
    else if (!strcmp(path, "stale")) c->stale = lit == 1;
    else if (!strcmp(path, "gps.err_m") || !strcmp(path, "err_m")) { if (lit == 2) c->err_null = 1; }
}

static void parse_value(jctx_t *c, const char *path, int depth);

static void child_path(char *dst, const char *path, const char *key) {
    size_t a = strlen(path), b = strlen(key);
    if (a + b + 2 > PATH_LEN) { dst[0] = '~'; dst[1] = 0; return; }   /* too deep: ignored */
    memcpy(dst, path, a);
    if (a) dst[a++] = '.';
    memcpy(dst + a, key, b + 1);
}

static void parse_object(jctx_t *c, const char *path, int depth) {
    c->p++;                                           /* '{' */
    ws(c);
    if (*c->p == '}') { c->p++; return; }
    while (!c->bad) {
        char key[STR_LEN], sub[PATH_LEN];
        ws(c);
        read_string(c, key, sizeof key);
        if (c->bad) return;
        ws(c);
        if (*c->p != ':') { c->bad = 1; return; }
        c->p++;
        child_path(sub, path, key);
        parse_value(c, sub, depth + 1);
        ws(c);
        if (*c->p == ',') { c->p++; continue; }
        if (*c->p == '}') { c->p++; return; }
        c->bad = 1;
    }
}

static void parse_array(jctx_t *c, int depth) {    /* arrays carry nothing we use */
    c->p++;
    ws(c);
    if (*c->p == ']') { c->p++; return; }
    while (!c->bad) {
        parse_value(c, "~", depth + 1);
        ws(c);
        if (*c->p == ',') { c->p++; continue; }
        if (*c->p == ']') { c->p++; return; }
        c->bad = 1;
    }
}

static void parse_value(jctx_t *c, const char *path, int depth) {
    if (depth > MAX_DEPTH) { c->bad = 1; return; }
    ws(c);
    switch (*c->p) {
    case '{': parse_object(c, path, depth); return;
    case '[': parse_array(c, depth); return;
    case '"': {
        char s[STR_LEN];
        read_string(c, s, sizeof s);
        if (!c->bad) on_string(c, path, s);
        return;
    }
    case 't':
        if (!strncmp(c->p, "true", 4)) { c->p += 4; on_literal(c, path, 1); return; }
        break;
    case 'f':
        if (!strncmp(c->p, "false", 5)) { c->p += 5; on_literal(c, path, 0); return; }
        break;
    case 'n':
        if (!strncmp(c->p, "null", 4)) { c->p += 4; on_literal(c, path, 2); return; }
        break;
    default: {
        char *end;
        double v = strtod(c->p, &end);
        if (end != c->p && isfinite(v)) { c->p = end; on_number(c, path, v); return; }
        break;
    }
    }
    c->bad = 1;
}

static int whole(double v, double lo, double hi) { return v >= lo && v <= hi && v == floor(v); }

int bp_record_from_json(const char *js, bp_record_t *out, uint32_t now_unix) {
    jctx_t c;
    bp_record_t r;
    uint32_t ttl_s, hl_s;

    if (!js || !out) return BP_ERR_FIELD;
    memset(&c, 0, sizeof c);
    c.p = js;
    c.kind = BP_KIND_WAYPOINT;
    ws(&c);
    if (*c.p != '{') return BP_ERR_FIELD;
    parse_value(&c, "", 0);
    ws(&c);
    if (c.bad || *c.p) return BP_ERR_FIELD;          /* trailing garbage too */
    if (!c.have_id || !c.have_lat || !c.have_lon) return BP_ERR_FIELD;

    memset(&r, 0, sizeof r);
    if (!whole(c.id, 0, 65534)) return BP_ERR_FIELD;
    r.origin_id = (uint16_t)c.id;
    if (c.have_seq && !whole(c.seq, 0, 65535)) return BP_ERR_FIELD;
    r.seq = c.have_seq ? (uint16_t)c.seq : 1;
    r.kind = (uint8_t)c.kind;
    if (c.lat < -90.0 || c.lat > 90.0 || c.lon < -180.0 || c.lon > 180.0) return BP_ERR_FIELD;
    r.lat_e7 = bp_deg_to_e7(c.lat);
    r.lon_e7 = bp_deg_to_e7(c.lon);
    if (c.have_err && !c.err_null) {
        if (c.err_m < 0) return BP_ERR_FIELD;
        r.err_dm = c.err_m >= 25.4 ? 254 : (uint8_t)(c.err_m * 10.0 + 0.5);   /* 25.4 m = cap */
    } else {
        r.err_dm = BP_ERR_UNKNOWN;
    }
    if (c.have_sev && !whole(c.sev, 0, 255)) return BP_ERR_FIELD;
    r.severity = c.have_sev ? (uint8_t)c.sev : 0;
    if (c.have_conf && (c.conf < 0.0 || c.conf > 1.0)) return BP_ERR_FIELD;
    r.confidence = c.have_conf ? (uint8_t)(c.conf * 255.0 + 0.5) : 255;
    if (c.gnss) r.flags |= BP_RF_POS_GNSS;
    if (c.retracted) r.flags |= BP_RF_RETRACTED;
    if (c.stale) r.flags |= BP_RF_STALE;
    r.next_id = BP_ID_NONE;
    if (c.have_next) {
        if (!whole(c.next_id, 0, 65534)) return BP_ERR_FIELD;
        r.flags |= BP_RF_HAS_NEXT;
        r.next_id = (uint16_t)c.next_id;
        if (c.have_dist) {
            if (c.next_dist < 0) return BP_ERR_FIELD;
            r.next_dist_dm = c.next_dist >= 6553.5 ? 0xFFFF : (uint16_t)(c.next_dist * 10.0 + 0.5);
        }
        if (c.have_bearing) {
            r.flags |= BP_RF_HAS_BEARING;
            r.next_bearing = bp_bearing_to_u8(c.bearing);
        }
    }
    if (c.have_ts && !whole(c.ts, 0, 4294967295.0)) return BP_ERR_FIELD;
    r.timestamp = c.have_ts ? (uint32_t)c.ts : now_unix;
    if (c.have_ttl && !(c.ttl > 0 && c.ttl <= 655350.0)) return BP_ERR_FIELD;
    if (c.have_hl && !(c.hl > 0 && c.hl <= 655350.0)) return BP_ERR_FIELD;
    ttl_s = c.have_ttl ? (uint32_t)c.ttl : bp_default_ttl_s(r.kind);
    hl_s = c.have_hl ? (uint32_t)c.hl : bp_default_half_life_s(r.kind);
    r.ttl_10s = bp_seconds_to_10s(ttl_s);
    r.half_life_10s = bp_seconds_to_10s(hl_s);
    if (bp_validate_record(&r) != BP_OK) return BP_ERR_FIELD;
    *out = r;
    return BP_OK;
}

static int hexval(char ch) {
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    return -1;
}

int bp_key_from_hex(const char *hex, uint8_t key[BP_KEY_LEN]) {
    int i;
    if (!hex || strlen(hex) != 2 * BP_KEY_LEN) return BP_ERR_FIELD;
    for (i = 0; i < BP_KEY_LEN; ++i) {
        int hi = hexval(hex[2 * i]), lo = hexval(hex[2 * i + 1]);
        if (hi < 0 || lo < 0) return BP_ERR_FIELD;
        key[i] = (uint8_t)(hi << 4 | lo);
    }
    return BP_OK;
}
