/*
 * bpsim - discrete-time simulator for a Living Map beacon network.
 * ==========================================================================
 * Every node runs the REAL bp_gossip engine (same code as the ESP32 firmware)
 * over a simulated 868 MHz LoRa channel:
 *   - log-distance path loss + rubble attenuation per metre + wall loss per
 *     corridor turn + static shadowing per link + per-packet fading
 *   - receiver sensitivity by spreading factor, capture effect (6 dB),
 *     collisions, half-duplex radios, exact time-on-air
 *   - 1 % duty-cycle token bucket enforced by the engine itself
 *
 * Scenario (defaults):
 *   The Writer drops beacons 1..N along a winding corridor, one every ~45 s.
 *   Beacon 1 is the EXIT, 4 is GAS, 7 is the VICTIM (the reference record),
 *   10 RADIATION, 12 OBSTRUCTION, the rest are trail WAYPOINTs. Each record
 *   points to the previous beacon ("next hop" = way out).
 *   A gateway (gw1) listens at the entrance and prints what it learns as
 *   JSON lines on stdout  ->  pipe into python/beaconnet/gateway_bridge.py
 *   to see it live on the Command Post dashboard.
 *   At 25 min beacon 6 is destroyed. At 30 min the victim record is updated
 *   (seq 2, severity 40). An attacker injects forged, tampered and replayed
 *   frames every 90 s.
 *
 * The run ends with a report and PASS/FAIL checks (exit code 1 on failure),
 * so this doubles as an end-to-end test.
 *
 *   ./bpsim                       # 60 min scenario, as fast as possible
 *   ./bpsim --speed 10 | python3 ../python/beaconnet/gateway_bridge.py --stdin
 *   ./bpsim --help
 */
#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#endif
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#include <windows.h>
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "beacon_proto.h"
#include "bp_gossip.h"

#define MAXN 40
#define MAXTX 512
#define GW_ID 900
#define ATTACK_FAKE_ID 4040

/* ------------------------------------------------------------ parameters */
static int    P_beacons = 14;
static double P_minutes = 60;
static uint32_t P_seed = 7;
static double P_speed = 0;          /* 0 = as fast as possible, 1 = real time */
static int    P_sf = 9;
static int    P_round_s = 0;        /* 0 = engine default (auto from SF)        */
static int    P_digest_s = 90;      /* gateway digest period, 0 = pull disabled */
static int    P_seed_route = 1;
static double P_gw_late = 0;        /* gateway powered on at this minute       */     /* Writer primes each new beacon with the route */
static int    P_kill_id = 6;
static double P_kill_min = 25;
static int    P_attacker = 1;
static int    P_quiet = 0;
static double P_ptx = 14.0;         /* dBm (EU 868 limit 25 mW ERP)       */
static double P_ple = 3.0;          /* path-loss exponent                  */
static double P_rubble = 1.2;       /* extra dB per metre through debris   */
static double P_wall = 8.0;         /* dB per corridor turn between nodes  */
static double P_shadow = 4.0;       /* static shadowing sigma, dB          */
static double P_fade = 2.0;         /* per-packet fading sigma, dB         */
static double P_lat0 = 34.4300, P_lon0 = 8.7830;   /* entrance anchor      */
static int    P_gateways = 1;       /* v9: 1..3 gateways (the ONA's), gw2/gw3 near the entrance */

static const uint8_t NET_KEY[BP_KEY_LEN] = {0x4c, 0x69, 0x76, 0x69, 0x6e, 0x67, 0x4d, 0x61,
                                            0x70, 0x2d, 0x44, 0x45, 0x4d, 0x4f, 0x21, 0x21};

/* ------------------------------------------------------------ rng */
static uint64_t rng_s = 88172645463325252ULL;
static uint32_t xr(void) { rng_s ^= rng_s << 13; rng_s ^= rng_s >> 7; rng_s ^= rng_s << 17; return (uint32_t)(rng_s >> 11); }
static double ur(void) { return (xr() % 1000000) / 1e6; }   /* uniform [0,1) */
static double gauss(double s) {
    double u1 = (xr() % 999999 + 1) / 1e6, u2 = (xr() % 1000000) / 1e6;
    return s * sqrt(-2.0 * log(u1)) * cos(2 * M_PI * u2);
}

/* ------------------------------------------------------------ world */
typedef struct {
    int      used, is_gw, alive, deployed;
    double   x, y;
    int      seg;
    uint32_t deploy_ms, kill_ms;
    int      clock_err_s;
    bp_node_t node;
    bp_record_t own;
    uint32_t busy_until;     /* radio TX end (half duplex)                   */
    uint32_t tx_log[4096];   /* TX start times, for the rolling-hour duty    */
    uint16_t tx_air[4096];   /* ... and each frame's time-on-air (ms)        */
    int      tx_n;
} simnode_t;

typedef struct {
    int      src;            /* node index, -1 = attacker                    */
    double   x, y; int seg;  /* transmitter location                         */
    uint32_t start, end;
    uint8_t  buf[256];       /* records are 44 B, digests up to 254 B        */
    uint8_t  len;
    int      delivered;
} air_t;

static simnode_t NODE[MAXN];
static int NN = 0;
/* v9: the ONA's 2nd and 3rd gateways. Kept out of NODE[] so that the report and
 * every check of the default scenario stay exactly as before (they are about gw1).
 * In the air they are transmitters -2 and -3; they hear and pull like gw1. */
#define MAXGW 3
static simnode_t XGW[MAXGW - 1];
static const double XGW_POS[MAXGW - 1][2] = {{-6.0, -5.0}, {-7.0, 4.0}};
static double xshadow[MAXGW - 1][MAXN + 2];
static int sim_index(const simnode_t *s) {          /* NODE index, or -2/-3 for the extra gateways */
    if (s >= NODE && s < NODE + MAXN) return (int)(s - NODE);
    return -2 - (int)(s - XGW);
}
static const char *gw_name(const simnode_t *s) {
    static const char *names[MAXGW] = {"gw1", "gw2", "gw3"};
    int i = sim_index(s);
    return i >= 0 ? names[0] : names[-1 - i];
}
static air_t AIR[MAXTX];
static uint32_t g_now = 0;
static uint32_t g_unix0 = 0;
static double shadow[MAXN + 1][MAXN + 1];
static uint32_t airtime_ms;
static double sens_dbm;

/* metrics */
static uint32_t gw_first[MAXN], gw_silent_at[MAXN];
static uint8_t  gw_first_hops[MAXN];    /* hops of the copy that first reached the gateway */
static int g_txlog = -1;               /* BPSIM_TXLOG=origin: log that record's TX/RX */
static uint32_t got_at[MAXN][MAXN];     /* [node][origin] first acquisition, for BPSIM_TRACE */
static uint32_t victim_update_at = 0, victim_update_sent = 0;
static uint32_t false_lost = 0;          /* gateway "lost" alarms for beacons still alive */
static uint32_t attack_forged = 0, attack_tampered = 0, attack_replayed = 0, attack_inflated = 0;
static uint8_t  last_capture[BP_FRAME_LEN]; static int have_capture = 0;

static int idx_of(uint16_t id) {
    int i;
    for (i = 0; i < NN; ++i) if (NODE[i].node.cfg.my_id == id) return i;
    return -1;
}

/* ------------------------------------------------------------ HAL */
static uint32_t hal_now(void *ctx) { (void)ctx; return g_now; }
static uint32_t hal_rand(void *ctx) { (void)ctx; return xr(); }

static uint32_t air_ms(uint8_t len) {
    return bp_lora_airtime_ms(len, (uint8_t)P_sf, 125000, 5, 8, 1, 0);
}

static void air_push(int src, double x, double y, int seg, const uint8_t *buf, uint8_t len) {
    int i, slot = -1;
    for (i = 0; i < MAXTX; ++i)
        if (AIR[i].end == 0 || (AIR[i].delivered && g_now - AIR[i].end > 2000)) { slot = i; break; }
    if (slot < 0) return;
    AIR[slot].src = src; AIR[slot].x = x; AIR[slot].y = y; AIR[slot].seg = seg;
    AIR[slot].start = g_now; AIR[slot].end = g_now + air_ms(len);
    memcpy(AIR[slot].buf, buf, len);
    AIR[slot].len = len;
    AIR[slot].delivered = 0;
}

static int hal_tx(void *ctx, const uint8_t *buf, uint8_t len) {
    simnode_t *s = (simnode_t *)ctx;
    air_push(sim_index(s), s->x, s->y, s->seg, buf, len);
    s->busy_until = g_now + air_ms(len);
    if (s->tx_n < 4096) { s->tx_log[s->tx_n] = g_now; s->tx_air[s->tx_n++] = (uint16_t)air_ms(len); }
    if (!s->is_gw && buf[0] == BP_VER_TYPE) { memcpy(last_capture, buf, BP_FRAME_LEN); have_capture = 1; }
    return 0;
}

static const char *ev_name(int ev) {
    switch (ev) {
        case BP_EV_NEW: return "new"; case BP_EV_UPDATE: return "update";
        case BP_EV_SILENT: return "silent"; case BP_EV_REVIVED: return "revived";
        case BP_EV_EXPIRED: return "expired"; case BP_EV_REJECT: return "reject";
        default: return "tx";
    }
}

static void hal_event(void *ctx, int ev, const bp_node_t *n, const bp_entry_t *e, int info) {
    simnode_t *s = (simnode_t *)ctx;
    char json[768];
    int si = sim_index(s);
    if (si >= 0 && e && (ev == BP_EV_NEW) && e->rec.origin_id < MAXN && !got_at[si][e->rec.origin_id])
        got_at[si][e->rec.origin_id] = g_now ? g_now : 1;
    if (getenv("BPSIM_DEBUG_LOST") && !s->is_gw && ev == BP_EV_SILENT && e && idx_of(e->rec.origin_id) >= 0 &&
        NODE[idx_of(e->rec.origin_id)].alive)
        fprintf(stderr, "    node %d judges #%u silent at %.1f min: any %04x rssi %d n %u, last direct %.0f s ago, "
                        "first-hand %d, hearsay %d\n", (int)(s - NODE), (unsigned)e->rec.origin_id, g_now / 60000.0,
                (unsigned)e->any_rounds, (int)e->link_rssi, (unsigned)e->direct_n, (g_now - e->last_direct_ms) / 1000.0,
                e->silent, e->origin_silent_flag);
    if (g_txlog >= 0 && e && e->rec.origin_id == g_txlog && (ev == BP_EV_TX || ev == BP_EV_NEW))
        fprintf(stderr, "  [txlog %8.1f s] node %2d %s (hops %u, kind %d)\n", g_now / 1000.0, (int)(s - NODE),
                ev == BP_EV_TX ? "TX " : "got", (unsigned)e->hops, info);
    if (!s->is_gw || ev == BP_EV_TX) return;
    if (ev == BP_EV_REJECT) {
        printf("{\"ev\":\"reject\",\"gw\":\"%s\",\"reason\":\"%s\",\"rx_node\":%u,\"ts\":%lu}\n", gw_name(s),
               info == -10 ? "future_timestamp" : bp_status_str(info), (unsigned)n->cfg.my_id,
               (unsigned long)bp_node_unix(n));
        fflush(stdout);
        return;
    }
    if (bp_entry_to_json(n, e, ev_name(ev), json, sizeof json) > 0) {
        if (P_gateways > 1 && json[0] == '{') printf("{\"gw\":\"%s\",%s\n", gw_name(s), json + 1);
        else printf("%s\n", json);
        fflush(stdout);
    }
    if (si != 0) return;                     /* the report below is about gw1 */
    {
        int id = e->rec.origin_id;
        if (id < MAXN) {
            if ((ev == BP_EV_NEW) && !gw_first[id]) { gw_first[id] = g_now; gw_first_hops[id] = e->hops; }
            if (ev == BP_EV_SILENT && !gw_silent_at[id]) gw_silent_at[id] = g_now;
            if (ev == BP_EV_SILENT && idx_of((uint16_t)id) >= 0 && NODE[idx_of((uint16_t)id)].alive) {
                false_lost++;
                if (getenv("BPSIM_DEBUG_LOST"))
                    fprintf(stderr, "  false lost: #%d at %.1f min, any %04x rssi %d n %u, last direct %.0f s ago, hearsay %d, last relay %u\n",
                            id, g_now / 60000.0, (unsigned)e->any_rounds, (int)e->link_rssi, (unsigned)e->direct_n,
                            (g_now - e->last_direct_ms) / 1000.0, e->origin_silent_flag, (unsigned)e->last_relay_id);
            }
        }
        /* the update may arrive as UPDATE, or as NEW if v1 never made it */
        if ((ev == BP_EV_UPDATE || ev == BP_EV_NEW) && id == 7 && e->rec.seq == 2 && victim_update_sent &&
            !victim_update_at) victim_update_at = g_now;
    }
}

/* ------------------------------------------------------------ radio channel */
static double rx_power(double tx_x, double tx_y, int tx_seg, int tx_idx, int rx) {
    double dx = tx_x - NODE[rx].x, dy = tx_y - NODE[rx].y;
    double d = sqrt(dx * dx + dy * dy);
    double pl, sh;
    if (d < 1.0) d = 1.0;
    pl = 31.5 + 10.0 * P_ple * log10(d) + P_rubble * d + P_wall * abs(tx_seg - NODE[rx].seg);
    sh = tx_idx >= 0 ? shadow[tx_idx][rx] : shadow[MAXN][rx];
    return P_ptx - pl + sh;
}

/* v9: reception at an extra gateway k (index -2-k in the air): same channel model as rx_power */
static double xgw_power(double tx_x, double tx_y, int tx_seg, int tx_idx, int k) {
    double dx = tx_x - XGW[k].x, dy = tx_y - XGW[k].y;
    double d = sqrt(dx * dx + dy * dy);
    if (d < 1.0) d = 1.0;
    return P_ptx - (31.5 + 10.0 * P_ple * log10(d) + P_rubble * d + P_wall * abs(tx_seg - XGW[k].seg))
           + xshadow[k][tx_idx >= 0 && tx_idx < MAXN ? tx_idx : MAXN];
}
static void xgw_rx(int ai, int k) {
    air_t *a = &AIR[ai];
    simnode_t *s = &XGW[k];
    int j, me = -2 - k;
    double p, snr;
    if (a->src == me || !s->deployed) return;
    for (j = 0; j < MAXTX; ++j) {                 /* half duplex */
        air_t *b = &AIR[j];
        if (j != ai && b->end && b->src == me && b->start < a->end && a->start < b->end) return;
    }
    p = xgw_power(a->x, a->y, a->seg, a->src, k) + gauss(P_fade);
    if (p < sens_dbm) return;
    for (j = 0; j < MAXTX; ++j) {                 /* collisions with capture effect */
        air_t *b = &AIR[j];
        if (j == ai || b->end == 0 || b->src == me) continue;
        if (!(b->start < a->end && a->start < b->end)) continue;
        if (xgw_power(b->x, b->y, b->seg, b->src, k) > p - 6.0) return;
    }
    snr = p + 117.0;
    if (snr > 12) snr = 12;
    bp_node_on_rx(&s->node, a->buf, a->len, (int16_t)p, (int16_t)(snr * 10));
}

#define DBG(a) (g_txlog >= 0 && (a)->buf[0] == BP_VER_TYPE && ((a)->buf[6] | ((a)->buf[7] << 8)) == g_txlog)
static void deliver(void) {
    int i, j, r;
    for (i = 0; i < MAXTX; ++i) {
        air_t *a = &AIR[i];
        if (a->end == 0 || a->delivered || a->end > g_now) continue;
        a->delivered = 1;
        for (r = 0; r < NN; ++r) {
            simnode_t *s = &NODE[r];
            double p, snr;
            int lost = 0;
            if (r == a->src || !s->alive || !s->deployed) continue;
            /* half duplex: receiver transmitting during this frame hears nothing */
            for (j = 0; j < MAXTX && !lost; ++j) {
                air_t *b = &AIR[j];
                if (j == i || b->end == 0 || b->src != r) continue;
                if (b->start < a->end && a->start < b->end) lost = 1;
            }
            if (lost) { if (DBG(a)) fprintf(stderr, "    rx %d: half-duplex\n", r); continue; }
            p = rx_power(a->x, a->y, a->seg, a->src, r) + gauss(P_fade);
            if (p < sens_dbm) continue;
            /* collisions with capture effect */
            for (j = 0; j < MAXTX && !lost; ++j) {
                air_t *b = &AIR[j];
                double pi;
                if (j == i || b->end == 0 || b->src == r) continue;
                if (!(b->start < a->end && a->start < b->end)) continue;
                pi = rx_power(b->x, b->y, b->seg, b->src, r);
                if (pi > p - 6.0) { lost = 1; if (DBG(a)) fprintf(stderr, "    rx %d: collision with src %d (%.0f vs %.0f dBm, len %u)\n", r, b->src, pi, p, b->len); }
            }
            if (lost) continue;
            if (DBG(a)) fprintf(stderr, "    rx %d: ok %.0f dBm\n", r, p);
            snr = p + 117.0;                           /* noise floor BW125 + 6 dB NF */
            if (snr > 12) snr = 12;
            bp_node_on_rx(&s->node, a->buf, a->len, (int16_t)p, (int16_t)(snr * 10));
        }
        for (r = 0; r < P_gateways - 1; ++r) xgw_rx(i, r);
    }
}

/* ------------------------------------------------------------ scenario */
static void to_latlon(double x, double y, double *lat, double *lon) {
    *lat = P_lat0 + y / 111320.0;
    *lon = P_lon0 + x / (111320.0 * cos(P_lat0 * M_PI / 180.0));
}

static int kind_for(int id) {
    switch (id) {
        case 1: return BP_KIND_EXIT; case 4: return BP_KIND_GAS; case 7: return BP_KIND_VICTIM;
        case 10: return BP_KIND_RADIATION; case 12: return BP_KIND_OBSTRUCTION;
        default: return BP_KIND_WAYPOINT;
    }
}
static int severity_for(int k) {
    switch (k) {
        case BP_KIND_VICTIM: return 15; case BP_KIND_GAS: return 60; case BP_KIND_RADIATION: return 70;
        case BP_KIND_OBSTRUCTION: return 40; default: return 0;
    }
}

static void build_world(void) {
    /* corridor: E x4, N x4, E x3, S x3, then E ... */
    static const int seglen[] = {4, 4, 3, 3, 99};
    static const double segdir[] = {0, 90, 0, -90, 0};
    double x = 2.0, y = 0.0, path = 2.0;
    int i, seg = 0, inseg = 0, j;
    bp_config_t c;

    NN = 0;
    /* gateway at the entrance */
    memset(&NODE[0], 0, sizeof NODE[0]);
    NODE[0].used = 1; NODE[0].is_gw = 1; NODE[0].alive = 1; NODE[0].deployed = P_gw_late <= 0;
    NODE[0].x = -4.0; NODE[0].y = 0.0; NODE[0].seg = 0;
    NN = 1;

    for (i = 1; i <= P_beacons; ++i) {
        simnode_t *s = &NODE[NN++];
        double lat, lon, step;
        memset(s, 0, sizeof *s);
        s->used = 1; s->alive = 1;
        if (i > 1) {
            if (inseg >= seglen[seg]) { seg++; inseg = 0; }
            step = 8.5 + (ur() - 0.5) * 2.0;
            x += step * cos(segdir[seg] * M_PI / 180.0);
            y += step * sin(segdir[seg] * M_PI / 180.0);
            path += step;
        }
        inseg++;
        s->x = x; s->y = y; s->seg = seg;
        s->deploy_ms = (uint32_t)(i * 45000);
        s->clock_err_s = (int)(xr() % 5) - 2;
        to_latlon(x, y, &lat, &lon);

        memset(&s->own, 0, sizeof s->own);
        s->own.origin_id = (uint16_t)i;
        s->own.seq = 1;
        s->own.kind = (uint8_t)kind_for(i);
        s->own.severity = (uint8_t)severity_for(s->own.kind);
        s->own.confidence = s->own.kind == BP_KIND_WAYPOINT ? 255 : (uint8_t)(200 + xr() % 55);
        s->own.lat_e7 = bp_deg_to_e7(lat);
        s->own.lon_e7 = bp_deg_to_e7(lon);
        s->own.err_dm = (uint8_t)((0.3 + 0.03 * path) * 10 + 0.5);    /* SLAM drift grows with path */
        s->own.flags = 0;
        s->own.timestamp = 0;                                            /* set at deploy */
        s->own.ttl_10s = bp_seconds_to_10s(bp_default_ttl_s(s->own.kind));
        s->own.half_life_10s = bp_seconds_to_10s(bp_default_half_life_s(s->own.kind));
        if (i > 1) {   /* next hop = previous beacon = way out */
            simnode_t *p = &NODE[NN - 2];
            double dx = p->x - x, dy = p->y - y;
            double brg = fmod(90.0 - atan2(dy, dx) * 180.0 / M_PI + 360.0, 360.0);  /* compass */
            s->own.flags |= BP_RF_HAS_NEXT | BP_RF_HAS_BEARING;
            s->own.next_id = (uint16_t)(i - 1);
            s->own.next_dist_dm = (uint16_t)(sqrt(dx * dx + dy * dy) * 10 + 0.5);
            s->own.next_bearing = bp_bearing_to_u8(brg);
        }
    }

    for (i = 0; i <= MAXN; ++i)
        for (j = 0; j <= i && j < MAXN + 1; ++j) shadow[i][j] = shadow[j][i] = gauss(P_shadow);

    /* v9: the ONA's other gateways (only when asked: the default scenario is unchanged) */
    for (i = 0; i < P_gateways - 1; ++i) {
        simnode_t *s = &XGW[i];
        bp_hal_t h = {hal_now, hal_tx, hal_rand, hal_event, s};
        memset(s, 0, sizeof *s);
        s->used = 1; s->is_gw = 1; s->alive = 1; s->deployed = 1;
        s->x = XGW_POS[i][0]; s->y = XGW_POS[i][1]; s->seg = 0;
        for (j = 0; j <= MAXN; ++j) xshadow[i][j] = gauss(P_shadow);
        bp_config_defaults(&c);
        memcpy(c.key, NET_KEY, BP_KEY_LEN);
        c.my_id = (uint16_t)(GW_ID + 1 + i);
        c.lora_sf = (uint8_t)P_sf;
        c.round_ms = (uint32_t)P_round_s * 1000u;
        c.relay_enabled = 0;
        c.digest_period_ms = P_digest_s * 1000u;
        bp_node_init(&s->node, &c, &h);
        bp_node_set_time(&s->node, g_unix0);
    }

    /* engines */
    for (i = 0; i < NN; ++i) {
        simnode_t *s = &NODE[i];
        bp_hal_t h = {hal_now, hal_tx, hal_rand, hal_event, s};
        bp_config_defaults(&c);
        memcpy(c.key, NET_KEY, BP_KEY_LEN);
        c.my_id = (uint16_t)(s->is_gw ? GW_ID : i);
        c.lora_sf = (uint8_t)P_sf;          /* engine derives frame + digest airtime */
        c.round_ms = (uint32_t)P_round_s * 1000u;   /* 0 -> auto */
        if (s->is_gw) { c.relay_enabled = 0; c.digest_period_ms = P_digest_s * 1000u; }
        bp_node_init(&s->node, &c, &h);
        if (s->is_gw) bp_node_set_time(&s->node, g_unix0);
    }
}

static void deploy(simnode_t *s) {
    int j;
    s->deployed = 1;
    bp_node_set_time(&s->node, (uint32_t)((int)(g_unix0 + g_now / 1000) + s->clock_err_s));
    s->own.timestamp = bp_node_unix(&s->node);
    bp_node_set_own(&s->node, &s->own);
    /* the Writer primes the new beacon with the route so far (what IT knows:
     * the records it provisioned - it does not know about later losses)   */
    if (P_seed_route)
        for (j = 1; j < NN; ++j)
            if (NODE + j != s && NODE[j].deployed) bp_node_seed(&s->node, &NODE[j].own);
    if (!P_quiet) fprintf(stderr, "[%6.1f min] Writer drops beacon %2d (%s)\n", g_now / 60000.0,
                          s->own.origin_id, bp_kind_name(s->own.kind));
}

static void attacker_step(int k) {
    /* attacker sits near beacon 8 */
    int ai = idx_of(8) >= 0 ? idx_of(8) : 1;
    double x = NODE[ai].x + 3, y = NODE[ai].y + 2;
    uint8_t buf[BP_FRAME_LEN];
    switch (k % 4) {
        case 0: {                         /* forged record with a guessed key */
            uint8_t badkey[BP_KEY_LEN];
            bp_frame_t f;
            int i;
            for (i = 0; i < BP_KEY_LEN; ++i) badkey[i] = (uint8_t)xr();
            memset(&f, 0, sizeof f);
            f.net_id = 0x2A; f.relay_id = ATTACK_FAKE_ID; f.hop_limit = 4;
            f.rec = NODE[ai].own; f.rec.origin_id = ATTACK_FAKE_ID; f.rec.kind = BP_KIND_VICTIM;
            f.rec.severity = 100; f.rec.timestamp = g_unix0 + g_now / 1000;
            bp_encode(&f, badkey, buf);
            attack_forged++;
            break;
        }
        case 1:                           /* tampered copy of a real frame */
            if (!have_capture) return;
            memcpy(buf, last_capture, BP_FRAME_LEN);
            buf[BP_HDR_LEN + 15] ^= 0x40;   /* flip a severity bit */
            attack_tampered++;
            break;
        case 2:                           /* verbatim replay */
            if (!have_capture) return;
            memcpy(buf, last_capture, BP_FRAME_LEN);
            attack_replayed++;
            break;
        default:                          /* hop-limit inflation (unsigned header) */
            if (!have_capture) return;
            memcpy(buf, last_capture, BP_FRAME_LEN);
            buf[4] = (uint8_t)((buf[4] & 0xF0) | 0x0F);
            attack_inflated++;
            break;
    }
    air_push(-1, x, y, NODE[ai].seg, buf, BP_FRAME_LEN);
}

/* ------------------------------------------------------------ pacing */
/* monotonic wall clock in seconds + sleep: POSIX and native Windows */
static double wall_s(void) {
#ifdef _WIN32
    static LARGE_INTEGER f;
    LARGE_INTEGER c;
    if (!f.QuadPart) QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    return (double)c.QuadPart / (double)f.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
#endif
}

static void sleep_s(double d) {
#ifdef _WIN32
    Sleep((DWORD)(d * 1000.0));
#else
    struct timespec ts;
    ts.tv_sec = (time_t)d;
    ts.tv_nsec = (long)((d - (double)ts.tv_sec) * 1e9);
    nanosleep(&ts, NULL);
#endif
}

static void pace(double t0_wall) {
    double now, target;
    if (P_speed <= 0) return;
    now = wall_s();
    target = t0_wall + (g_now / 1000.0) / P_speed;
    if (target > now + 0.002) sleep_s(target - now);
}

/* ------------------------------------------------------------ report */
static int holders_of(uint16_t id) {
    int i, c = 0;
    for (i = 0; i < NN; ++i) {
        const bp_entry_t *e;
        if (!NODE[i].alive || !NODE[i].deployed) continue;
        e = bp_node_find(&NODE[i].node, id);
        if (e && !(NODE[i].node.cfg.my_id == id)) c++;
    }
    return c;
}

/* latency origin: the drop, or the gateway's power-on if it came later */
static uint32_t since(const simnode_t *s) {
    uint32_t on = (uint32_t)(P_gw_late * 60000.0);
    return s->deploy_ms > on ? s->deploy_ms : on;
}

static uint32_t sum_sync(void) {
    uint32_t c = 0;
    int i;
    for (i = 1; i < NN; ++i) c += NODE[i].node.st.tx_sync;
    return c;
}

/* ETSI EN 300 220 definition: airtime in the worst rolling 60-minute window,
 * as a fraction of one hour (permille). */
static uint32_t duty_rolling_hour_permille(const simnode_t *s) {
    int a = 0, b;
    uint64_t sum = 0, best = 0;
    for (b = 0; b < s->tx_n; ++b) {
        sum += s->tx_air[b];
        while (s->tx_log[b] - s->tx_log[a] >= 3600000u) sum -= s->tx_air[a++];
        if (sum > best) best = sum;
    }
    return (uint32_t)(best * 1000u / 3600000u);
}

/* Which surviving nodes still have a radio path to the gateway?  A link is
 * usable when its mean received power (path loss + shadowing, no fading) is
 * above sensitivity. reach[i] = 1 if node i is connected to the gateway.   */
static int connectivity(int reach[MAXN]) {
    int q[MAXN], h = 0, t = 0, i, j, cnt = 0;
    for (i = 0; i < NN; ++i) reach[i] = 0;
    reach[0] = 1; q[t++] = 0;
    while (h < t) {
        i = q[h++];
        for (j = 0; j < NN; ++j) {
            if (reach[j] || !NODE[j].alive || !NODE[j].deployed) continue;
            if (rx_power(NODE[i].x, NODE[i].y, NODE[i].seg, i, j) >= sens_dbm) { reach[j] = 1; q[t++] = j; }
        }
    }
    for (i = 1; i < NN; ++i) cnt += (NODE[i].alive && NODE[i].deployed && !reach[i]);
    return cnt;   /* surviving nodes cut off from the gateway */
}

static int check(int ok, const char *what) {
    fprintf(stderr, "  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    return ok;
}

static void usage(void) {
    fprintf(stderr,
            "bpsim - Living Map beacon network simulator\n"
            "  --beacons N      beacons in the chain (default 14, max %d)\n"
            "  --minutes M      simulated duration (default 60)\n"
            "  --speed S        0 = max speed, 1 = real time, 10 = 10x (default 0)\n"
            "  --sf 7..12       LoRa spreading factor (default 9)\n"
            "  --round S        gossip round seconds (default 0 = auto from SF)\n"
            "  --digest S       gateway digest (pull) period, 0 = off (default 90)\n"
            "  --kill ID@MIN    destroy beacon ID at minute MIN (default 6@25, 0@0 = none)\n"
            "  --no-attacker    disable forged/tampered/replayed frames\n"
            "  --no-seed        don't prime new beacons with the route (Writer PROV SEED)\n"
            "  --gw-late MIN    gateway powered on late: everything must be pulled\n"
            "  --rubble DB      extra attenuation per metre through debris (default 1.2)\n"
            "  --gateways N     1..3 gateways (the ONA's three: gw2, gw3 near the entrance)\n"
            "  --seed N         random seed (default 7)\n"
            "  --quiet          no timeline on stderr\n"
            "stdout: gateway JSON lines (pipe into gateway_bridge.py --stdin)\n",
            MAXN - 2);
}

int main(int argc, char **argv) {
    static const double SENS[13] = {0, 0, 0, 0, 0, 0, 0, -123, -126, -129, -132, -134.5, -137};
    int i, fails = 0, attack_k = 0;
    uint32_t end_ms, next_hb = 0, next_attack = 5 * 60000;
    double t0_wall;

    for (i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--beacons") && i + 1 < argc) P_beacons = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--minutes") && i + 1 < argc) P_minutes = atof(argv[++i]);
        else if (!strcmp(argv[i], "--speed") && i + 1 < argc) P_speed = atof(argv[++i]);
        else if (!strcmp(argv[i], "--sf") && i + 1 < argc) P_sf = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--round") && i + 1 < argc) P_round_s = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--digest") && i + 1 < argc) P_digest_s = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--seed") && i + 1 < argc) P_seed = (uint32_t)atol(argv[++i]);
        else if (!strcmp(argv[i], "--kill") && i + 1 < argc) sscanf(argv[++i], "%d@%lf", &P_kill_id, &P_kill_min);
        else if (!strcmp(argv[i], "--no-attacker")) P_attacker = 0;
        else if (!strcmp(argv[i], "--no-seed")) P_seed_route = 0;
        else if (!strcmp(argv[i], "--gw-late") && i + 1 < argc) P_gw_late = atof(argv[++i]);
        else if (!strcmp(argv[i], "--rubble") && i + 1 < argc) P_rubble = atof(argv[++i]);
        else if (!strcmp(argv[i], "--quiet")) P_quiet = 1;
        else if (!strcmp(argv[i], "--gateways") && i + 1 < argc) P_gateways = atoi(argv[++i]);
        else { usage(); return 2; }
    }
    if (P_beacons < 2 || P_beacons > MAXN - 2 || P_sf < 7 || P_sf > 12 || P_gateways < 1 || P_gateways > MAXGW) {
        usage();
        return 2;
    }

    rng_s ^= (uint64_t)P_seed * 0x9E3779B97F4A7C15ULL;
    if (!rng_s) rng_s = 1;
    airtime_ms = bp_lora_airtime_ms(BP_FRAME_LEN, (uint8_t)P_sf, 125000, 5, 8, 1, 0);
    sens_dbm = SENS[P_sf] + 3.0;                   /* 3 dB implementation margin */
    g_unix0 = (uint32_t)time(NULL);
    if (getenv("BPSIM_TXLOG")) g_txlog = atoi(getenv("BPSIM_TXLOG"));
    build_world();

    fprintf(stderr, "bpsim: %d beacons, SF%d, frame %d B, airtime %u ms, sensitivity %.1f dBm, round %.1f s, digest %d s\n",
            P_beacons, P_sf, BP_FRAME_LEN, (unsigned)airtime_ms, sens_dbm, NODE[1].node.cfg.round_ms / 1000.0, P_digest_s);

    t0_wall = wall_s();
    end_ms = (uint32_t)(P_minutes * 60000.0);

    for (g_now = 0; g_now <= end_ms; g_now += 10) {
        for (i = 1; i < NN; ++i) {
            simnode_t *s = &NODE[i];
            if (!s->deployed && g_now >= s->deploy_ms) deploy(s);
        }
        if (!NODE[0].deployed && g_now >= (uint32_t)(P_gw_late * 60000.0)) {
            NODE[0].deployed = 1;          /* command post comes online late: must PULL */
            bp_node_set_time(&NODE[0].node, (uint32_t)(g_unix0 + g_now / 1000));
            if (!P_quiet) fprintf(stderr, "[%6.1f min] gateway comes online (knows nothing yet)\n", g_now / 60000.0);
        }
        for (i = 1; i < NN; ++i) {
            simnode_t *s = &NODE[i];
            if (s->alive && s->own.origin_id == P_kill_id && P_kill_min > 0 && g_now >= (uint32_t)(P_kill_min * 60000)) {
                s->alive = 0;
                s->kill_ms = g_now;
                if (!P_quiet) fprintf(stderr, "[%6.1f min] *** beacon %d destroyed ***\n", g_now / 60000.0, P_kill_id);
                if (getenv("BPSIM_DEBUG_LOST")) {
                    int q;
                    for (q = 0; q < NN; ++q) {
                        const bp_entry_t *ke = bp_node_find(&NODE[q].node, (uint16_t)P_kill_id);
                        if (ke && ke->heard_direct)
                            fprintf(stderr, "    at kill: node %d sees #%d any %04x rssi %d n %u tx_own_of_victim %u\n", q, P_kill_id,
                                    (unsigned)ke->any_rounds, (int)ke->link_rssi, (unsigned)ke->direct_n,
                                    (unsigned)s->node.st.tx_own);
                    }
                }
            }
        }
        if (g_now == 30u * 60000u && idx_of(7) >= 0 && NODE[idx_of(7)].alive && P_beacons >= 7) {
            simnode_t *v = &NODE[idx_of(7)];
            v->own.seq = 2;
            v->own.severity = 40;
            v->own.timestamp = bp_node_unix(&v->node);
            bp_node_set_own(&v->node, &v->own);
            victim_update_sent = g_now;
            if (!P_quiet) fprintf(stderr, "[%6.1f min] victim record updated (seq 2, severity 40)\n", g_now / 60000.0);
        }
        if (P_attacker && g_now >= next_attack) {
            attacker_step(attack_k++);
            next_attack = g_now + 90000;
        }
        deliver();
        for (i = 0; i < NN; ++i)
            if (NODE[i].alive && NODE[i].deployed) bp_node_tick(&NODE[i].node);
        for (i = 0; i < P_gateways - 1; ++i) bp_node_tick(&XGW[i].node);
        if (g_now >= next_hb) {
            int k;
            for (k = 0; k < P_gateways; ++k) {
                simnode_t *gs = k == 0 ? &NODE[0] : &XGW[k - 1];
                bp_node_t *gw = &gs->node;
                printf("{\"ev\":\"hb\",\"gw\":\"%s\",\"ts\":%lu,\"rx_ok\":%lu,\"rx_bad\":%lu,\"known\":%d,"
                       "\"digests\":%lu,\"duty\":%.4f,\"sim_speed\":%g}\n", gw_name(gs),
                       (unsigned long)bp_node_unix(gw), (unsigned long)gw->st.rx_ok,
                       (unsigned long)(gw->st.rx_bad_mac + gw->st.rx_bad_other), bp_node_count(gw),
                       (unsigned long)gw->st.tx_digest, (double)bp_node_duty_used(gw), P_speed);
            }
            fflush(stdout);
            next_hb = g_now + 4000;
        }
        if (g_now % 100 == 0) pace(t0_wall);
    }
    g_now = end_ms;

    /* ---------------------------------------------------------------- report */
    {
        bp_node_t *gw = &NODE[0].node;
        uint32_t bad_mac = 0, max_duty_permille = 0, forged_accepted = 0, future = 0;
        int all_known = 1, chain_ok = 1, hops_max = 0, victim_cut = 0;
        uint16_t cur = 7;
        int guard = 0;

        fprintf(stderr, "\n=========================== REPORT ===========================\n");
        fprintf(stderr, " id kind         dropped  ->gw(s) hops  tx own/rel/ae/syn   duty%%  cache  replicas\n");
        fprintf(stderr, "                          latency  @gw                      /hour\n");
        for (i = 1; i < NN; ++i) {
            simnode_t *s = &NODE[i];
            const bp_entry_t *ge = bp_node_find(gw, s->own.origin_id);
            uint32_t duty = duty_rolling_hour_permille(s);
            if (duty > max_duty_permille) max_duty_permille = duty;
            bad_mac += s->node.st.rx_bad_mac;
            future += s->node.st.rx_future;
            if (bp_node_find(&s->node, ATTACK_FAKE_ID)) forged_accepted++;
            if (!gw_first[i]) all_known = 0;
            if (gw_first[i] && gw_first_hops[i] > hops_max) hops_max = gw_first_hops[i];
            {
                char lat_s[16], hop_s[8];
                if (gw_first[i]) snprintf(lat_s, sizeof lat_s, "%.1f", (gw_first[i] - since(s)) / 1000.0);
                else snprintf(lat_s, sizeof lat_s, "never");
                if (ge) snprintf(hop_s, sizeof hop_s, "%u", (unsigned)ge->hops);
                else snprintf(hop_s, sizeof hop_s, "-");
                fprintf(stderr, " %2d %-12s %5.1fm %8s %4s  %4u/%3u/%3u/%3u  %5.2f  %5d  %8d%s\n",
                        s->own.origin_id, bp_kind_name(s->own.kind), s->deploy_ms / 60000.0, lat_s, hop_s,
                        (unsigned)s->node.st.tx_own, (unsigned)s->node.st.tx_relay,
                        (unsigned)s->node.st.tx_ae, (unsigned)s->node.st.tx_sync, duty / 10.0, bp_node_count(&s->node),
                        holders_of(s->own.origin_id), s->alive ? "" : "   <- DESTROYED");
            }
        }
        bad_mac += gw->st.rx_bad_mac;
        if (bp_node_find(gw, ATTACK_FAKE_ID)) forged_accepted++;
        {
            uint32_t gd = duty_rolling_hour_permille(&NODE[0]);
            if (gd > max_duty_permille) max_duty_permille = gd;
            fprintf(stderr, " gw GATEWAY     digests sent %u, duty %.2f %%/hour, sync pushes answered %u\n",
                    (unsigned)gw->st.tx_digest, gd / 10.0, (unsigned)sum_sync());
        }

        {   /* delivery latency distribution, drop -> gateway */
            double lat[MAXN];
            int nl = 0, a, b;
            for (i = 1; i < NN; ++i)
                if (gw_first[i]) lat[nl++] = (gw_first[i] - since(&NODE[i])) / 1000.0;
            for (a = 1; a < nl; ++a)
                for (b = a; b > 0 && lat[b - 1] > lat[b]; --b) { double t = lat[b]; lat[b] = lat[b - 1]; lat[b - 1] = t; }
            if (nl)
                fprintf(stderr, "\n delivery latency drop->gateway: median %.1f s, p90 %.1f s, max %.1f s\n",
                        lat[nl / 2], lat[(nl * 9) / 10 < nl ? (nl * 9) / 10 : nl - 1], lat[nl - 1]);
        }
        fprintf(stderr, " gateway: rx_ok %lu, dup %lu, stale/replay %lu, bad MAC %lu, known records %d\n",
                (unsigned long)gw->st.rx_ok, (unsigned long)gw->st.rx_dup, (unsigned long)gw->st.rx_stale,
                (unsigned long)gw->st.rx_bad_mac, bp_node_count(gw));
        fprintf(stderr, " attacker: forged %u, tampered %u, replayed %u, hop-inflated %u\n",
                (unsigned)attack_forged, (unsigned)attack_tampered, (unsigned)attack_replayed,
                (unsigned)attack_inflated);

        /* chain from the victim back to the exit, using ONLY the gateway's cache */
        fprintf(stderr, " route victim->exit from gateway cache: ");
        while (guard++ < 64) {
            const bp_entry_t *e = bp_node_find(gw, cur);
            if (!e) { chain_ok = 0; fprintf(stderr, "[%u missing]", (unsigned)cur); break; }
            fprintf(stderr, "%u%s", (unsigned)cur, e->reported_silent ? "(lost)" : "");
            if (!(e->rec.flags & BP_RF_HAS_NEXT)) break;
            fprintf(stderr, " -> ");
            cur = e->rec.next_id;
        }
        if (getenv("BPSIM_TRACE")) {   /* debug: who holds which version of an origin + link margins */
            int o = atoi(getenv("BPSIM_TRACE")), a2, b2;
            fprintf(stderr, "\n TRACE origin %d:", o);
            for (a2 = 0; a2 < NN; ++a2) {
                const bp_entry_t *te = bp_node_find(&NODE[a2].node, (uint16_t)o);
                fprintf(stderr, " [%d:%s%s]", a2, te ? (te->rec.seq == 2 ? "v2" : "v1") : "--",
                        NODE[a2].alive ? "" : "X");
            }
            fprintf(stderr, "\n acquired (min after drop):");
            for (a2 = 0; a2 < NN; ++a2)
                if (o < MAXN && got_at[a2][o] && idx_of((uint16_t)o) >= 0)
                    fprintf(stderr, " %d@%.1f", a2, ((double)got_at[a2][o] - NODE[idx_of((uint16_t)o)].deploy_ms) / 60000.0);
            fprintf(stderr, "\n link margins (dB above sensitivity, alive nodes, >=0 only):\n");
            for (a2 = 0; a2 < NN; ++a2) {
                if (!NODE[a2].alive) continue;
                fprintf(stderr, "  %2d:", a2);
                for (b2 = 0; b2 < NN; ++b2) {
                    double m;
                    if (b2 == a2 || !NODE[b2].alive) continue;
                    m = rx_power(NODE[a2].x, NODE[a2].y, NODE[a2].seg, a2, b2) - sens_dbm;
                    if (m >= 0) fprintf(stderr, " %d(%+.0f)", b2, m);
                }
                fprintf(stderr, "\n");
            }
        }
        {
            int reach[MAXN], cut = connectivity(reach), k;
            if (cut) {
                fprintf(stderr, "\n\n NETWORK PARTITIONED by beacon loss: %d surviving node(s) have no radio path to the gateway:", cut);
                for (k = 1; k < NN; ++k)
                    if (NODE[k].alive && NODE[k].deployed && !reach[k]) fprintf(stderr, " %d", k);
                fprintf(stderr, "\n  -> records made BEFORE the loss survive (replicated); NEW information from that side\n"
                                "     needs a replacement beacon or the Executor acting as a data mule.");
            }
            victim_cut = P_beacons >= 7 && idx_of(7) >= 0 && !reach[idx_of(7)];
        }
        fprintf(stderr, "\n\n CHECKS\n");

        fails += !check(all_known, "every beacon's record reached the gateway (latency column)");
        fails += !check(hops_max >= 1, "far records arrived by multi-hop relaying (hops on first arrival >= 1)");
        fails += !check(forged_accepted == 0, "zero forged records accepted anywhere");
        fails += !check(!P_attacker || bad_mac > 0, "forged/tampered frames were rejected by HMAC");
        if (P_kill_id > 0 && P_kill_min > 0 && P_kill_id <= P_beacons && P_kill_min < P_minutes) {
            int ki = idx_of((uint16_t)P_kill_id);
            char msg[160];
            fails += !check(bp_node_find(gw, (uint16_t)P_kill_id) != NULL,
                            "destroyed beacon's record still held by the gateway");
            snprintf(msg, sizeof msg, "destroyed beacon's record replicated on %d surviving nodes (>= 2)",
                     holders_of((uint16_t)P_kill_id));
            fails += !check(holders_of((uint16_t)P_kill_id) >= 2, msg);
            if (ki >= 0 && gw_silent_at[ki]) {
                snprintf(msg, sizeof msg, "gateway flagged beacon %d as lost %.1f min after destruction",
                         P_kill_id, (gw_silent_at[ki] - NODE[ki].kill_ms) / 60000.0);
                fails += !check(1, msg);
            } else if (ki >= 0 && NODE[ki].kill_ms - NODE[ki].deploy_ms < 3u * gw->cfg.round_ms) {
                fprintf(stderr, "  [N/A ] beacon %d lived only %.1f min: loss is judged after >= 3 gossip rounds (%.0f min) of link history\n",
                        P_kill_id, (NODE[ki].kill_ms - NODE[ki].deploy_ms) / 60000.0, 3.0 * gw->cfg.round_ms / 60000.0);
            } else if (P_kill_min * 60000.0 + 1.3 * BP_SILENT_ROUNDS * gw->cfg.round_ms > P_minutes * 60000.0) {
                fprintf(stderr, "  [N/A ] loss detection needs ~%.0f min at this setting (5 gossip rounds); run is too short\n",
                        1.3 * BP_SILENT_ROUNDS * gw->cfg.round_ms / 60000.0);
            } else {
                fails += !check(0, "gateway flagged the destroyed beacon as lost");
            }
        }
        {
            char msg[128];
            snprintf(msg, sizeof msg, "no false \"beacon lost\" alarms at the gateway (%u)", (unsigned)false_lost);
            fails += !check(false_lost == 0, msg);
        }
        if (P_beacons >= 7) {
            char msg[128];
            fails += !check(chain_ok, "full next-hop route victim -> exit known at the gateway");
            if (victim_update_at) {
                snprintf(msg, sizeof msg, "victim update (seq 2) reached the gateway in %.1f s",
                         (victim_update_at - victim_update_sent) / 1000.0);
                fails += !check(1, msg);
            } else if (P_minutes > 30 && victim_cut) {
                fprintf(stderr, "  [N/A ] victim update could not reach the gateway: victim is in a partition cut off by the beacon loss\n");
            } else if (P_minutes > 30) {
                fails += !check(0, "victim update (seq 2) reached the gateway");
            }
        }
        if (P_beacons >= 12 && P_minutes >= 50) {   /* adaptive aging, per kind */
            const bp_entry_t *gas = bp_node_find(gw, 4), *obs = bp_node_find(gw, 12);
            uint32_t un = bp_node_unix(gw);
            char msg[160];
            snprintf(msg, sizeof msg, "GAS event (10 min half-life) aged out -> kept as stale route marker%s",
                     gas && (gas->rec.flags & BP_RF_STALE) && gas->rec.kind == BP_KIND_GAS ? "" : " [not seen]");
            fails += !check(gas && (gas->rec.flags & BP_RF_STALE) && gas->rec.kind == BP_KIND_GAS, msg);
            snprintf(msg, sizeof msg, "OBSTRUCTION (48 h half-life) still confident at end: %.2f",
                     obs ? (double)bp_effective_confidence(&obs->rec, un) : 0.0);
            fails += !check(obs && bp_effective_confidence(&obs->rec, un) > 0.7f, msg);
        }
        {
            char msg[96];
            snprintf(msg, sizeof msg, "worst rolling-hour duty cycle %.2f %% (<= 1 %%, ETSI EN 300 220)",
                     max_duty_permille / 10.0);
            fails += !check(max_duty_permille <= 10, msg);
        }
        fprintf(stderr, "==============================================================\n");
        fprintf(stderr, "%s\n", fails ? "RESULT: FAIL" : "RESULT: ALL CHECKS PASSED");
    }
    return fails ? 1 : 0;
}
