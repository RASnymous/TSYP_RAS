/*
 * bptool - command-line access to the beacon protocol (host / test harness).
 *
 *   bptool example                         the reference VICTIM record, JSON -> frame
 *   bptool fromjson <keyhex> <net> '<json>' [now_unix]
 *                                          mission-log JSON -> 44-byte frame (as origin)
 *   bptool hmac    <keyhex> <msghex>
 *   bptool encode  <keyhex> <net> <relay> <hops> <limit> <hflags>
 *                  <origin> <seq> <kind> <flags> <lat_e7> <lon_e7> <err_dm>
 *                  <sev> <conf> <bearing> <next_id> <next_dist_dm> <ts>
 *                  <ttl_10s> <half_life_10s>
 *   bptool digest  <keyhex> <net> <sender> <page> [origin:seq ...]
 *   bptool decode  <keyhex> <net> <framehex>     record (0x21) or digest (0x22)
 *   bptool airtime <payload_len> <sf> <bw_hz> <cr_denom>
 *
 * Used by python/tests/test_crosscheck.py to prove the C and Python
 * implementations produce identical bytes.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "beacon_proto.h"
#include "bp_json.h"
#include "bp_sha256.h"

static int hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static int unhex(const char *s, uint8_t *out, size_t max) {
    size_t n = strlen(s), i;
    if (n % 2 || n / 2 > max) return -1;
    for (i = 0; i < n / 2; ++i) {
        int hi = hexval(s[2 * i]), lo = hexval(s[2 * i + 1]);
        if (hi < 0 || lo < 0) return -1;
        out[i] = (uint8_t)(hi * 16 + lo);
    }
    return (int)(n / 2);
}

static void phex(const uint8_t *b, size_t n) {
    size_t i;
    for (i = 0; i < n; ++i) printf("%02x", b[i]);
    printf("\n");
}

static int usage(void) {
    fprintf(stderr, "usage: bptool example|fromjson|hmac|encode|digest|decode|airtime ... (see source header)\n");
    return 2;
}

int main(int argc, char **argv) {
    uint8_t key[BP_KEY_LEN];
    if (argc < 2) return usage();

    if (!strcmp(argv[1], "example")) {
        /* the reference record from the mission brief, exactly as the Writer sends it */
        static const char *in =
            "{\"beacon_id\":7,\"kind\":\"VICTIM\",\"gps\":{\"lat\":34.4312,\"lon\":8.7845,\"err_m\":2.1},"
            " \"severity\":15,\"next\":{\"id\":6,\"dist_m\":8.5},\"ttl_s\":7200}";
        static const uint8_t demo_key[BP_KEY_LEN] = {0x4c, 0x69, 0x76, 0x69, 0x6e, 0x67, 0x4d, 0x61,
                                                     0x70, 0x2d, 0x44, 0x45, 0x4d, 0x4f, 0x21, 0x21};
        bp_frame_t f;
        uint8_t buf[BP_FRAME_LEN];
        char json[512];
        memset(&f, 0, sizeof f);
        if (bp_record_from_json(in, &f.rec, 1758500000u) != BP_OK) { fprintf(stderr, "parse failed\n"); return 1; }
        f.net_id = 0x2A; f.relay_id = f.rec.origin_id; f.hops = 0; f.hop_limit = BP_HOPS_UNSCOPED;
        if (bp_encode(&f, demo_key, buf) != BP_OK) { fprintf(stderr, "encode failed\n"); return 1; }
        bp_record_to_json(&f.rec, json, sizeof json);
        printf("input  : %s\n", in);
        printf("key    : \"LivingMap-DEMO!!\" (demo only), net 0x2A, ts 1758500000\n");
        printf("json   : %s\n", json);
        printf("bytes  : %d\n", BP_FRAME_LEN);
        printf("frame  : ");
        phex(buf, BP_FRAME_LEN);
        printf("airtime: SF7 %u ms | SF9 %u ms | SF12 %u ms (BW125, CR4/5)\n",
               (unsigned)bp_lora_airtime_ms(BP_FRAME_LEN, 7, 125000, 5, 8, 1, 0),
               (unsigned)bp_lora_airtime_ms(BP_FRAME_LEN, 9, 125000, 5, 8, 1, 0),
               (unsigned)bp_lora_airtime_ms(BP_FRAME_LEN, 12, 125000, 5, 8, 1, 0));
        return 0;
    }

    if (!strcmp(argv[1], "hmac") && argc == 4) {
        static uint8_t k[256], m[4096];
        uint8_t out[BP_SHA256_LEN];
        int kl = unhex(argv[2], k, sizeof k), ml = unhex(argv[3], m, sizeof m);
        if (kl < 0 || ml < 0) return usage();
        bp_hmac_sha256(k, (size_t)kl, m, (size_t)ml, out);
        phex(out, BP_SHA256_LEN);
        return 0;
    }

    if (!strcmp(argv[1], "fromjson") && (argc == 5 || argc == 6)) {
        bp_frame_t f;
        uint8_t buf[BP_FRAME_LEN];
        char json[512];
        int st;
        if (unhex(argv[2], key, sizeof key) != BP_KEY_LEN) return usage();
        memset(&f, 0, sizeof f);
        st = bp_record_from_json(argv[4], &f.rec, argc == 6 ? (uint32_t)strtoul(argv[5], 0, 0) : 0u);
        if (st != BP_OK) { printf("{\"error\":\"%s\"}\n", bp_status_str(st)); return 1; }
        f.net_id = (uint8_t)strtoul(argv[3], 0, 0);
        f.relay_id = f.rec.origin_id; f.hops = 0; f.hop_limit = BP_HOPS_UNSCOPED;
        st = bp_encode(&f, key, buf);
        if (st != BP_OK) { printf("{\"error\":\"%s\"}\n", bp_status_str(st)); return 1; }
        bp_record_to_json(&f.rec, json, sizeof json);
        phex(buf, BP_FRAME_LEN);
        printf("%s\n", json);
        return 0;
    }

    if (!strcmp(argv[1], "digest") && argc >= 6) {
        uint16_t org[BP_DIGEST_MAX_ENTRIES], sq[BP_DIGEST_MAX_ENTRIES];
        uint8_t buf[BP_DIGEST_LEN(BP_DIGEST_MAX_ENTRIES)];
        int i, n = argc - 6, len;
        if (unhex(argv[2], key, sizeof key) != BP_KEY_LEN || n > BP_DIGEST_MAX_ENTRIES) return usage();
        for (i = 0; i < n; ++i) {
            unsigned long o, q;
            if (sscanf(argv[6 + i], "%lu:%lu", &o, &q) != 2) return usage();
            org[i] = (uint16_t)o;
            sq[i] = (uint16_t)q;
        }
        len = bp_encode_digest((uint8_t)strtoul(argv[3], 0, 0), (uint16_t)strtoul(argv[4], 0, 0), org, sq,
                               (uint8_t)n, (uint8_t)strtoul(argv[5], 0, 0), key, buf, sizeof buf);
        if (len < 0) { printf("error %s\n", bp_status_str(len)); return 1; }
        phex(buf, (size_t)len);
        return 0;
    }

    if (!strcmp(argv[1], "encode") && argc == 23) {
        bp_frame_t f;
        uint8_t buf[BP_FRAME_LEN];
        int st;
        if (unhex(argv[2], key, sizeof key) != BP_KEY_LEN) return usage();
        memset(&f, 0, sizeof f);
        f.net_id = (uint8_t)strtoul(argv[3], 0, 0);
        f.relay_id = (uint16_t)strtoul(argv[4], 0, 0);
        f.hops = (uint8_t)strtoul(argv[5], 0, 0);
        f.hop_limit = (uint8_t)strtoul(argv[6], 0, 0);
        f.hdr_flags = (uint8_t)strtoul(argv[7], 0, 0);
        f.rec.origin_id = (uint16_t)strtoul(argv[8], 0, 0);
        f.rec.seq = (uint16_t)strtoul(argv[9], 0, 0);
        f.rec.kind = (uint8_t)strtoul(argv[10], 0, 0);
        f.rec.flags = (uint8_t)strtoul(argv[11], 0, 0);
        f.rec.lat_e7 = (int32_t)strtol(argv[12], 0, 0);
        f.rec.lon_e7 = (int32_t)strtol(argv[13], 0, 0);
        f.rec.err_dm = (uint8_t)strtoul(argv[14], 0, 0);
        f.rec.severity = (uint8_t)strtoul(argv[15], 0, 0);
        f.rec.confidence = (uint8_t)strtoul(argv[16], 0, 0);
        f.rec.next_bearing = (uint8_t)strtoul(argv[17], 0, 0);
        f.rec.next_id = (uint16_t)strtoul(argv[18], 0, 0);
        f.rec.next_dist_dm = (uint16_t)strtoul(argv[19], 0, 0);
        f.rec.timestamp = (uint32_t)strtoul(argv[20], 0, 0);
        f.rec.ttl_10s = (uint16_t)strtoul(argv[21], 0, 0);
        f.rec.half_life_10s = (uint16_t)strtoul(argv[22], 0, 0);
        st = bp_encode(&f, key, buf);
        if (st != BP_OK) { printf("error %s\n", bp_status_str(st)); return 1; }
        phex(buf, BP_FRAME_LEN);
        return 0;
    }

    if (!strcmp(argv[1], "decode") && argc == 5) {
        uint8_t buf[256];
        bp_frame_t f;
        char json[512];
        int n, st;
        if (unhex(argv[2], key, sizeof key) != BP_KEY_LEN) return usage();
        n = unhex(argv[4], buf, sizeof buf);
        if (n < 0) return usage();
        if (n > 0 && buf[0] == BP_VER_DIGEST) {
            uint16_t sender, org[BP_DIGEST_MAX_ENTRIES], sq[BP_DIGEST_MAX_ENTRIES];
            uint8_t cnt = 0;
            int i;
            st = bp_decode_digest(buf, (size_t)n, key, (uint8_t)strtoul(argv[3], 0, 0), &sender, org, sq,
                                  &cnt, BP_DIGEST_MAX_ENTRIES);
            if (st != BP_OK) { printf("{\"error\":\"%s\"}\n", bp_status_str(st)); return 1; }
            printf("{\"digest\":true,\"sender\":%u,\"page\":%u,\"final\":%s,\"entries\":[",
                   (unsigned)sender, (unsigned)(buf[5] & BP_DIGEST_PAGE_MASK),
                   (buf[5] & BP_DIGEST_FINAL) ? "true" : "false");
            for (i = 0; i < cnt; ++i) printf("%s[%u,%u]", i ? "," : "", (unsigned)org[i], (unsigned)sq[i]);
            printf("]}\n");
            return 0;
        }
        st = bp_decode(buf, (size_t)n, key, (uint8_t)strtoul(argv[3], 0, 0), &f);
        if (st != BP_OK) { printf("{\"error\":\"%s\"}\n", bp_status_str(st)); return 1; }
        bp_record_to_json(&f.rec, json, sizeof json);
        printf("{\"relay\":%u,\"hops\":%u,\"limit\":%u,\"hflags\":%u,\"record\":%s}\n",
               (unsigned)f.relay_id, (unsigned)f.hops, (unsigned)f.hop_limit, (unsigned)f.hdr_flags, json);
        return 0;
    }

    if (!strcmp(argv[1], "airtime") && argc == 6) {
        printf("%u\n", (unsigned)bp_lora_airtime_ms((uint8_t)atoi(argv[2]), (uint8_t)atoi(argv[3]),
                                                    (uint32_t)atol(argv[4]), (uint8_t)atoi(argv[5]), 8, 1, 0));
        return 0;
    }
    return usage();
}
