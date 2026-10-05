/*
 * bp_sha256.h - compact, dependency-free SHA-256 and HMAC-SHA256.
 *
 * Portable C99 so the exact same code runs on the ESP32 beacons, the gateway
 * and the host tools/simulator (no mbedTLS / OpenSSL dependency -> identical
 * results everywhere, verified against Python's hashlib in the test-suite).
 */
#ifndef BP_SHA256_H
#define BP_SHA256_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BP_SHA256_LEN 32

typedef struct {
    uint32_t state[8];
    uint64_t bitlen;
    uint8_t  data[64];
    uint32_t datalen;
} bp_sha256_ctx;

void bp_sha256_init(bp_sha256_ctx *c);
void bp_sha256_update(bp_sha256_ctx *c, const uint8_t *data, size_t len);
void bp_sha256_final(bp_sha256_ctx *c, uint8_t out[BP_SHA256_LEN]);

/* HMAC-SHA256 (RFC 2104). key may be any length. */
void bp_hmac_sha256(const uint8_t *key, size_t key_len,
                    const uint8_t *msg, size_t msg_len,
                    uint8_t out[BP_SHA256_LEN]);

/* Constant-time comparison (avoids MAC timing leaks). Returns 1 if equal. */
int bp_ct_equal(const uint8_t *a, const uint8_t *b, size_t n);

#ifdef __cplusplus
}
#endif
#endif /* BP_SHA256_H */
