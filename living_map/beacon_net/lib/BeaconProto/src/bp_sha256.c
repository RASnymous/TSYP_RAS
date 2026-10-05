/* bp_sha256.c - SHA-256 (FIPS 180-4) + HMAC (RFC 2104), portable C99. */
#include "bp_sha256.h"
#include <string.h>

#define ROTR(x, n) (((x) >> (n)) | ((x) << (32 - (n))))
#define CH(x, y, z) (((x) & (y)) ^ (~(x) & (z)))
#define MAJ(x, y, z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))
#define EP0(x) (ROTR(x, 2) ^ ROTR(x, 13) ^ ROTR(x, 22))
#define EP1(x) (ROTR(x, 6) ^ ROTR(x, 11) ^ ROTR(x, 25))
#define SIG0(x) (ROTR(x, 7) ^ ROTR(x, 18) ^ ((x) >> 3))
#define SIG1(x) (ROTR(x, 17) ^ ROTR(x, 19) ^ ((x) >> 10))

static const uint32_t K[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

static void transform(bp_sha256_ctx *c, const uint8_t d[64]) {
    uint32_t a, b, cc, dd, e, f, g, h, t1, t2, m[64];
    int i;
    for (i = 0; i < 16; ++i)
        m[i] = ((uint32_t)d[i * 4] << 24) | ((uint32_t)d[i * 4 + 1] << 16) |
               ((uint32_t)d[i * 4 + 2] << 8) | ((uint32_t)d[i * 4 + 3]);
    for (; i < 64; ++i) m[i] = SIG1(m[i - 2]) + m[i - 7] + SIG0(m[i - 15]) + m[i - 16];

    a = c->state[0]; b = c->state[1]; cc = c->state[2]; dd = c->state[3];
    e = c->state[4]; f = c->state[5]; g = c->state[6]; h = c->state[7];
    for (i = 0; i < 64; ++i) {
        t1 = h + EP1(e) + CH(e, f, g) + K[i] + m[i];
        t2 = EP0(a) + MAJ(a, b, cc);
        h = g; g = f; f = e; e = dd + t1;
        dd = cc; cc = b; b = a; a = t1 + t2;
    }
    c->state[0] += a; c->state[1] += b; c->state[2] += cc; c->state[3] += dd;
    c->state[4] += e; c->state[5] += f; c->state[6] += g; c->state[7] += h;
}

void bp_sha256_init(bp_sha256_ctx *c) {
    c->datalen = 0;
    c->bitlen = 0;
    c->state[0] = 0x6a09e667; c->state[1] = 0xbb67ae85; c->state[2] = 0x3c6ef372; c->state[3] = 0xa54ff53a;
    c->state[4] = 0x510e527f; c->state[5] = 0x9b05688c; c->state[6] = 0x1f83d9ab; c->state[7] = 0x5be0cd19;
}

void bp_sha256_update(bp_sha256_ctx *c, const uint8_t *data, size_t len) {
    size_t i;
    for (i = 0; i < len; ++i) {
        c->data[c->datalen++] = data[i];
        if (c->datalen == 64) {
            transform(c, c->data);
            c->bitlen += 512;
            c->datalen = 0;
        }
    }
}

void bp_sha256_final(bp_sha256_ctx *c, uint8_t out[BP_SHA256_LEN]) {
    uint32_t i = c->datalen;
    int j;
    c->data[i++] = 0x80;
    if (c->datalen < 56) {
        while (i < 56) c->data[i++] = 0x00;
    } else {
        while (i < 64) c->data[i++] = 0x00;
        transform(c, c->data);
        memset(c->data, 0, 56);
    }
    c->bitlen += (uint64_t)c->datalen * 8;
    for (j = 0; j < 8; ++j) c->data[63 - j] = (uint8_t)(c->bitlen >> (8 * j));
    transform(c, c->data);
    for (j = 0; j < 8; ++j) {
        out[j * 4]     = (uint8_t)(c->state[j] >> 24);
        out[j * 4 + 1] = (uint8_t)(c->state[j] >> 16);
        out[j * 4 + 2] = (uint8_t)(c->state[j] >> 8);
        out[j * 4 + 3] = (uint8_t)(c->state[j]);
    }
}

void bp_hmac_sha256(const uint8_t *key, size_t key_len, const uint8_t *msg, size_t msg_len,
                    uint8_t out[BP_SHA256_LEN]) {
    uint8_t k0[64], pad[64], inner[BP_SHA256_LEN];
    bp_sha256_ctx c;
    size_t i;

    memset(k0, 0, sizeof k0);
    if (key_len > 64) {
        bp_sha256_init(&c);
        bp_sha256_update(&c, key, key_len);
        bp_sha256_final(&c, k0);
    } else {
        memcpy(k0, key, key_len);
    }

    for (i = 0; i < 64; ++i) pad[i] = k0[i] ^ 0x36;
    bp_sha256_init(&c);
    bp_sha256_update(&c, pad, 64);
    bp_sha256_update(&c, msg, msg_len);
    bp_sha256_final(&c, inner);

    for (i = 0; i < 64; ++i) pad[i] = k0[i] ^ 0x5c;
    bp_sha256_init(&c);
    bp_sha256_update(&c, pad, 64);
    bp_sha256_update(&c, inner, BP_SHA256_LEN);
    bp_sha256_final(&c, out);

    memset(k0, 0, sizeof k0);   /* don't leave key material on the stack */
    memset(pad, 0, sizeof pad);
}

int bp_ct_equal(const uint8_t *a, const uint8_t *b, size_t n) {
    uint8_t diff = 0;
    size_t i;
    for (i = 0; i < n; ++i) diff |= (uint8_t)(a[i] ^ b[i]);
    return diff == 0;
}
