/*
 * ARM NVIDIA X1 emulation.
 *
 * Copyright (c) yellows8
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, see <http://www.gnu.org/licenses/>.
 */

// Based on tegra2 device code by digetx.
// This uses various definitions from: https://github.com/Atmosphere-NX/Atmosphere/blob/master/libraries/libexosphere/source/se/se_registers.hpp

#include <gcrypt.h>

#include "tegra_common.h"

#include "hw/sysbus.h"

#include "crypto/secret_common.h"
#include "crypto/cipher.h"
#include "crypto/akcipher.h"
#include "crypto/akcipherpriv.h"
#include "crypto/cipherpriv.h"
#include "crypto/aes.h"
#include "crypto/hash.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"
#include "qemu/bswap.h"
#include "qemu/bitmap.h"
#include "qapi/qapi-commands.h"
#include "qapi/visitor.h"

#include "exec/address-spaces.h"
#include "sysemu/dma.h"

#include "iomap.h"
#include "tegra_trace.h"
#include "devices.h"

#include "qemu/cutils.h"
#include "qemu/log.h"

#include "se.h"

#include "../apb/pmc/pmc.h"
#include "../ppsb/apb_misc/apb_misc.h"

#define TYPE_TEGRA_SE "tegra.se"
#define TEGRA_SE(obj) OBJECT_CHECK(tegra_se, (obj), TYPE_TEGRA_SE)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

//#define TEGRA_SE_RSA_DEBUG

typedef uint32_t u32;

typedef struct {
    u32 zero;
    u32 address;
    u32 size;
} LinkedListEntry;

typedef struct {
    u32 rand[0x4];
    u32 sticky_bits[0x8];

    u32 aes_keytable[0x10*0x10];
    u32 rsa_keytable[0x100];

    u32 fixed_pattern[0x10>>2];
} tegra_se_context;

typedef struct tegra_se_state {
    SysBusDevice parent_obj;

    qemu_irq irq;
    MemoryRegion iomem;
    MemoryRegion iomem_pka;
    uint32_t engine;

    union {
        struct SecurityEngineRegisters {
            u32 SE_SE_SECURITY;
            u32 SE_TZRAM_SECURITY;
            u32 SE_OPERATION;
            u32 SE_INT_ENABLE;
            u32 SE_INT_STATUS;
            u32 SE_CONFIG;
            u32 SE_IN_LL_ADDR;
            u32 SE_IN_CUR_BYTE_ADDR;
            u32 SE_IN_CUR_LL_ID;
            u32 SE_OUT_LL_ADDR;
            u32 SE_OUT_CUR_BYTE_ADDR;
            u32 SE_OUT_CUR_LL_ID;
            u32 SE_HASH_RESULT[0x10];
            u32 SE_CTX_SAVE_CONFIG;
            u32 SE_CTX_SAVE_AUTO;
            u32 _0x78[0x62];
            u32 SE_SHA_CONFIG;
            u32 SE_SHA_MSG_LENGTH[0x4];
            u32 SE_SHA_MSG_LEFT[0x4];
            u32 _0x224[0x17];
            u32 SE_CRYPTO_SECURITY_PERKEY;
            u32 SE_CRYPTO_KEYTABLE_ACCESS[0x10];
            u32 _0x2C4[0x10];
            u32 SE_CRYPTO_CONFIG;
            u32 SE_CRYPTO_LINEAR_CTR[0x4];
            u32 SE_CRYPTO_LAST_BLOCK;
            u32 SE_CRYPTO_KEYTABLE_ADDR;
            u32 SE_CRYPTO_KEYTABLE_DATA;
            u32 _0x324[0x3];
            u32 SE_CRYPTO_KEYTABLE_DST;
            u32 _0x334[0x3];
            u32 SE_RNG_CONFIG;
            u32 SE_RNG_SRC_CONFIG;
            u32 SE_RNG_RESEED_INTERVAL;
            u32 _0x34C[0x2D];
            u32 SE_RSA_CONFIG;
            u32 SE_RSA_KEY_SIZE;
            u32 SE_RSA_EXP_SIZE;
            u32 SE_RSA_SECURITY_PERKEY;
            u32 SE_RSA_KEYTABLE_ACCESS[0x2];
            u32 _0x418[0x2];
            u32 SE_RSA_KEYTABLE_ADDR;
            u32 SE_RSA_KEYTABLE_DATA;
            u32 SE_RSA_OUTPUT[0x40];
            u32 _0x528[0x6];
            u32 SE_TZRAM_OPERATION;
            u32 _0x544[0xAF];
            u32 SE_STATUS;
            u32 SE_ERR_STATUS;
            u32 SE_MISC;
            u32 SE_SPARE;
            u32 SE_ENTROPY_DEBUG_COUNTER;
            u32 _0x814;
            u32 _0x818;
            u32 _0x81C;
            u32 _0x820[0x5F8];
        } regs;

        u32 regs_raw[0x2000>>2];
    };

    u32 regs_pka[0x10000>>2];

    u32 aes_keytable[0x10*0x10];
    u32 rsa_keytable[0x100];
    u32 srk[0x20>>2];
    u32 srk_iv[0x10>>2];

    u32 pka_keytable[(0x800*0x4)>>2];

    u32 aes_keyslots_lock[0x10];

    u32 aes_dataoverride[0x10*0x8];
    bool aes_dataoverride_enable[0x10];
} tegra_se;

static const VMStateDescription vmstate_tegra_se = {
    .name = "tegra.se",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(engine, tegra_se),
        VMSTATE_UINT32_ARRAY(regs_raw, tegra_se, 0x2000>>2),
        VMSTATE_UINT32_ARRAY(regs_pka, tegra_se, 0x10000>>2),
        VMSTATE_UINT32_ARRAY(aes_keytable, tegra_se, 0x10*0x10),
        VMSTATE_UINT32_ARRAY(rsa_keytable, tegra_se, 0x100),
        VMSTATE_UINT32_ARRAY(srk, tegra_se, 0x20>>2),
        VMSTATE_UINT32_ARRAY(srk_iv, tegra_se, 0x10>>2),
        VMSTATE_UINT32_ARRAY(pka_keytable, tegra_se, (0x800*0x4)>>2),
        VMSTATE_UINT32_ARRAY(aes_keyslots_lock, tegra_se, 0x10),
        VMSTATE_UINT32_ARRAY(aes_dataoverride, tegra_se, 0x10*0x8),
        VMSTATE_BOOL_ARRAY(aes_dataoverride_enable, tegra_se, 0x10),
        VMSTATE_END_OF_LIST()
    }
};

static void se_log_hexdump(const char *prefix,
                           const void *bufptr, size_t size)
{
    FILE *f = qemu_log_trylock();
    if (f) {
        qemu_hexdump(f, prefix, bufptr, size);
        qemu_log_unlock(f);
    }
}

// RSA code is copied from / based on akcipher-gcrypt.c.inc. The crypto/ code requires using DER keys, which we want to avoid.

typedef struct QCryptoGcryptRSA {
    QCryptoAkCipher akcipher;
    gcry_sexp_t key;
    QCryptoRSAPaddingAlgo padding_alg;
    QCryptoHashAlgo hash_alg;
} QCryptoGcryptRSA;

static void byteswap_rsa(uint32_t *p, uint32_t *outptr, size_t size) { // SE uses little-endian, convert to/from big-endian.
    uint32_t out[0x40]={};
    if (size>sizeof(out)) size = sizeof(out);
    size_t count = size>>2;

    for (size_t i=0; i<count; i++) {
        out[count-1-i] = p[i];
        bswap32s(&out[count-1-i]);
    }
    memcpy(outptr, out, sizeof(out));
}

static void qcrypto_gcrypt_rsa_free(QCryptoAkCipher *akcipher)
{
    QCryptoGcryptRSA *rsa = (QCryptoGcryptRSA *)akcipher;
    if (!rsa) {
        return;
    }

    gcry_sexp_release(rsa->key);
    g_free(rsa);
}

static void qcrypto_gcrypt_set_rsa_size(QCryptoAkCipher *akcipher, gcry_mpi_t n)
{
    size_t key_size = (gcry_mpi_get_nbits(n) + 7) / 8;
    akcipher->max_plaintext_len = key_size;
    akcipher->max_ciphertext_len = key_size;
    akcipher->max_dgst_len = key_size;
    akcipher->max_signature_len = key_size;
}

static int qcrypto_gcrypt_parse_rsa_public_key_raw(QCryptoGcryptRSA *rsa,
                                               const uint8_t *n_buf,
                                               size_t n_size,
                                               const uint8_t *e_buf,
                                               size_t e_size,
                                               Error **errp)
{

    gcry_mpi_t n = NULL, e = NULL;
    int ret = -1;
    gcry_error_t err;

    err = gcry_mpi_scan(&n, GCRYMPI_FMT_STD,
                        n_buf, n_size, NULL);
    if (gcry_err_code(err) != 0) {
        error_setg(errp, "Failed to parse RSA parameter n: %s/%s",
                   gcry_strsource(err), gcry_strerror(err));
        goto cleanup;
    }

    err = gcry_mpi_scan(&e, GCRYMPI_FMT_STD,
                        e_buf, e_size, NULL);
    if (gcry_err_code(err) != 0) {
        error_setg(errp, "Failed to parse RSA parameter e: %s/%s",
                   gcry_strsource(err), gcry_strerror(err));
        goto cleanup;
    }

    err = gcry_sexp_build(&rsa->key, NULL,
                          "(public-key (rsa (n %m) (e %m)))", n, e);
    if (gcry_err_code(err) != 0) {
        error_setg(errp, "Failed to build RSA public key: %s/%s",
                   gcry_strsource(err), gcry_strerror(err));
        goto cleanup;
    }
    qcrypto_gcrypt_set_rsa_size((QCryptoAkCipher *)rsa, n);
    ret = 0;

cleanup:
    gcry_mpi_release(n);
    gcry_mpi_release(e);
    return ret;
}

extern QCryptoAkCipherDriver gcrypt_rsa;

static QCryptoGcryptRSA *qcrypto_gcrypt_rsa_new(
    const QCryptoAkCipherOptionsRSA *opt,
    QCryptoAkCipherKeyType type,
    const uint8_t *n, size_t n_size,
    const uint8_t *e, size_t e_size,
    Error **errp)
{
    QCryptoGcryptRSA *rsa = g_new0(QCryptoGcryptRSA, 1);
    rsa->padding_alg = opt->padding_alg;
    rsa->hash_alg = opt->hash_alg;
    rsa->akcipher.driver = &gcrypt_rsa;

    switch (type) {
    /*case QCRYPTO_AKCIPHER_KEY_TYPE_PRIVATE:
        if (qcrypto_gcrypt_parse_rsa_private_key(rsa, key, keylen, errp) != 0) {
            goto error;
        }
        break;*/

    case QCRYPTO_AK_CIPHER_KEY_TYPE_PUBLIC:
        if (qcrypto_gcrypt_parse_rsa_public_key_raw(rsa, n, n_size, e, e_size, errp) != 0) {
            goto error;
        }
        break;

    default:
        error_setg(errp, "Unknown akcipher key type %d", type);
        goto error;
    }

    return rsa;

error:
    qcrypto_gcrypt_rsa_free((QCryptoAkCipher *)rsa);
    return NULL;
}

static QCryptoAkCipher *qcrypto_akcipher_new_raw(const QCryptoAkCipherOptions *opts,
                                      QCryptoAkCipherKeyType type,
                                      const uint8_t *n, size_t n_size,
                                      const uint8_t *e, size_t e_size,
                                      Error **errp)
{
    switch (opts->alg) {
    case QCRYPTO_AK_CIPHER_ALGO_RSA:
        return (QCryptoAkCipher *)qcrypto_gcrypt_rsa_new(
            &opts->u.rsa, type, n, n_size, e, e_size, errp);

    default:
        error_setg(errp, "Unsupported algorithm: %u", opts->alg);
        return NULL;
    }

    return NULL;
}

// qcrypto/gcrypt doesn't expose the lastiv (updated-IV). The AES code below is a modified version of crypto/cipher-builtin.c.inc.

typedef struct QCryptoCipherBuiltinAESContext QCryptoCipherBuiltinAESContext;
struct QCryptoCipherBuiltinAESContext {
    AES_KEY enc;
    AES_KEY dec;
};

typedef struct QCryptoCipherBuiltinAES QCryptoCipherBuiltinAES;
struct QCryptoCipherBuiltinAES {
    QCryptoCipher base;
    QCryptoCipherBuiltinAESContext key;
    uint8_t iv[AES_BLOCK_SIZE];
};


static inline bool qcrypto_length_check(size_t len, size_t blocksize,
                                        Error **errp)
{
    if (unlikely(len & (blocksize - 1))) {
        error_setg(errp, "Length %zu must be a multiple of block size %zu",
                   len, blocksize);
        return false;
    }
    return true;
}

static void qcrypto_cipher_ctx_free(QCryptoCipher *cipher)
{
    g_free(cipher);
}

static int qcrypto_cipher_no_setiv(QCryptoCipher *cipher,
                                   const uint8_t *iv, size_t niv,
                                   Error **errp)
{
    error_setg(errp, "Setting IV is not supported");
    return -1;
}

static void do_aes_encrypt_ecb(const void *vctx,
                               size_t len,
                               uint8_t *out,
                               const uint8_t *in)
{
    const QCryptoCipherBuiltinAESContext *ctx = vctx;

    /* We have already verified that len % AES_BLOCK_SIZE == 0. */
    while (len) {
        AES_encrypt(in, out, &ctx->enc);
        in += AES_BLOCK_SIZE;
        out += AES_BLOCK_SIZE;
        len -= AES_BLOCK_SIZE;
    }
}

static void do_aes_decrypt_ecb(const void *vctx,
                               size_t len,
                               uint8_t *out,
                               const uint8_t *in)
{
    const QCryptoCipherBuiltinAESContext *ctx = vctx;

    /* We have already verified that len % AES_BLOCK_SIZE == 0. */
    while (len) {
        AES_decrypt(in, out, &ctx->dec);
        in += AES_BLOCK_SIZE;
        out += AES_BLOCK_SIZE;
        len -= AES_BLOCK_SIZE;
    }
}

// Based on the func from libgcrypt cipher-internal.h.
static inline void
cipher_block_add(void *_dstsrc, unsigned int add, size_t blocksize)
{
  uint64_t *dstsrc = _dstsrc;
  uint64_t s[2];

  if (blocksize == 8)
    {
      *((uint64_t*)&dstsrc[0]) += add;
    }
  else /* blocksize == 16 */
    {
      s[0] = ldq_be_p(&dstsrc[1]);
      s[1] = ldq_be_p(&dstsrc[0]);
      s[0] += add;
      s[1] += (s[0] < add);
      stq_be_p(&dstsrc[1], s[0]);
      stq_be_p(&dstsrc[0], s[1]);
    }
}

static void do_aes_encrypt_ctr(const AES_KEY *key,
                               size_t len,
                               uint8_t *out,
                               const uint8_t *in,
                               uint8_t *ivec)
{
    uint8_t tmp[AES_BLOCK_SIZE];
    size_t n;

    /* We have already verified that len % AES_BLOCK_SIZE == 0. */
    while (len) {
        AES_encrypt(ivec, tmp, key);
        cipher_block_add(ivec, 1, AES_BLOCK_SIZE);
        for (n = 0; n < AES_BLOCK_SIZE; ++n) {
            out[n] = in[n] ^ tmp[n];
        }
        len -= AES_BLOCK_SIZE;
        in += AES_BLOCK_SIZE;
        out += AES_BLOCK_SIZE;
    }
}

static void do_aes_encrypt_cbc(const AES_KEY *key,
                               size_t len,
                               uint8_t *out,
                               const uint8_t *in,
                               uint8_t *ivec)
{
    uint8_t tmp[AES_BLOCK_SIZE];
    size_t n;

    /* We have already verified that len % AES_BLOCK_SIZE == 0. */
    while (len) {
        for (n = 0; n < AES_BLOCK_SIZE; ++n) {
            tmp[n] = in[n] ^ ivec[n];
        }
        AES_encrypt(tmp, out, key);
        memcpy(ivec, out, AES_BLOCK_SIZE);
        len -= AES_BLOCK_SIZE;
        in += AES_BLOCK_SIZE;
        out += AES_BLOCK_SIZE;
    }
}

static void do_aes_decrypt_cbc(const AES_KEY *key,
                               size_t len,
                               uint8_t *out,
                               const uint8_t *in,
                               uint8_t *ivec)
{
    uint8_t tmp[AES_BLOCK_SIZE];
    size_t n;

    /* We have already verified that len % AES_BLOCK_SIZE == 0. */
    while (len) {
        memcpy(tmp, in, AES_BLOCK_SIZE);
        AES_decrypt(in, out, key);
        for (n = 0; n < AES_BLOCK_SIZE; ++n) {
            out[n] ^= ivec[n];
        }
        memcpy(ivec, tmp, AES_BLOCK_SIZE);
        len -= AES_BLOCK_SIZE;
        in += AES_BLOCK_SIZE;
        out += AES_BLOCK_SIZE;
    }
}

static int qcrypto_cipher_aes_encrypt_ecb(QCryptoCipher *cipher,
                                          const void *in, void *out,
                                          size_t len, Error **errp)
{
    QCryptoCipherBuiltinAES *ctx
        = container_of(cipher, QCryptoCipherBuiltinAES, base);

    if (!qcrypto_length_check(len, AES_BLOCK_SIZE, errp)) {
        return -1;
    }
    do_aes_encrypt_ecb(&ctx->key, len, out, in);
    return 0;
}

static int qcrypto_cipher_aes_decrypt_ecb(QCryptoCipher *cipher,
                                          const void *in, void *out,
                                          size_t len, Error **errp)
{
    QCryptoCipherBuiltinAES *ctx
        = container_of(cipher, QCryptoCipherBuiltinAES, base);

    if (!qcrypto_length_check(len, AES_BLOCK_SIZE, errp)) {
        return -1;
    }
    do_aes_decrypt_ecb(&ctx->key, len, out, in);
    return 0;
}

static int qcrypto_cipher_aes_encrypt_ctr(QCryptoCipher *cipher,
                                          const void *in, void *out,
                                          size_t len, Error **errp)
{
    QCryptoCipherBuiltinAES *ctx
        = container_of(cipher, QCryptoCipherBuiltinAES, base);

    if (!qcrypto_length_check(len, AES_BLOCK_SIZE, errp)) {
        return -1;
    }
    do_aes_encrypt_ctr(&ctx->key.enc, len, out, in, ctx->iv);
    return 0;
}

static int qcrypto_cipher_aes_encrypt_cbc(QCryptoCipher *cipher,
                                          const void *in, void *out,
                                          size_t len, Error **errp)
{
    QCryptoCipherBuiltinAES *ctx
        = container_of(cipher, QCryptoCipherBuiltinAES, base);

    if (!qcrypto_length_check(len, AES_BLOCK_SIZE, errp)) {
        return -1;
    }
    do_aes_encrypt_cbc(&ctx->key.enc, len, out, in, ctx->iv);
    return 0;
}

static int qcrypto_cipher_aes_decrypt_cbc(QCryptoCipher *cipher,
                                          const void *in, void *out,
                                          size_t len, Error **errp)
{
    QCryptoCipherBuiltinAES *ctx
        = container_of(cipher, QCryptoCipherBuiltinAES, base);

    if (!qcrypto_length_check(len, AES_BLOCK_SIZE, errp)) {
        return -1;
    }
    do_aes_decrypt_cbc(&ctx->key.dec, len, out, in, ctx->iv);
    return 0;
}

static int qcrypto_cipher_aes_setiv(QCryptoCipher *cipher, const uint8_t *iv,
                             size_t niv, Error **errp)
{
    QCryptoCipherBuiltinAES *ctx
        = container_of(cipher, QCryptoCipherBuiltinAES, base);

    if (niv != AES_BLOCK_SIZE) {
        error_setg(errp, "IV must be %d bytes not %zu",
                   AES_BLOCK_SIZE, niv);
        return -1;
    }

    memcpy(ctx->iv, iv, AES_BLOCK_SIZE);
    return 0;
}

static int qcrypto_cipher_aes_getiv(QCryptoCipher *cipher, uint8_t *iv,
                             size_t niv, Error **errp)
{
    QCryptoCipherBuiltinAES *ctx
        = container_of(cipher, QCryptoCipherBuiltinAES, base);

    if (niv != AES_BLOCK_SIZE) {
        error_setg(errp, "IV must be %d bytes not %zu",
                   AES_BLOCK_SIZE, niv);
        return -1;
    }

    memcpy(iv, ctx->iv, AES_BLOCK_SIZE);
    return 0;
}

static const struct QCryptoCipherDriver qcrypto_cipher_aes_driver_ecb = {
    .cipher_encrypt = qcrypto_cipher_aes_encrypt_ecb,
    .cipher_decrypt = qcrypto_cipher_aes_decrypt_ecb,
    .cipher_setiv = qcrypto_cipher_no_setiv,
    .cipher_free = qcrypto_cipher_ctx_free,
};

static const struct QCryptoCipherDriver qcrypto_cipher_aes_driver_ctr = {
    .cipher_encrypt = qcrypto_cipher_aes_encrypt_ctr,
    .cipher_decrypt = qcrypto_cipher_aes_encrypt_ctr,
    .cipher_setiv = qcrypto_cipher_aes_setiv,
    .cipher_free = qcrypto_cipher_ctx_free,
};

static const struct QCryptoCipherDriver qcrypto_cipher_aes_driver_cbc = {
    .cipher_encrypt = qcrypto_cipher_aes_encrypt_cbc,
    .cipher_decrypt = qcrypto_cipher_aes_decrypt_cbc,
    .cipher_setiv = qcrypto_cipher_aes_setiv,
    .cipher_free = qcrypto_cipher_ctx_free,
};

static QCryptoCipher *qcrypto_cipher_ctx_new(QCryptoCipherAlgo alg,
                                             QCryptoCipherMode mode,
                                             const uint8_t *key,
                                             size_t nkey,
                                             Error **errp)
{
    switch (alg) {
    case QCRYPTO_CIPHER_ALGO_AES_128:
    case QCRYPTO_CIPHER_ALGO_AES_192:
    case QCRYPTO_CIPHER_ALGO_AES_256:
        {
            QCryptoCipherBuiltinAES *ctx;
            const QCryptoCipherDriver *drv;

            switch (mode) {
            case QCRYPTO_CIPHER_MODE_ECB:
                drv = &qcrypto_cipher_aes_driver_ecb;
                break;
            case QCRYPTO_CIPHER_MODE_CTR:
                drv = &qcrypto_cipher_aes_driver_ctr;
                break;
            case QCRYPTO_CIPHER_MODE_CBC:
                drv = &qcrypto_cipher_aes_driver_cbc;
                break;
            default:
                goto bad_mode;
            }

            ctx = g_new0(QCryptoCipherBuiltinAES, 1);
            ctx->base.driver = drv;

            if (AES_set_encrypt_key(key, nkey * 8, &ctx->key.enc)) {
                error_setg(errp, "Failed to set encryption key");
                goto error;
            }
            if (AES_set_decrypt_key(key, nkey * 8, &ctx->key.dec)) {
                error_setg(errp, "Failed to set decryption key");
                goto error;
            }

            return &ctx->base;

        error:
            g_free(ctx);
            return NULL;
        }

    default:
        error_setg(errp,
                   "Unsupported cipher algorithm %s",
                   QCryptoCipherAlgo_str(alg));
        return NULL;
    }

 bad_mode:
    error_setg(errp, "Unsupported cipher mode %s",
               QCryptoCipherMode_str(mode));
    return NULL;
}

static MemTxAttrs tegra_se_get_memattrs(tegra_se *s)
{
    return (MemTxAttrs){ .secure = 1 };
}

int tegra_se_crypto_operation(void *opaque, void* key, void* iv, QCryptoCipherAlgo cipher_alg, QCryptoCipherMode mode, bool encrypt, uintptr_t inbuf, uintptr_t outbuf, dma_addr_t databuf_size, bool inbuf_host, bool outbuf_host)
{
    tegra_se *s = opaque;
    Error *err = NULL;
    int tmpret=0;
    size_t datasize=0;
    void* databuf_in = NULL;
    void* databuf_out = NULL;
    dma_addr_t tmplen_in=databuf_size, tmplen_out=databuf_size;
    QCryptoCipher *cipher = qcrypto_cipher_ctx_new(cipher_alg, mode, key, qcrypto_cipher_get_key_len(cipher_alg), &err);

    if (!inbuf_host)
        databuf_in = dma_memory_map(&address_space_memory, inbuf, &tmplen_in, DMA_DIRECTION_TO_DEVICE, tegra_se_get_memattrs(s));
    else
        databuf_in = (void*)inbuf;
    if (!outbuf_host)
        databuf_out = dma_memory_map(&address_space_memory, outbuf, &tmplen_out, DMA_DIRECTION_FROM_DEVICE, tegra_se_get_memattrs(s));
    else
        databuf_out = (void*)outbuf;

    if (cipher) {
        if (databuf_in==NULL || databuf_out==NULL)
            qemu_log_mask(LOG_GUEST_ERROR, "tegra.se: Failed to DMA map input/output buffer.\n");
        else {
            datasize = databuf_size;

            if (mode != QCRYPTO_CIPHER_MODE_ECB) {
                tmpret = qcrypto_cipher_setiv(cipher, iv, 0x10, &err);
                if (tmpret!=0) datasize=0;
            }

            if (tmpret==0) {
                if (encrypt) {
                    if (qcrypto_cipher_encrypt(cipher, databuf_in, databuf_out, datasize, &err)!=0) datasize=0;
                }
                else {
                    if (qcrypto_cipher_decrypt(cipher, databuf_in, databuf_out, datasize, &err)!=0) datasize=0;
                }
                if (datasize==0) tmpret = -1;
            }
        }

        qcrypto_cipher_free(cipher);
    }
    else tmpret = -1;

    if (databuf_in && !inbuf_host) dma_memory_unmap(&address_space_memory, databuf_in, databuf_size, DMA_DIRECTION_TO_DEVICE, databuf_size);
    if (databuf_out && !outbuf_host) dma_memory_unmap(&address_space_memory, databuf_out, databuf_size, DMA_DIRECTION_FROM_DEVICE, datasize);
    if (err) error_report_err(err);

    return tmpret;
}

static void tegra_se_set_aes_keyslots_lock(Object *obj, Visitor *v, const char *name,
                                           void *opaque, Error **errp)
{
    tegra_se *s = TEGRA_SE(obj);
    uint64_t value=0;
    char *keyname = NULL;
    char *strvalue = NULL;
    char *strptr = NULL;
    char tmpstr[256]={};

    if (!visit_type_str(v, name, &strvalue, errp)) {
        return;
    }

    strptr = tmpstr;
    keyname = tmpstr;
    pstrcpy(tmpstr, sizeof(tmpstr), strvalue);
    qemu_strsep(&strptr, ":");
    strvalue = strptr;

    if (strvalue == NULL) {
        error_setg(errp, "error reading %s '%s'", name, tmpstr);
        return;
    }

    unsigned long _value=0;
    if (qemu_strtoul(strvalue, NULL, 16, &_value)) {
        error_setg(errp, "error reading %s '%s'", name, strvalue);
        return;
    }
    value = _value;

    if (qemu_strtoul(keyname, NULL, 16, &_value)) {
        error_setg(errp, "error reading %s '%s'", name, keyname);
        return;
    }

    if (_value < ARRAY_SIZE(s->aes_keyslots_lock)) {
        s->aes_keyslots_lock[_value] = value;
    }
    else
        error_setg(errp, "error reading %s '%s': id 0x%" PRIx64 " is too large.", name, keyname, _value);
}

static uint32_t tegra_se_get_aes_keyslot_access(tegra_se *s, uint32_t slot)
{
    if (slot < ARRAY_SIZE(s->aes_keyslots_lock)) {
        return s->regs.SE_CRYPTO_KEYTABLE_ACCESS[slot] & s->aes_keyslots_lock[slot];
    }
    else
        return 0x7F;
}

static bool tegra_se_check_aes_keyslot_write(tegra_se *s, uint32_t aes_tableoffset)
{
    uint32_t keyslot = aes_tableoffset>>4;
    uint32_t keytable_type = (aes_tableoffset & 0xF) >> 2;
    keytable_type = keytable_type < 2 ? 0 : keytable_type - 1;
    uint32_t access = tegra_se_get_aes_keyslot_access(s, keyslot);

    return access & BIT(1 + 2*keytable_type); // *Write
}

void tegra_se_lock_aes_keyslot(uint32_t slot, uint32_t flags) {
    tegra_se *s = tegra_se_dev;

    // We don't call this with any low-8-bits flags, so don't impl locking those.

    if (flags & 0x100)
        s->regs.SE_CRYPTO_SECURITY_PERKEY &= ~(1<<slot);
}

void tegra_se_set_aes_keyslot(uint32_t slot, void* key, size_t key_size) {
    tegra_se *s = tegra_se_dev;

    if (key_size > 0x20) key_size = 0x20;
    memcpy(&s->aes_keytable[slot*0x10], key, key_size);
}

static void tegra_se_copy_bitmap(void* dst, unsigned long data,
                                 unsigned long *shift, unsigned long nbits)
{
    bitmap_copy_with_dst_offset(dst, &data,
                                *shift, nbits);
    *shift += nbits;
}

// Extract sticky-bits state into the packed context data.
static void tegra_se_get_context_sticky_bits(tegra_se *s, uint32_t *out)
{
   unsigned long bitpos = 0;

   tegra_se_copy_bitmap(out, (s->regs.SE_SE_SECURITY & 0x7) | (((s->regs.SE_SE_SECURITY>>16) & 0x1)<<3), &bitpos, 4);
   tegra_se_copy_bitmap(out, s->regs.SE_TZRAM_SECURITY, &bitpos, 2);
   tegra_se_copy_bitmap(out, s->regs.SE_CRYPTO_SECURITY_PERKEY, &bitpos, 16);

    unsigned long bitnum = 7;
    if (tegra_board >= TEGRAX1PLUS_BOARD) bitnum++;

    for (int i=0; i<0x10; i++) {
        tegra_se_copy_bitmap(out, s->regs.SE_CRYPTO_KEYTABLE_ACCESS[i], &bitpos, bitnum);
    }

   tegra_se_copy_bitmap(out, s->regs.SE_RSA_SECURITY_PERKEY, &bitpos, 2);
   tegra_se_copy_bitmap(out, s->regs.SE_RSA_KEYTABLE_ACCESS[0], &bitpos, 3);
   tegra_se_copy_bitmap(out, s->regs.SE_RSA_KEYTABLE_ACCESS[1], &bitpos, 3);

   if (tegra_board >= TEGRAX1PLUS_BOARD)
       tegra_se_copy_bitmap(out, (s->regs.SE_SE_SECURITY >> 4) & 0x1, &bitpos, 1);
}

// Extract PKA sticky-bits state into the packed context data.
static void tegra_se_get_context_pka_sticky_bits(tegra_se *s, uint32_t *out)
{
   unsigned long bitpos = 0;
   unsigned long unk=0;

    for (int i=0; i<0x4*2; i++) {
        tegra_se_copy_bitmap(out, s->regs_pka[(0x8860>>2) + i], &bitpos, 3);
    }

   tegra_se_copy_bitmap(out, (s->regs_pka[0x8850>>2] & 0x7) | (((s->regs_pka[0x8850>>2]>>16) & 0x1)<<3), &bitpos, 4);
   tegra_se_copy_bitmap(out, ((s->regs_pka[0x8120>>2]>>31) & 0x1) | ((s->regs_pka[0x8120>>2] & 0xFFFFF)<<1), &bitpos, 21);
   tegra_se_copy_bitmap(out, unk, &bitpos, 32);
}

// Save the context.
static void tegra_se_auto_save_context(tegra_se *s)
{
    Error *err = NULL;
    size_t size = s->engine == 1 ? 0x840 : 0x2850;
    uint32_t data[0x2850>>2]={};
    tegra_se_context *context = (tegra_se_context*)data;
    // The fixed_pattern was reversed compared with X1 (T210).
    static uint8_t fixed_pattern[0x10] = { 0x0F, 0x0E, 0x0D, 0x0C, 0x0B, 0x0A, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00 };

    dma_addr_t context_addr = tegra_pmc_get_se_context_addr(s->engine);
    uint32_t key[4]={};
    uint32_t iv[4]={};

    s->regs.SE_CTX_SAVE_AUTO &= ~(0x3FF<<16); // CTX_SAVE_AUTO_CURR_CNT
    s->regs.SE_CTX_SAVE_AUTO |= ((size>>4)+1)<<16;

    if (!context_addr) {
        qemu_log_mask(LOG_GUEST_ERROR, "tegra.se: Skipping the rest of the auto-context-save for engine %d since the PMC scratch reg for the context_addr is not set.\n", s->engine);
        return;
    }

    qemu_guest_getrandom(key, sizeof(key), &err);
    if (err) {
        error_report_err(err);
        err = NULL;
    }
    tegra_pmc_set_srk(s->engine, key);

    qemu_guest_getrandom(context->rand, sizeof(context->rand), &err);
    if (err) {
        error_report_err(err);
        err = NULL;
    }

    tegra_se_get_context_sticky_bits(s, context->sticky_bits);

    memcpy(context->aes_keytable, s->aes_keytable, sizeof(s->aes_keytable));
    memcpy(context->rsa_keytable, s->rsa_keytable, sizeof(s->rsa_keytable));

    if (s->engine != 1) { // SE2, PKA
        tegra_se_get_context_pka_sticky_bits(s, context->fixed_pattern); // PKA sticky-bits are located where fixed_pattern was with SE1.
        memcpy(context+1, s->pka_keytable, sizeof(s->pka_keytable));
    }

    memcpy(&data[(size-0x10)>>2], fixed_pattern, sizeof(fixed_pattern));

    tegra_se_crypto_operation(s, key, iv, QCRYPTO_CIPHER_ALGO_AES_128,
                              QCRYPTO_CIPHER_MODE_CBC, true,
                              (uintptr_t)data, context_addr,
                              size, true, false);
}

static uint64_t tegra_se_priv_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    tegra_se *s = opaque;
    uint64_t ret = 0;

    assert(offset < sizeof(s->regs_raw));

    TRACE_READ(s->iomem.addr, offset, ret);

    uint32_t *regs = s->regs_raw;
    ret = regs[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);

    return ret;
}

static void tegra_se_priv_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    tegra_se *s = opaque;

    assert(offset+size <= sizeof(s->regs_raw));

    TRACE_WRITE(s->iomem.addr, offset, 0, value);

    Error *err = NULL;
    uint32_t *regs = s->regs_raw;
    uint32_t tmp=0;
    size_t n_size=0, e_size=0;
    LinkedListEntry in_entry={}, out_entry={};
    uint8_t indata[0x100]={};
    uint32_t n_buf[0x40]={};
    uint32_t e_buf[0x40]={};

    uint32_t iv[4]={};

    uint32_t sticky_bits[0x20>>2]={};

    uint32_t cfg_dst = (s->regs.SE_CONFIG >> 2) & 0x7;

    if (offset == SE_OPERATION_OFFSET || (offset == SE_RSA_KEYTABLE_ADDR_OFFSET && cfg_dst==0)) {
        dma_memory_read(&address_space_memory, s->regs.SE_IN_LL_ADDR, &in_entry, sizeof(in_entry), tegra_se_get_memattrs(s));
    }

    switch (offset) {
        case SE_TZRAM_SECURITY_OFFSET:
            s->regs.SE_TZRAM_SECURITY = value;
            tegra_apb_misc_set_slave_sec_extra(tegra_apb_misc_dev, (~s->regs.SE_TZRAM_SECURITY) & 0x1);
        break;

        case SE_INT_ENABLE_OFFSET:
            tmp = s->regs.SE_INT_ENABLE & s->regs.SE_INT_STATUS;
            s->regs.SE_INT_ENABLE = value;
            if (tmp!=0 && (tmp & value)==0) TRACE_IRQ_LOWER(s->iomem.addr, s->irq);
        break;

        case SE_INT_STATUS_OFFSET:
            tmp = s->regs.SE_INT_ENABLE & s->regs.SE_INT_STATUS;
            s->regs.SE_INT_STATUS &= ~value;
            if (tmp != 0 && ((s->regs.SE_INT_ENABLE & s->regs.SE_INT_STATUS) == 0)) TRACE_IRQ_LOWER(s->iomem.addr, s->irq);
        break;

        case SE_OPERATION_OFFSET:
        case SE_OUT_LL_ADDR_OFFSET:
            regs[offset/sizeof(uint32_t)] = value;
            if (offset == SE_OPERATION_OFFSET) s->regs.SE_INT_STATUS |= 1<<4; // INT_STATUS_SE_OP_DONE
            if (s->regs.SE_INT_ENABLE & s->regs.SE_INT_STATUS) TRACE_IRQ_RAISE(s->iomem.addr, s->irq);

            bool ctxsave = false;

            if (offset == SE_OPERATION_OFFSET) {
                uint32_t op = s->regs.SE_OPERATION;

                dma_addr_t tmplen_in = in_entry.size;
                void* databuf_in = NULL;
                uint32_t ctxsave_src = (s->regs.SE_CTX_SAVE_CONFIG >> 29) & 0x7;

                if (op==3) { // CTX_SAVE
                    bool ctx_save_auto_enable = s->regs.SE_CTX_SAVE_AUTO & 0x1;

                    if (ctx_save_auto_enable != (tegra_board >= TEGRAX1PLUS_BOARD)) {
                        qemu_log_mask(LOG_GUEST_ERROR, "tegra.se: Ignoring attempt to use automatic/original context save since the current board doesn't support it.\n");
                    }
                    else if (ctx_save_auto_enable) { // Automatic context save
                        tegra_se_auto_save_context(s);
                    }
                    else { // Original context save.
                        ctxsave = true;

                        uint32_t word_quad=0;
                        if (ctxsave_src!=4) { // MEM, handled below.
                           if (ctxsave_src==0) { // STICKY_BITS
                               uint32_t tmpaddr = (s->regs.SE_CTX_SAVE_CONFIG >> 24) & 0x1; // CTX_SAVE_CONFIG_STICKY_WORD_QUAD
                               tegra_se_get_context_sticky_bits(s, sticky_bits);
                               databuf_in = &sticky_bits[tmpaddr*0x4];
                           }
                           else if (ctxsave_src==1) { // RSA_KEYTABLE
                               uint32_t tmpaddr = (s->regs.SE_CTX_SAVE_CONFIG >> 16) & 0x3; // CTX_SAVE_CONFIG_RSA_KEY_INDEX
                               word_quad = (s->regs.SE_CTX_SAVE_CONFIG>>12) & 0xF; // CTX_SAVE_CONFIG_RSA_WORD_QUAD
                               tmpaddr = (tmpaddr*0x40) + (word_quad*0x4);
                               databuf_in = &s->rsa_keytable[tmpaddr];
                           }
                           else if (ctxsave_src==2) { // AES_KEYTABLE
                               uint32_t tmpaddr = (s->regs.SE_CTX_SAVE_CONFIG >> 8) & 0xF; // CTX_SAVE_CONFIG_AES_KEY_INDEX
                               word_quad = s->regs.SE_CTX_SAVE_CONFIG & 0x3; // CTX_SAVE_CONFIG_AES_WORD_QUAD
                               tmpaddr = (tmpaddr*0x10) | (word_quad*0x4);
                               databuf_in = &s->aes_keytable[tmpaddr];
                           }
                           // 3: PKA1_STICKY_BITS, not impl'd
                           // 4: MEM, handled below.
                           else if (ctxsave_src==6) { // SRK
                               tegra_pmc_set_srk(s->engine, s->srk);
                               break;
                           }
                           else {
                               qemu_log_mask(LOG_UNIMP, "tegra.se: Ignoring unsupported ctxsave_src=%d.\n", ctxsave_src);
                               break;
                           }
                           tmplen_in = 0x10;
                        }
                    }

                    if (!ctxsave) break;
                }
                else if (op!=1) { // START
                    qemu_log_mask(LOG_UNIMP, "tegra.se: Ignoring unsupported op=%d.\n", op);
                    break;
                }

                if (op==1 || (ctxsave && ctxsave_src==4)) databuf_in = dma_memory_map(&address_space_memory, in_entry.address, &tmplen_in, DMA_DIRECTION_TO_DEVICE, tegra_se_get_memattrs(s));

                if (cfg_dst==0) dma_memory_read(&address_space_memory, s->regs.SE_OUT_LL_ADDR, &out_entry, sizeof(out_entry), tegra_se_get_memattrs(s));
                //printf("se in: 0x%x, 0x%x, 0x%x\n", in_entry.zero, in_entry.address, in_entry.size);
                //printf("se out: 0x%x, 0x%x, 0x%x\n", out_entry.zero, out_entry.address, out_entry.size);

                uint32_t dec_alg = (s->regs.SE_CONFIG >> 8) & 0xF;
                uint32_t enc_alg = (s->regs.SE_CONFIG >> 12) & 0xF;
                uint32_t dec_mode = (s->regs.SE_CONFIG >> 16) & 0xFF;
                uint32_t enc_mode = (s->regs.SE_CONFIG >> 24) & 0xFF;

                void* databuf_out = NULL;
                dma_addr_t databuf_outsize = 0;
                size_t datasize=0;

                //printf("SE: dec_alg = 0x%x, enc_alg = 0x%x, dec_mode = 0x%x, enc_mode = 0x%x\n", dec_alg, enc_alg, dec_mode, enc_mode);

                if (cfg_dst==0) { // MEMORY
                    databuf_outsize = out_entry.size;
                    databuf_out = dma_memory_map(&address_space_memory, out_entry.address, &databuf_outsize, DMA_DIRECTION_FROM_DEVICE, tegra_se_get_memattrs(s));
                }
                else if (cfg_dst==1) { // HASH_REG
                    databuf_out = &s->regs.SE_HASH_RESULT;
                    databuf_outsize = sizeof(s->regs.SE_HASH_RESULT);
                }
                else if (cfg_dst==2) { // KEYTABLE
                    uint32_t dst_word = s->regs.SE_CRYPTO_KEYTABLE_DST & 0x3;
                    uint32_t dst_keyindex = (s->regs.SE_CRYPTO_KEYTABLE_DST>>8) & 0xF;
                    size_t keyoff = dst_keyindex*0x10 + dst_word*0x4;
                    databuf_out = &s->aes_keytable[keyoff];
                    databuf_outsize = sizeof(s->aes_keytable) - (keyoff<<2);

                    if (!tegra_se_check_aes_keyslot_write(s, keyoff)) { // *Write
                        databuf_out = NULL;
                        qemu_log_mask(LOG_GUEST_ERROR, "tegra.se: Ignoring operation with dst AES keytable for keyslot %"PRIu32" since *Write is locked.\n", dst_keyindex);
                    }
                }
                else if (cfg_dst==3) { // SRK
                    databuf_out = &s->srk;
                    databuf_outsize = sizeof(s->srk);
                    memset(s->srk_iv, 0, sizeof(s->srk_iv));
                }
                else if (cfg_dst==4) { // RSA_REG
                    databuf_out = &s->regs.SE_RSA_OUTPUT;
                    databuf_outsize = sizeof(s->regs.SE_RSA_OUTPUT);
                }

                if (databuf_out==NULL) {
                    if (cfg_dst==0) qemu_log_mask(LOG_GUEST_ERROR, "tegra.se: Failed to DMA map output buffer.\n");
                    else qemu_log_mask(LOG_UNIMP, "tegra.se: Ignoring cfg_dst=%d.\n", cfg_dst);
                    datasize = 0;
                }
                else if (dec_alg == 0x1 || enc_alg == 0x1) { // AES
                    uint32_t cfg = s->regs.SE_CRYPTO_CONFIG;
                    bool hash_enable = cfg & 0x1;
                    uint32_t xor_pos = (cfg>>1) & 0x3;
                    uint32_t input_sel = (cfg>>3) & 0x3;
                    //uint32_t vctram_sel = (cfg>>5) & 0x3;
                    bool iv_select = (cfg>>7) & 0x1;
                    bool encrypt = (cfg>>8) & 0x1; // CRYPTO_CONFIG_CORE_SEL
                    //uint32_t ctr_cntn = (cfg>>11) & 0xFF;
                    uint32_t keyindex = (cfg>>24) & 0xF;
                    uint8_t *key = (uint8_t*)&s->aes_keytable[keyindex*0x10];

                    if (ctxsave) hash_enable = false;

                    if (hash_enable && !encrypt) {
                        qemu_log_mask(LOG_UNIMP, "tegra.se: AES hash_enable=1 with encrypt=0 is not supported.\n");
                    }
                    else if (cfg_dst!=0 && cfg_dst>4) {
                        qemu_log_mask(LOG_UNIMP, "tegra.se: SE_CONFIG DST>4 is not supported: %u.\n", cfg_dst);
                    }
                    else if (tegra_board >= TEGRAX1PLUS_BOARD && (tegra_se_get_aes_keyslot_access(s, keyindex) & BIT(7)) && cfg_dst!=2) {
                        qemu_log_mask(LOG_GUEST_ERROR, "tegra.se: Ignoring operation with cfg_dst=%"PRIu32" for keyslot %"PRIu32" since DstKeyTableOnly is locked.\n", cfg_dst, keyindex);
                    }
                    else {
                        uint32_t tmpmode = encrypt ? enc_mode : dec_mode;

                        QCryptoCipherAlgo cipher_alg = QCRYPTO_CIPHER_ALGO__MAX;
                        switch (tmpmode) {
                            case SE_CONFIG_ENC_MODE_AESMODE_KEY128:
                                cipher_alg = QCRYPTO_CIPHER_ALGO_AES_128;
                            break;

                            case SE_CONFIG_ENC_MODE_AESMODE_KEY192:
                                cipher_alg = QCRYPTO_CIPHER_ALGO_AES_192;
                            break;

                            case SE_CONFIG_ENC_MODE_AESMODE_KEY256:
                                cipher_alg = QCRYPTO_CIPHER_ALGO_AES_256;
                            break;

                            default:
                                qemu_log_mask(LOG_UNIMP, "tegra.se: Unsupported SE AES alg: %u.\n", tmpmode);
                            break;
                        }

                        QCryptoCipherMode mode = QCRYPTO_CIPHER_MODE__MAX;

                        if (ctxsave) {
                            encrypt = true;
                            mode = QCRYPTO_CIPHER_MODE_CBC;
                            key = (uint8_t*)s->srk;
                        }
                        else {
                            if (xor_pos==0) mode = QCRYPTO_CIPHER_MODE_ECB; // BYPASS
                            else if (input_sel==3) mode = QCRYPTO_CIPHER_MODE_CTR; // LINEAR_CTR
                            else if (xor_pos==2 || xor_pos==3) mode = QCRYPTO_CIPHER_MODE_CBC; // TOP/BOTTOM
                            else {
                                qemu_log_mask(LOG_UNIMP, "tegra.se: Unsupported SE AES mode, SE_CRYPTO_CONFIG: 0x%x.\n", cfg);
                            }
                        }

                        if (cipher_alg!=QCRYPTO_CIPHER_ALGO__MAX && mode!=QCRYPTO_CIPHER_MODE__MAX) {
                            QCryptoCipher *cipher = qcrypto_cipher_ctx_new(cipher_alg, mode, key, qcrypto_cipher_get_key_len(cipher_alg), &err);

                            if (cipher) {
                                size_t blocklen = qcrypto_cipher_get_block_len(cipher_alg);
                                datasize = (s->regs.SE_CRYPTO_LAST_BLOCK+1)*blocklen;
                                if (!hash_enable && datasize > databuf_outsize) datasize = databuf_outsize;
                                if (datasize > tmplen_in) datasize = tmplen_in;

                                if (databuf_in==NULL || databuf_out==NULL) {
                                    qemu_log_mask(LOG_GUEST_ERROR, "tegra.se: Failed to DMA map AES input/output buffer.\n");
                                    datasize = 0;
                                }
                                else {
                                    if (mode == QCRYPTO_CIPHER_MODE_CTR) {
                                        for (size_t i=0; i<4; i++) {
                                            iv[i] = s->regs.SE_CRYPTO_LINEAR_CTR[i];
                                            //bswap32s(&iv[i]);
                                        }
                                    }
                                    else if (mode != QCRYPTO_CIPHER_MODE_ECB) {
                                        if (!ctxsave)
                                            memcpy(iv, &s->aes_keytable[keyindex*0x10 + (iv_select==0 ? 0x8 : 0xC)], 0x10);
                                        else
                                            memcpy(iv, s->srk_iv, 0x10);
                                    }

                                    if (!ctxsave)
                                        qemu_log("tegra_se_aes_info keyindex = %u, cfg_dst = %u, tmpmode = 0x%x, mode = %d, encrypt = %d, perkey[%u] = 0x%x\n", keyindex, cfg_dst, tmpmode, mode, encrypt,  keyindex, (s->regs.SE_CRYPTO_SECURITY_PERKEY & (1<<keyindex))!=0);
                                    else
                                        qemu_log("tegra_se_aes_info cfg_dst = %u, tmpmode = 0x%x, ctxsave_src = 0x%x\n", cfg_dst, tmpmode, ctxsave_src);
                                    se_log_hexdump("tegra_se_aes_key", key, qcrypto_cipher_get_key_len(cipher_alg));
                                    se_log_hexdump("tegra_se_crypto_input", databuf_in, 0x10);

                                    int tmpret=0;
                                    if (mode != QCRYPTO_CIPHER_MODE_ECB) {
                                        se_log_hexdump("tegra_se_crypto_iv", iv, sizeof(iv));
                                        tmpret = qcrypto_cipher_setiv(cipher, (uint8_t*)iv, sizeof(iv), &err);
                                    }

                                    // If this is a single block and the input matches the override, use the override data for output instead.
                                    if (datasize==blocklen &&
                                        s->aes_dataoverride_enable[keyindex] &&
                                        memcmp(databuf_in, &s->aes_dataoverride[keyindex*0x8], 0x10)==0) {

                                        memcpy(databuf_out, &s->aes_dataoverride[keyindex*0x8 + 0x4], 0x10);
                                        se_log_hexdump("tegra_se_crypto_output_overridden", databuf_out, 0x10);
                                    }
                                    else if (tmpret==0) {
                                        if (encrypt) {
                                            if (!hash_enable) {
                                                if (qcrypto_cipher_encrypt(cipher, databuf_in, databuf_out, datasize, &err)!=0) datasize=0;
                                            }
                                            else {
                                                uint8_t tmpblock[0x20]={};
                                                uint8_t *databuf_in_u8 = (uint8_t*)databuf_in;
                                                for (size_t i=0; i<datasize; i+=blocklen) {
                                                    if (qcrypto_cipher_encrypt(cipher, &databuf_in_u8[i], tmpblock, blocklen, &err)!=0) {
                                                        datasize=0;
                                                        break;
                                                    }
                                                }
                                                if (datasize>0) {
                                                    datasize = blocklen;
                                                    if (datasize > databuf_outsize) datasize = databuf_outsize;
                                                    memcpy(databuf_out, tmpblock, datasize);
                                                }
                                            }
                                        }
                                        else {
                                            if (qcrypto_cipher_decrypt(cipher, databuf_in, databuf_out, datasize, &err)!=0) datasize=0;
                                        }

                                        if (datasize>0 && mode != QCRYPTO_CIPHER_MODE_ECB) { // Store the updated IV into the regs.
                                            tmpret = qcrypto_cipher_aes_getiv(cipher, (uint8_t*)iv, sizeof(iv), &err);
                                            if (tmpret==0) {
                                                if (mode == QCRYPTO_CIPHER_MODE_CTR) {
                                                    for (size_t i=0; i<4; i++) {
                                                        s->regs.SE_CRYPTO_LINEAR_CTR[i] = iv[i];
                                                        //bswap32s(&s->regs.SE_CRYPTO_LINEAR_CTR[i]);
                                                    }
                                                }
                                                else {
                                                    if (!ctxsave)
                                                        memcpy(&s->aes_keytable[keyindex*0x10 + 0xC], iv, 0x10);
                                                    else
                                                        memcpy(s->srk_iv, iv, 0x10);
                                                }
                                            }
                                        }

                                        se_log_hexdump("tegra_se_crypto_output", databuf_out, 0x10);
                                    }
                                }

                                qcrypto_cipher_free(cipher);
                            }
                        }
                    }
                }
                else if (enc_alg == 0x2) { // CONFIG_ENC_ALG RNG
                    datasize = (s->regs.SE_CRYPTO_LAST_BLOCK+1)*qcrypto_cipher_get_block_len(QCRYPTO_CIPHER_ALGO_AES_128);
                    if(datasize>databuf_outsize) datasize = databuf_outsize;
                    qemu_guest_getrandom(databuf_out, datasize, &err);
                    //se_log_hexdump("tegra_se_rng_output", databuf_out, datasize);
                }
                else if (enc_alg == 0x3) { // CONFIG_ENC_ALG SHA
                    datasize = s->regs.SE_SHA_MSG_LENGTH[0]/8;
                    s->regs.SE_SHA_MSG_LEFT[0] = 0;
                    QCryptoHashAlgo hash_alg = QCRYPTO_HASH_ALGO__MAX;
                    switch (enc_mode) {
                       case SE_CONFIG_ENC_MODE_SHA1:
                           hash_alg = QCRYPTO_HASH_ALGO_SHA1;
                       break;

                       case SE_CONFIG_ENC_MODE_SHA224:
                           hash_alg = QCRYPTO_HASH_ALGO_SHA224;
                       break;

                       case SE_CONFIG_ENC_MODE_SHA256:
                           hash_alg = QCRYPTO_HASH_ALGO_SHA256;
                       break;

                       case SE_CONFIG_ENC_MODE_SHA384:
                           hash_alg = QCRYPTO_HASH_ALGO_SHA384;
                       break;

                       case SE_CONFIG_ENC_MODE_SHA512:
                           hash_alg = QCRYPTO_HASH_ALGO_SHA512;
                       break;

                       default:
                           qemu_log_mask(LOG_UNIMP, "tegra.se: Unsupported SE hash alg: %u.\n", enc_mode);
                       break;
                   }

                   if (hash_alg!=QCRYPTO_HASH_ALGO__MAX) {
                       dma_addr_t tmplen = in_entry.size;
                       void* databuf = dma_memory_map(&address_space_memory, in_entry.address, &tmplen, DMA_DIRECTION_TO_DEVICE, tegra_se_get_memattrs(s));

                       uint8_t *result=NULL;
                       size_t resultlen=0;
                       if (databuf) {
                           if (datasize>tmplen) datasize = tmplen;

                           if (qcrypto_hash_bytes(hash_alg, databuf, datasize, &result, &resultlen, &err)==0) {
                               uint32_t *hashptr = databuf_out;
                               memset(hashptr, 0, databuf_outsize);
                               for (size_t i=0; i<resultlen && i<databuf_outsize; i+=sizeof(uint32_t)) {
                                   *hashptr = *((uint32_t*)&result[i]);
                                   if (cfg_dst!=0) bswap32s(hashptr);
                                   hashptr++;
                               }
                               //se_log_hexdump("tegra_se_sha_data", databuf, datasize);
                               //se_log_hexdump("tegra_se_sha_calchash", result, resultlen);
                               g_free(result);
                           }
                           else datasize = 0;
                           dma_memory_unmap(&address_space_memory, databuf, datasize, DMA_DIRECTION_TO_DEVICE, datasize);
                       }
                       else {
                           qemu_log_mask(LOG_GUEST_ERROR, "tegra.se: Failed to DMA map SE hash data buffer: 0x%x 0x%x.\n", in_entry.address, in_entry.size);
                       }
                   }
                }
                else if (enc_alg == 0x4) { // CONFIG_ENC_ALG RSA
                    uint32_t slot = (s->regs.SE_RSA_CONFIG>>24) & 0x1;

                    datasize = in_entry.size;
                    if (datasize > sizeof(indata)) datasize = sizeof(indata);
                    dma_memory_read(&address_space_memory, in_entry.address, indata, datasize, tegra_se_get_memattrs(s));
                    if (datasize > databuf_outsize) datasize = databuf_outsize;

                    s->regs.SE_INT_STATUS |= 1<<1; // INT_STATUS_IN_DONE
                    if (s->regs.SE_INT_ENABLE & s->regs.SE_INT_STATUS) TRACE_IRQ_RAISE(s->iomem.addr, s->irq);

                    n_size = (s->regs.SE_RSA_KEY_SIZE+1) * 0x40;
                    e_size = s->regs.SE_RSA_EXP_SIZE<<2;
                    if (n_size > sizeof(n_buf)) n_size = sizeof(n_buf);
                    if (e_size > sizeof(e_buf)) e_size = sizeof(e_buf);

                    byteswap_rsa((uint32_t*)indata, (uint32_t*)indata, sizeof(indata));
                    byteswap_rsa(&s->rsa_keytable[0x80*slot + 0x40], n_buf, n_size);
                    byteswap_rsa(&s->rsa_keytable[0x80*slot], e_buf, e_size);

                    #ifdef TEGRA_SE_RSA_DEBUG
                    se_log_hexdump("tegra_se_rsa_indata", indata, sizeof(indata));
                    se_log_hexdump("tegra_se_rsa_n_buf", n_buf, n_size);
                    se_log_hexdump("tegra_se_rsa_e_buf", e_buf, e_size);
                    #endif

                    QCryptoAkCipherOptions opts = { .alg = QCRYPTO_AK_CIPHER_ALGO_RSA };
                    opts.u.rsa.hash_alg = QCRYPTO_HASH_ALGO__MAX;
                    opts.u.rsa.padding_alg = QCRYPTO_RSA_PADDING_ALGO_RAW;
                    QCryptoAkCipher *akcipher = qcrypto_akcipher_new_raw(&opts, QCRYPTO_AK_CIPHER_KEY_TYPE_PUBLIC,
                                      (const uint8_t*)&n_buf, n_size,
                                      (const uint8_t*)&e_buf, e_size,
                                      &err);

                    if (akcipher==NULL) error_report_err(err);
                    else {
                        if (qcrypto_akcipher_encrypt(akcipher,
                             indata, sizeof(indata),
                             s->regs.SE_RSA_OUTPUT, sizeof(s->regs.SE_RSA_OUTPUT), &err)==-1) {
                             error_report_err(err);
                        }
                        else {
                            datasize = databuf_outsize;
                            if (datasize > sizeof(s->regs.SE_RSA_OUTPUT)) datasize = sizeof(s->regs.SE_RSA_OUTPUT);
                            byteswap_rsa(s->regs.SE_RSA_OUTPUT, databuf_out, datasize);
                            #ifdef TEGRA_SE_RSA_DEBUG
                            se_log_hexdump("tegra_se_rsa_outdata", databuf_out, datasize);
                            #endif
                        }
                        qcrypto_akcipher_free(akcipher);
                    }
                }
                if (databuf_in && !(ctxsave && ctxsave_src!=4)) dma_memory_unmap(&address_space_memory, databuf_in, tmplen_in, DMA_DIRECTION_TO_DEVICE, tmplen_in);
                if (cfg_dst==0 && databuf_out) dma_memory_unmap(&address_space_memory, databuf_out, databuf_outsize, DMA_DIRECTION_FROM_DEVICE, datasize);
                if (err) error_report_err(err);
            }
        break;

        case SE_CRYPTO_KEYTABLE_DATA_OFFSET:
            uint32_t aes_tableoffset = s->regs.SE_CRYPTO_KEYTABLE_ADDR & 0xff;
            uint32_t keyslot = aes_tableoffset>>4;
            if (tegra_se_check_aes_keyslot_write(s, aes_tableoffset)) // *Write
                s->aes_keytable[aes_tableoffset] = value;
            else
                qemu_log_mask(LOG_GUEST_ERROR, "tegra.se: Ignoring attempt to write AES keytable for keyslot %"PRIu32" since *Write is locked.\n", keyslot);
        break;

        case SE_RSA_KEYTABLE_ADDR_OFFSET:
            regs[offset/sizeof(uint32_t)] = value;
            if (value & 0x100) { // RSA_KEYTABLE_ADDR_INPUT_MODE = MEMORY
                n_size = (s->regs.SE_RSA_KEY_SIZE+1) * 0x40;
                e_size = s->regs.SE_RSA_EXP_SIZE<<2;
                if (n_size > 0x100) n_size = 0x100;
                if (e_size > 0x100) e_size = 0x100;

                uint32_t rsa_tableoffset = s->regs.SE_RSA_KEYTABLE_ADDR & 0xff;
                memset(&s->rsa_keytable[rsa_tableoffset], 0, 0x200);
                dma_memory_read(&address_space_memory, in_entry.address, &s->rsa_keytable[rsa_tableoffset + 0x40], n_size, tegra_se_get_memattrs(s));
                dma_memory_read(&address_space_memory, in_entry.address + 0x100, &s->rsa_keytable[rsa_tableoffset], e_size, tegra_se_get_memattrs(s));
            }
        break;

        case SE_RSA_KEYTABLE_DATA_OFFSET:
            uint32_t rsa_tableoffset = s->regs.SE_RSA_KEYTABLE_ADDR & 0xff;
            s->rsa_keytable[rsa_tableoffset] = value;
        break;

        case SE_CRYPTO_SECURITY_PERKEY_OFFSET ... SE_CRYPTO_SECURITY_PERKEY_OFFSET+sizeof(s->regs.SE_CRYPTO_SECURITY_PERKEY)-1:
            s->regs.SE_CRYPTO_SECURITY_PERKEY &= value;
        break;

        case SE_RSA_SECURITY_PERKEY_OFFSET ... SE_RSA_SECURITY_PERKEY_OFFSET+sizeof(s->regs.SE_RSA_SECURITY_PERKEY)-1:
            s->regs.SE_RSA_SECURITY_PERKEY &= value;
        break;

        case SE_RSA_KEYTABLE_ACCESS_OFFSET ... SE_RSA_KEYTABLE_ACCESS_OFFSET+sizeof(s->regs.SE_RSA_KEYTABLE_ACCESS)-1:
            s->regs.SE_RSA_KEYTABLE_ACCESS[(offset - SE_RSA_KEYTABLE_ACCESS_OFFSET) / 4] &= value;
        break;

        case SE_CRYPTO_KEYTABLE_ACCESS_OFFSET ... SE_CRYPTO_KEYTABLE_ACCESS_OFFSET+sizeof(s->regs.SE_CRYPTO_KEYTABLE_ACCESS)-1: // SE_CRYPTO_KEYTABLE_ACCESS
            value &= 0xFF;
            if (tegra_board < TEGRAX1PLUS_BOARD) value &= ~0x80; // Filter out DST_KEYTABLE_ONLY for non-TX1+.
            s->regs.SE_CRYPTO_KEYTABLE_ACCESS[(offset - SE_CRYPTO_KEYTABLE_ACCESS_OFFSET)>>2] &= value;
            if (tegra_board >= TEGRAX1PLUS_BOARD) s->regs.SE_CRYPTO_KEYTABLE_ACCESS[(offset - SE_CRYPTO_KEYTABLE_ACCESS_OFFSET)>>2] |= value & 0x80;
        break;

        case SE_TZRAM_OPERATION_OFFSET:
            if (tegra_board >= TEGRAX1PLUS_BOARD) {
                if (value & 0x1) { // REQ = INITIATE
                    value &= ~0x1;
                    value &= ~(1<<2); // BUSY = NO

                    // We don't really need to support TZRAM save/restore since qemu memory will persist during suspend anyway.
                }
                s->regs.SE_TZRAM_OPERATION = value;
            }
        break;

        default:
            regs[offset/sizeof(uint32_t)] = (regs[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;
        break;
    }
}

static uint64_t tegra_pka_priv_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    tegra_se *s = opaque;
    uint64_t ret = 0;

    TRACE_READ(s->iomem.addr, offset, ret);

    if (offset+size <= sizeof(s->regs_pka)) {
        uint32_t *regs = s->regs_pka;
        ret = regs[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);
    }

    return ret;
}

static void tegra_pka_priv_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    tegra_se *s = opaque;
    uint32_t keyslot=0;

    TRACE_WRITE(s->iomem.addr, offset, 0, value);

    if (offset+size <= sizeof(s->regs_pka)) {
        uint32_t *regs = s->regs_pka;

        switch (offset) {
            case 0x10 ... 0x10+0xC: // keyslot keydata FIFO
                keyslot = (offset - 0x10) >> 2;
                uint32_t addr = regs[0x2200 + keyslot]; // byteoff 0x8800
                addr = (addr & 0x7F) | ((addr & 0x300) >> 1);
                s->pka_keytable[((0x800*keyslot) >> 2) + addr] = value;
            break;

            default:
                regs[offset/sizeof(uint32_t)] = (regs[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;
            break;
        }
    }
}

static void tegra_se_priv_reset(DeviceState *dev)
{
    tegra_se *s = TEGRA_SE(dev);

    memset(&s->regs, 0, sizeof(s->regs));
    memset(s->aes_keytable, 0, sizeof(s->aes_keytable));
    memset(s->rsa_keytable, 0, sizeof(s->rsa_keytable));
    memset(s->srk, 0, sizeof(s->srk));
    memset(s->srk_iv, 0, sizeof(s->srk_iv));

    memset(&s->regs_pka, 0, sizeof(s->regs_pka));
    memset(&s->pka_keytable, 0, sizeof(s->pka_keytable));

    s->regs._0x814 = 0x89040800;

    s->regs.SE_SE_SECURITY = 0x00010005; // SECURITY_HARD_SETTING = NONSECURE, SECURITY_PERKEY_SETTING = NONSECURE, SECURITY_SOFT_SETTING = NONSECURE
    if (tegra_board >= TEGRAX1PLUS_BOARD) s->regs.SE_SE_SECURITY |= 1<<5; // SE_SECURITY TX1+ sticky bit
    s->regs.SE_TZRAM_SECURITY = 0x1; // NONSECURE
    s->regs.SE_CRYPTO_SECURITY_PERKEY = 0xFFFF;

    memset(s->aes_dataoverride, 0, sizeof(s->aes_dataoverride));
    memset(s->aes_dataoverride_enable, 0, sizeof(s->aes_dataoverride_enable));

    Error *err = NULL;
    char tmpstr[17]={};
    for (int keyslot=0; keyslot<16; keyslot++) { // Specifying an input secret for every keyslot is not required, it will use the memset data below if not specified. These secrets are only needed when starting emulation post-bootrom.
        memset(&s->aes_keytable[keyslot*0x10], 0x01*(keyslot+1), 0x10);
        s->regs.SE_CRYPTO_KEYTABLE_ACCESS[keyslot] = 0x7F;

        uint8_t *data=NULL;
        size_t datalen = 0;
        size_t keylen = 0;
        snprintf(tmpstr, sizeof(tmpstr)-1, "se.aeskeyslot%d", keyslot);
        if (qcrypto_secret_lookup(tmpstr, &data, &datalen, &err)==0) {
            if (datalen<0x10 || datalen>0x40) error_setg(&err, "SE: Invalid datalen for secret %s, datalen=0x%zx expected 0x10-0x40.", tmpstr, datalen);
            else {
                keylen = MIN(datalen, 0x20);
                memcpy(&s->aes_keytable[keyslot*0x10], data, keylen);
                qemu_hexdump(stdout, tmpstr, &s->aes_keytable[keyslot*0x10], keylen);
                if (datalen==0x40) {
                    s->aes_dataoverride_enable[keyslot] = 1;
                    memcpy(&s->aes_dataoverride[keyslot*0x8], &data[0x20], 0x20);
                }
            }
            g_free(data);
        }
        if (err) {
            error_report_err(err);
            err = NULL;
        }
    }

    s->regs.SE_RSA_SECURITY_PERKEY = 0x3;
    for (int keyslot=0; keyslot<2; keyslot++) s->regs.SE_RSA_KEYTABLE_ACCESS[keyslot] = 0x7;

    s->regs_pka[0x8114>>2] = 0x1;
}

static const MemoryRegionOps tegra_se_mem_ops = {
    .read = tegra_se_priv_read,
    .write = tegra_se_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps tegra_pka_mem_ops = {
    .read = tegra_pka_priv_read,
    .write = tegra_pka_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_se_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_se *s = TEGRA_SE(dev);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_se_mem_ops, s,
                          "tegra.se", TEGRA_SE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    if (s->engine==2) {
        memory_region_init_io(&s->iomem_pka, OBJECT(dev), &tegra_pka_mem_ops, s,
                              "tegra.pka", TEGRA_PKA1_SIZE);
        sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem_pka);
    }

    memset(&s->regs, 0, sizeof(s->regs));
    memset(&s->regs_pka, 0, sizeof(s->regs_pka));
}

static void tegra_se_init(Object *obj)
{
    tegra_se *s = TEGRA_SE(obj);

    for (int keyslot=0; keyslot<16; keyslot++) s->aes_keyslots_lock[keyslot] = 0xFF;
}

static Property tegra_se_properties[] = {
    DEFINE_PROP_UINT32("engine", tegra_se, engine, 1), \
    DEFINE_PROP_END_OF_LIST(),
};

static void tegra_se_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, tegra_se_properties);
    dc->realize = tegra_se_priv_realize;
    dc->vmsd = &vmstate_tegra_se;
    device_class_set_legacy_reset(dc, tegra_se_priv_reset);

    object_class_property_add(klass, "aes-keyslots-lock", "uint",
                              NULL,
                              tegra_se_set_aes_keyslots_lock, NULL, NULL);
}

static const TypeInfo tegra_se_info = {
    .name = TYPE_TEGRA_SE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_se),
    .class_init = tegra_se_class_init,
    .instance_init = tegra_se_init,
};

static void tegra_se_register_types(void)
{
    type_register_static(&tegra_se_info);
}

type_init(tegra_se_register_types)
