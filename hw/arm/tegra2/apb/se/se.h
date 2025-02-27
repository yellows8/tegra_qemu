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

// This uses various definitions from: https://github.com/Atmosphere-NX/Atmosphere/blob/master/libraries/libexosphere/source/se/se_registers.hpp

#ifndef TEGRA_SE_H
#define TEGRA_SE_H

#define SE_TZRAM_SECURITY_OFFSET 0x4
#define SE_OPERATION_OFFSET 0x8
#define SE_INT_ENABLE_OFFSET 0xC
#define SE_INT_STATUS_OFFSET 0x10
#define SE_OUT_LL_ADDR_OFFSET 0x24
#define SE_CRYPTO_SECURITY_PERKEY_OFFSET 0x280
#define SE_CRYPTO_KEYTABLE_ACCESS_OFFSET 0x284
#define SE_CRYPTO_KEYTABLE_DATA_OFFSET 0x320
#define SE_RSA_SECURITY_PERKEY_OFFSET 0x40C
#define SE_RSA_KEYTABLE_ACCESS_OFFSET 0x410
#define SE_RSA_KEYTABLE_ADDR_OFFSET 0x420
#define SE_RSA_KEYTABLE_DATA_OFFSET 0x424
#define SE_TZRAM_OPERATION_OFFSET 0x540

enum SE_CONFIG_ENC_MODE {
    SE_CONFIG_ENC_MODE_AESMODE_KEY128 = 0,
    SE_CONFIG_ENC_MODE_AESMODE_KEY192 = 1,
    SE_CONFIG_ENC_MODE_AESMODE_KEY256 = 2,

    SE_CONFIG_ENC_MODE_SHA1   = 1,
    SE_CONFIG_ENC_MODE_SHA224 = 4,
    SE_CONFIG_ENC_MODE_SHA256 = 5,
    SE_CONFIG_ENC_MODE_SHA384 = 6,
    SE_CONFIG_ENC_MODE_SHA512 = 7,
};

int tegra_se_crypto_operation(void *opaque, void* key, void* iv, QCryptoCipherAlgo cipher_alg, QCryptoCipherMode mode, bool encrypt, uintptr_t inbuf, uintptr_t outbuf, dma_addr_t databuf_size, bool inbuf_host, bool outbuf_host);

void tegra_se_lock_aes_keyslot(uint32_t slot, uint32_t flags);

void tegra_se_set_aes_keyslot(uint32_t slot, void* key, size_t key_size);

#endif // TEGRA_SE_H
