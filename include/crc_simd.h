/* CRC SIMD Acceleration Header
 * Copyright (c) 2024
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * SIMD-accelerated CRC computation using:
 * - PCLMULQDQ on x86/x64 (Intel/AMD)
 * - PMULL on ARM64/NEON (Apple Silicon, ARM servers)
 */

#ifndef CRC_SIMD_H
#define CRC_SIMD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Feature detection */
bool crc_simd_available(void);

/* CRC16 SIMD functions - use CRC-16-CCITT polynomial 0x1021 */
void crc16_simd_init(void);
uint16_t crc16_simd(uint16_t crc, const void *data, uint64_t len);

/* CRC16 with integrated 24-bit unpacking.
 * Input is packed 24-bit words (3 bytes each). Computes CRC as if each word
 * were zero-extended to 32 bits (upper byte = 0x00), matching the CRC the
 * transmitter calculated on the unpacked representation.
 * packed_len must be divisible by 3. */
uint16_t crc16_simd_packed24(uint16_t crc, const void *packed_data,
                             uint64_t packed_len);

#endif /* CRC_SIMD_H */
