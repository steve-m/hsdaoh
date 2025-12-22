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

/* CRC64 SIMD functions - use Jones polynomial 0xad93d23594c935a9 */
void crc64_simd_init(void);
uint64_t crc64_simd(uint64_t crc, const void *data, uint64_t len);

/* CRC16 SIMD functions - use CRC-16-CCITT polynomial 0x1021 */
void crc16_simd_init(void);
uint16_t crc16_simd(uint16_t crc, const void *data, uint64_t len);

#endif /* CRC_SIMD_H */
