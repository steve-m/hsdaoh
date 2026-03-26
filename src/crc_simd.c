/* CRC SIMD Acceleration Implementation
 * Copyright (c) 2024
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * SIMD-accelerated CRC computation using PCLMULQDQ (x86) or PMULL (ARM64)
 * Based on Intel's "Fast CRC Computation for Generic Polynomials Using
 * PCLMULQDQ Instruction" whitepaper and Intel ISA-L implementation.
 *
 * Supports:
 * - CRC16-CCITT: polynomial 0x1021 (non-reflected)
 *
 * References:
 * - Intel whitepaper: Fast CRC Computation for Generic Polynomials
 * - Intel ISA-L: https://github.com/intel/isa-l
 */

#include "crc_simd.h"
#include <string.h>

/* ============================================================================
 * Architecture Detection and Feature Testing
 * ============================================================================
 */

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) ||             \
    defined(_M_IX86)
#define ARCH_X86 1
#if defined(__GNUC__) || defined(__clang__)
#include <cpuid.h>
#endif
#include <immintrin.h>
#include <wmmintrin.h>
#elif defined(__aarch64__) || defined(_M_ARM64)
#define ARCH_ARM64 1
#include <arm_neon.h>
#if defined(__ARM_FEATURE_CRYPTO) || defined(__APPLE__)
#define HAS_PMULL 1
#include <arm_acle.h>
#endif
#endif

static bool g_simd_checked = false;
static bool g_simd_available = false;

bool crc_simd_available(void) {
    if (g_simd_checked) {
        return g_simd_available;
    }
    g_simd_checked = true;

#if defined(ARCH_X86)
#if defined(__GNUC__) || defined(__clang__)
    unsigned int eax, ebx, ecx, edx;
    if (__get_cpuid(1, &eax, &ebx, &ecx, &edx)) {
        /* Check for PCLMULQDQ (bit 1 of ECX) and SSE4.1 (bit 19 of ECX) */
        g_simd_available = (ecx & (1 << 1)) && (ecx & (1 << 19));
    }
#endif
#elif defined(ARCH_ARM64)
#if defined(__APPLE__)
    /* Apple Silicon always has PMULL */
    g_simd_available = true;
#elif defined(HAS_PMULL)
    g_simd_available = true;
#else
    g_simd_available = false;
#endif
#else
    g_simd_available = false;
#endif

    return g_simd_available;
}

/* ============================================================================
 * CRC16-CCITT Polynomial: 0x1021 (non-reflected)
 *
 * Constants computed by gen_constants.c
 * ============================================================================
 */

/* 16-byte folding constants (gap = 64 between hi and lo for 64-bit CLMUL lanes) */
#define CRC16_K_HI UINT64_C(0x650b)  /* x^192 mod P */
#define CRC16_K_LO UINT64_C(0xaefc)  /* x^128 mod P */

/* 128-byte stride folding constants (8-way parallel, 1024-bit fold distance) */
#define CRC16_K128_HI UINT64_C(0x71c4)  /* x^1088 mod P */
#define CRC16_K128_LO UINT64_C(0x36c4)  /* x^1024 mod P */

/* 8-way reduction constants: fold 8 accumulators down to 1.
 * Each pair folds one accumulator across the given byte distance.
 * Convention: k_hi = x^(D+64) mod P, k_lo = x^D mod P. */
#define CRC16_FOLD112_HI UINT64_C(0xd24c)  /* x^960 mod P */
#define CRC16_FOLD112_LO UINT64_C(0xcbc5)  /* x^896 mod P */
#define CRC16_FOLD96_HI  UINT64_C(0x0447)  /* x^832 mod P */
#define CRC16_FOLD96_LO  UINT64_C(0x106f)  /* x^768 mod P */
#define CRC16_FOLD80_HI  UINT64_C(0x87b3)  /* x^704 mod P */
#define CRC16_FOLD80_LO  UINT64_C(0xda35)  /* x^640 mod P */
#define CRC16_FOLD64_HI  UINT64_C(0x8832)  /* x^576 mod P */
#define CRC16_FOLD64_LO  UINT64_C(0x13fc)  /* x^512 mod P */
#define CRC16_FOLD48_HI  UINT64_C(0x2535)  /* x^448 mod P */
#define CRC16_FOLD48_LO  UINT64_C(0xcde2)  /* x^384 mod P */
#define CRC16_FOLD32_HI  UINT64_C(0x26aa)  /* x^320 mod P */
#define CRC16_FOLD32_LO  UINT64_C(0x8e29)  /* x^256 mod P */

/* ============================================================================
 * X86/X64 PCLMULQDQ Implementation
 * ============================================================================
 */

#if defined(ARCH_X86)

/* ============================================================================
 * CRC16 SIMD Implementation (x86/x64 with PCLMULQDQ)
 *
 * CRC16-CCITT (polynomial 0x1021) is a non-reflected CRC.
 *
 * Non-reflected CRCs process data MSB-first, which requires:
 * 1. Byte-swapping data to get correct byte order for PCLMULQDQ
 * 2. Positioning CRC in high bits
 * 3. Different folding constant interpretation
 *
 * This implementation uses the marzooqy/crc-clmul approach:
 * - Data is byte-swapped (reversed within 128-bit blocks)
 * - CRC goes in high 64 bits
 * - For non-reflected: PCLMULQDQ multiplies acc_hi*k_hi and acc_lo*k_lo
 * - Final reduction via table lookup
 *
 * Folding constants for CRC16-CCITT polynomial 0x1021:
 * - x^192 mod P = 0x650b (for high 64-bit lane)
 * - x^128 mod P = 0xaefc (for low 64-bit lane)
 * ============================================================================
 */

/* CRC16-CCITT folding constants for 128-bit blocks (aliased from global) */
#define CRC16_K_HI_X86 CRC16_K_HI
#define CRC16_K_LO_X86 CRC16_K_LO

/* Byte-swap 128-bit register: reverse all 16 bytes */
static inline __m128i crc16_bswap_128_x86(__m128i x) {
    const __m128i mask =
        _mm_setr_epi8(15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0);
    return _mm_shuffle_epi8(x, mask);
}

/* Fold helper for non-reflected CRC16 on x86
 * For non-reflected: we multiply acc_hi by k_hi, acc_lo by k_lo
 * Constants packed as: k_hi in high 64 bits, k_lo in low 64 bits
 */
static inline __m128i crc16_fold_x86(__m128i acc, __m128i data, __m128i k) {
    /* Non-reflected folding:
     * - acc_hi (high 64) gets multiplied by k_hi (high 64) -> use 0x11
     * - acc_lo (low 64) gets multiplied by k_lo (low 64) -> use 0x00
     */
    __m128i t1 = _mm_clmulepi64_si128(acc, k, 0x11); /* acc_hi * k_hi */
    __m128i t2 = _mm_clmulepi64_si128(acc, k, 0x00); /* acc_lo * k_lo */
    return _mm_xor_si128(_mm_xor_si128(t1, t2), data);
}

uint16_t crc16_simd(uint16_t crc, const void *data, uint64_t len) {
    const uint8_t *buf = (const uint8_t *)data;
    extern uint16_t crc16speed(uint16_t crc, const void *s, const uint64_t l);

    /* Use table-based for small buffers - SIMD overhead not worth it */
    if (!crc_simd_available() || len < 64) {
        return crc16speed(crc, data, len);
    }

    /* 16-byte fold constants */
    const __m128i k16 = _mm_set_epi64x(CRC16_K_HI_X86, CRC16_K_LO_X86);
    __m128i acc;
    uint64_t remaining;

    /* ----------------------------------------------------------------
     * 8-way parallel folding for large buffers (>= 256 bytes)
     *
     * Uses 8 independent XMM accumulators processing 128 bytes per
     * iteration. This hides PCLMULQDQ latency (7-8 cycles) since the
     * CPU can pipeline 8 independent multiply operations per loop.
     * Inspired by crc-fast-rust / Intel ISA-L approach.
     * ---------------------------------------------------------------- */
    if (len >= 256) {
        const __m128i k128 = _mm_set_epi64x(CRC16_K128_HI, CRC16_K128_LO);

        /* Load and byte-swap first 128 bytes into 8 accumulators */
        __m128i a0 = crc16_bswap_128_x86(_mm_loadu_si128((const __m128i *)(buf +   0)));
        __m128i a1 = crc16_bswap_128_x86(_mm_loadu_si128((const __m128i *)(buf +  16)));
        __m128i a2 = crc16_bswap_128_x86(_mm_loadu_si128((const __m128i *)(buf +  32)));
        __m128i a3 = crc16_bswap_128_x86(_mm_loadu_si128((const __m128i *)(buf +  48)));
        __m128i a4 = crc16_bswap_128_x86(_mm_loadu_si128((const __m128i *)(buf +  64)));
        __m128i a5 = crc16_bswap_128_x86(_mm_loadu_si128((const __m128i *)(buf +  80)));
        __m128i a6 = crc16_bswap_128_x86(_mm_loadu_si128((const __m128i *)(buf +  96)));
        __m128i a7 = crc16_bswap_128_x86(_mm_loadu_si128((const __m128i *)(buf + 112)));

        /* XOR initial CRC into high 16 bits of first accumulator */
        a0 = _mm_xor_si128(a0, _mm_set_epi64x((uint64_t)crc << 48, 0));

        buf += 128;
        remaining = len - 128;

        /* Main 8-way folding loop: 128 bytes per iteration.
         * Loads are separated from folds for better instruction scheduling. */
        while (remaining >= 128) {
            _mm_prefetch((const char *)(buf + 256), _MM_HINT_T0);
            __m128i d0 = crc16_bswap_128_x86(_mm_loadu_si128((const __m128i *)(buf +   0)));
            __m128i d1 = crc16_bswap_128_x86(_mm_loadu_si128((const __m128i *)(buf +  16)));
            __m128i d2 = crc16_bswap_128_x86(_mm_loadu_si128((const __m128i *)(buf +  32)));
            __m128i d3 = crc16_bswap_128_x86(_mm_loadu_si128((const __m128i *)(buf +  48)));
            __m128i d4 = crc16_bswap_128_x86(_mm_loadu_si128((const __m128i *)(buf +  64)));
            __m128i d5 = crc16_bswap_128_x86(_mm_loadu_si128((const __m128i *)(buf +  80)));
            __m128i d6 = crc16_bswap_128_x86(_mm_loadu_si128((const __m128i *)(buf +  96)));
            __m128i d7 = crc16_bswap_128_x86(_mm_loadu_si128((const __m128i *)(buf + 112)));
            a0 = crc16_fold_x86(a0, d0, k128);
            a1 = crc16_fold_x86(a1, d1, k128);
            a2 = crc16_fold_x86(a2, d2, k128);
            a3 = crc16_fold_x86(a3, d3, k128);
            a4 = crc16_fold_x86(a4, d4, k128);
            a5 = crc16_fold_x86(a5, d5, k128);
            a6 = crc16_fold_x86(a6, d6, k128);
            a7 = crc16_fold_x86(a7, d7, k128);
            buf += 128;
            remaining -= 128;
        }

        /* Fold 8 accumulators down to 1.
         * Each accumulator a[i] is folded across the distance to a[7]:
         *   a0: 112 bytes, a1: 96 bytes, ... a6: 16 bytes, a7: target */
        __m128i res = a7;
        res = crc16_fold_x86(a0, res, _mm_set_epi64x(CRC16_FOLD112_HI, CRC16_FOLD112_LO));
        res = crc16_fold_x86(a1, res, _mm_set_epi64x(CRC16_FOLD96_HI,  CRC16_FOLD96_LO));
        res = crc16_fold_x86(a2, res, _mm_set_epi64x(CRC16_FOLD80_HI,  CRC16_FOLD80_LO));
        res = crc16_fold_x86(a3, res, _mm_set_epi64x(CRC16_FOLD64_HI,  CRC16_FOLD64_LO));
        res = crc16_fold_x86(a4, res, _mm_set_epi64x(CRC16_FOLD48_HI,  CRC16_FOLD48_LO));
        res = crc16_fold_x86(a5, res, _mm_set_epi64x(CRC16_FOLD32_HI,  CRC16_FOLD32_LO));
        res = crc16_fold_x86(a6, res, k16);  /* 16 bytes = same as regular fold */

        acc = res;
    } else {
        /* For 64-255 bytes: single-accumulator fold */
        acc = crc16_bswap_128_x86(_mm_loadu_si128((const __m128i *)buf));
        acc = _mm_xor_si128(acc, _mm_set_epi64x((uint64_t)crc << 48, 0));
        buf += 16;
        remaining = len - 16;
    }

    /* Continue with single-accumulator fold for remaining 16-byte blocks */
    while (remaining >= 16) {
        __m128i data_vec =
            crc16_bswap_128_x86(_mm_loadu_si128((const __m128i *)buf));
        acc = crc16_fold_x86(acc, data_vec, k16);
        buf += 16;
        remaining -= 16;
    }

    /* Store folded result (byte-swap back to original order) */
    __m128i acc_swapped = crc16_bswap_128_x86(acc);
    uint8_t final_buf[32];
    _mm_storeu_si128((__m128i *)final_buf, acc_swapped);

    /* Process remaining bytes through table lookup */
    if (remaining > 0) {
        memcpy(final_buf + 16, buf, remaining);
        return crc16speed(0, final_buf, 16 + remaining);
    } else {
        return crc16speed(0, final_buf, 16);
    }
}

/* ============================================================================
 * CRC16 with integrated 24-bit -> 32-bit unpacking (x86/x64 PCLMULQDQ)
 *
 * Problem:
 *   Data arrives from a device as packed 24-bit words (3 bytes each).
 *   The transmitter computed CRC on the unpacked (zero-extended to 32-bit)
 *   representation: [b0, b1, b2, 0x00, b3, b4, b5, 0x00, ...].
 *   We need to reproduce this CRC without first copying into an unpacked buffer.
 *
 * Approach:
 *   Use SSSE3 _mm_shuffle_epi8 to perform the 24->32 bit expansion and the
 *   byte-swap (needed for non-reflected CRC16) in a single instruction.
 *
 *   Each iteration loads 12 packed bytes (4 × 3-byte words) and shuffles them
 *   into 16 unpacked+byte-swapped bytes, which are then fed directly into the
 *   existing PCLMULQDQ folding pipeline.
 *
 *   Packed input (12 bytes used out of 16-byte load):
 *     [A0 A1 A2 | B0 B1 B2 | C0 C1 C2 | D0 D1 D2 | xx xx xx xx]
 *
 *   Unpacked (what the CRC should see, little-endian uint32_t):
 *     [A0 A1 A2 00 | B0 B1 B2 00 | C0 C1 C2 00 | D0 D1 D2 00]
 *
 *   After byte-swap for non-reflected CRC (reverse all 16 bytes):
 *     [00 D2 D1 D0 | 00 C2 C1 C0 | 00 B2 B1 B0 | 00 A2 A1 A0]
 *
 *   Combined shuffle mask (one _mm_shuffle_epi8 does both transforms):
 *     {0x80, 11, 10, 9, 0x80, 8, 7, 6, 0x80, 5, 4, 3, 0x80, 2, 1, 0}
 *     (0x80 = zero the byte, other values = source byte index)
 *
 * Tail handling:
 *   Remaining packed bytes (< 16 safe-load boundary) are unpacked with scalar
 *   code and appended to the folded accumulator for final reduction via the
 *   table-based crc16speed().
 * ============================================================================
 */

/* Scalar fallback: unpack packed 24-bit words and compute CRC via table.
 * Used for small buffers and for tail bytes after the SIMD loop. */
static uint16_t crc16_packed24_scalar(uint16_t crc, const uint8_t *buf,
                                      uint64_t packed_len) {
    extern uint16_t crc16speed(uint16_t crc, const void *s, const uint64_t l);
    uint8_t batch[16];
    uint64_t words = packed_len / 3;
    uint64_t i = 0;

    /* Process 4 words (12 packed -> 16 unpacked bytes) at a time */
    for (; i + 4 <= words; i += 4) {
        for (int j = 0; j < 4; j++) {
            batch[j * 4 + 0] = buf[(i + j) * 3 + 0];
            batch[j * 4 + 1] = buf[(i + j) * 3 + 1];
            batch[j * 4 + 2] = buf[(i + j) * 3 + 2];
            batch[j * 4 + 3] = 0;
        }
        crc = crc16speed(crc, batch, 16);
    }

    /* Handle remaining 1-3 words */
    uint64_t rem = words - i;
    for (uint64_t j = 0; j < rem; j++) {
        batch[j * 4 + 0] = buf[(i + j) * 3 + 0];
        batch[j * 4 + 1] = buf[(i + j) * 3 + 1];
        batch[j * 4 + 2] = buf[(i + j) * 3 + 2];
        batch[j * 4 + 3] = 0;
    }
    if (rem > 0) {
        crc = crc16speed(crc, batch, rem * 4);
    }
    return crc;
}

uint16_t crc16_simd_packed24(uint16_t crc, const void *packed_data,
                             uint64_t packed_len) {
    extern uint16_t crc16speed(uint16_t crc, const void *s, const uint64_t l);
    const uint8_t *buf = (const uint8_t *)packed_data;
    const uint8_t *end = buf + packed_len;

    /* Scalar fallback for small buffers or no SIMD.
     * 48 packed bytes = 16 words = 64 unpacked bytes (matches regular threshold).
     * Also guarantees at least 4 SIMD blocks and safe 16-byte loads. */
    if (!crc_simd_available() || packed_len < 48) {
        return crc16_packed24_scalar(crc, buf, packed_len);
    }

    /* Combined unpack (24->32 bit zero-extend) + byte-swap shuffle mask.
     * Reads 12 packed bytes from a 16-byte load, outputs 16 unpacked+swapped bytes. */
    const __m128i unpack_bswap = _mm_setr_epi8(
        (char)0x80, 11, 10, 9,   /* word D byte-swapped, zero-extended */
        (char)0x80,  8,  7, 6,   /* word C */
        (char)0x80,  5,  4, 3,   /* word B */
        (char)0x80,  2,  1, 0    /* word A */
    );

    /* Helper macro: load 12 packed bytes, unpack+bswap to 16 bytes */
    #define UNPACK_LOAD(ptr) \
        _mm_shuffle_epi8(_mm_loadu_si128((const __m128i *)(ptr)), unpack_bswap)

    const __m128i k16 = _mm_set_epi64x(CRC16_K_HI_X86, CRC16_K_LO_X86);
    __m128i acc;
    uint64_t remaining_packed;

    /* ----------------------------------------------------------------
     * 8-way parallel folding for packed24 (>= 192 packed bytes = 256 unpacked)
     *
     * 128 unpacked bytes = 32 words × 3 bytes = 96 packed bytes per 8-way block.
     * Need ≥ 2 blocks (192 packed bytes) plus safe 16-byte loads.
     * Threshold: 192 + 16 = 208, round up to guarantee ≥ 2 full iterations.
     * ---------------------------------------------------------------- */
    if (packed_len >= 208) {
        const __m128i k128 = _mm_set_epi64x(CRC16_K128_HI, CRC16_K128_LO);

        /* Load first 96 packed bytes -> 8 × 16 unpacked bytes */
        __m128i a0 = UNPACK_LOAD(buf +  0);
        __m128i a1 = UNPACK_LOAD(buf + 12);
        __m128i a2 = UNPACK_LOAD(buf + 24);
        __m128i a3 = UNPACK_LOAD(buf + 36);
        __m128i a4 = UNPACK_LOAD(buf + 48);
        __m128i a5 = UNPACK_LOAD(buf + 60);
        __m128i a6 = UNPACK_LOAD(buf + 72);
        __m128i a7 = UNPACK_LOAD(buf + 84);

        a0 = _mm_xor_si128(a0, _mm_set_epi64x((uint64_t)crc << 48, 0));
        buf += 96;

        /* 8-way folding loop: 96 packed bytes per iteration.
         * Each load needs 12 packed bytes but does a 16-byte SIMD load,
         * so the last load at buf+84 reads up to buf+100. Need buf+100 <= end. */
        while (buf + 100 <= end) {
            a0 = crc16_fold_x86(a0, UNPACK_LOAD(buf +  0), k128);
            a1 = crc16_fold_x86(a1, UNPACK_LOAD(buf + 12), k128);
            a2 = crc16_fold_x86(a2, UNPACK_LOAD(buf + 24), k128);
            a3 = crc16_fold_x86(a3, UNPACK_LOAD(buf + 36), k128);
            a4 = crc16_fold_x86(a4, UNPACK_LOAD(buf + 48), k128);
            a5 = crc16_fold_x86(a5, UNPACK_LOAD(buf + 60), k128);
            a6 = crc16_fold_x86(a6, UNPACK_LOAD(buf + 72), k128);
            a7 = crc16_fold_x86(a7, UNPACK_LOAD(buf + 84), k128);
            buf += 96;
        }

        /* Fold 8 accumulators down to 1 */
        __m128i res = a7;
        res = crc16_fold_x86(a0, res, _mm_set_epi64x(CRC16_FOLD112_HI, CRC16_FOLD112_LO));
        res = crc16_fold_x86(a1, res, _mm_set_epi64x(CRC16_FOLD96_HI,  CRC16_FOLD96_LO));
        res = crc16_fold_x86(a2, res, _mm_set_epi64x(CRC16_FOLD80_HI,  CRC16_FOLD80_LO));
        res = crc16_fold_x86(a3, res, _mm_set_epi64x(CRC16_FOLD64_HI,  CRC16_FOLD64_LO));
        res = crc16_fold_x86(a4, res, _mm_set_epi64x(CRC16_FOLD48_HI,  CRC16_FOLD48_LO));
        res = crc16_fold_x86(a5, res, _mm_set_epi64x(CRC16_FOLD32_HI,  CRC16_FOLD32_LO));
        res = crc16_fold_x86(a6, res, k16);
        acc = res;
    } else {
        /* Single-accumulator fold for smaller inputs */
        acc = UNPACK_LOAD(buf);
        acc = _mm_xor_si128(acc, _mm_set_epi64x((uint64_t)crc << 48, 0));
        buf += 12;
    }

    /* Continue with single-accumulator fold for remaining 12-packed-byte blocks */
    while (buf + 16 <= end) {
        acc = crc16_fold_x86(acc, UNPACK_LOAD(buf), k16);
        buf += 12;
    }

    #undef UNPACK_LOAD

    /* Final reduction via byte-swap + table lookup */
    __m128i acc_swapped = crc16_bswap_128_x86(acc);
    uint8_t final_buf[48];
    _mm_storeu_si128((__m128i *)final_buf, acc_swapped);

    /* Unpack remaining packed bytes (scalar) into final_buf */
    remaining_packed = end - buf;
    uint64_t remaining_words = remaining_packed / 3;
    for (uint64_t i = 0; i < remaining_words; i++) {
        final_buf[16 + i * 4 + 0] = buf[i * 3 + 0];
        final_buf[16 + i * 4 + 1] = buf[i * 3 + 1];
        final_buf[16 + i * 4 + 2] = buf[i * 3 + 2];
        final_buf[16 + i * 4 + 3] = 0;
    }

    return crc16speed(0, final_buf, 16 + remaining_words * 4);
}

void crc16_simd_init(void) {
    crc_simd_available();
}

#elif defined(ARCH_ARM64) && defined(HAS_PMULL)

/* ============================================================================
 * ARM64 NEON/PMULL Implementation
 * ============================================================================
 */


/* ============================================================================
 * CRC16 SIMD Implementation for ARM64
 *
 * CRC16-CCITT (polynomial 0x1021) is a non-reflected CRC.
 *
 * Non-reflected CRCs process data MSB-first, which requires:
 * 1. Byte-swapping data to get correct byte order for PMULL
 * 2. Positioning CRC in high bits
 * 3. Different folding constant interpretation
 *
 * This implementation uses the marzooqy/crc-clmul approach:
 * - Data is byte-swapped (reversed within 128-bit blocks)
 * - CRC goes in high 64 bits: SET(crc << 48, 0)
 * - For non-reflected: PMULL immediates swap acc_hi*k1 and acc_lo*k2
 * - Final reduction via table lookup
 *
 * Folding constants for CRC16-CCITT polynomial 0x1021:
 * - x^192 mod P = 0x650b (for high 64-bit lane)
 * - x^128 mod P = 0xaefc (for low 64-bit lane)
 * ============================================================================
 */

/* Byte-swap 128 bits: reverse all 16 bytes */
static inline uint8x16_t crc16_bswap_128(uint8x16_t x) {
    /* Reverse bytes within each 64-bit lane, then swap lanes */
    uint8x16_t rev = vrev64q_u8(x);
    return vcombine_u8(vget_high_u8(rev), vget_low_u8(rev));
}

/* Fold helper for non-reflected CRC16
 * For non-reflected: we multiply acc_hi by k_hi, acc_lo by k_lo
 * This is different from reflected where we use 0x00 and 0x11 immediates
 */
static inline uint64x2_t crc16_fold_neon(uint64x2_t acc, uint64x2_t data,
                                         uint64_t k_hi, uint64_t k_lo) {
    /* Non-reflected folding:
     * - acc_hi (lane 1) gets multiplied by k_hi
     * - acc_lo (lane 0) gets multiplied by k_lo
     */
    poly128_t t1 = vmull_p64(vgetq_lane_u64(acc, 1), k_hi); /* acc_hi * k_hi */
    poly128_t t2 = vmull_p64(vgetq_lane_u64(acc, 0), k_lo); /* acc_lo * k_lo */
    uint64x2_t t1_vec = vreinterpretq_u64_p128(t1);
    uint64x2_t t2_vec = vreinterpretq_u64_p128(t2);
    return veorq_u64(veorq_u64(t1_vec, t2_vec), data);
}

/* Helper: load 16 bytes, byte-swap */
static inline uint64x2_t crc16_load_bswap_neon(const uint8_t *ptr) {
    return vreinterpretq_u64_u8(crc16_bswap_128(vld1q_u8(ptr)));
}

uint16_t crc16_simd(uint16_t crc, const void *data, uint64_t len) {
    extern uint16_t crc16speed(uint16_t crc, const void *s, const uint64_t l);
    const uint8_t *buf = (const uint8_t *)data;

    /* Use table-based for small buffers - SIMD overhead not worth it */
    if (!crc_simd_available() || len < 64) {
        return crc16speed(crc, data, len);
    }

    uint64x2_t acc;
    uint64_t remaining;

    /* 8-way parallel folding for large buffers (>= 256 bytes) */
    if (len >= 256) {
        uint64x2_t a0 = crc16_load_bswap_neon(buf +   0);
        uint64x2_t a1 = crc16_load_bswap_neon(buf +  16);
        uint64x2_t a2 = crc16_load_bswap_neon(buf +  32);
        uint64x2_t a3 = crc16_load_bswap_neon(buf +  48);
        uint64x2_t a4 = crc16_load_bswap_neon(buf +  64);
        uint64x2_t a5 = crc16_load_bswap_neon(buf +  80);
        uint64x2_t a6 = crc16_load_bswap_neon(buf +  96);
        uint64x2_t a7 = crc16_load_bswap_neon(buf + 112);

        a0 = veorq_u64(a0, vsetq_lane_u64((uint64_t)crc << 48, vdupq_n_u64(0), 1));

        buf += 128;
        remaining = len - 128;

        /* Main 8-way folding loop: 128 bytes per iteration */
        while (remaining >= 128) {
            a0 = crc16_fold_neon(a0, crc16_load_bswap_neon(buf +   0), CRC16_K128_HI, CRC16_K128_LO);
            a1 = crc16_fold_neon(a1, crc16_load_bswap_neon(buf +  16), CRC16_K128_HI, CRC16_K128_LO);
            a2 = crc16_fold_neon(a2, crc16_load_bswap_neon(buf +  32), CRC16_K128_HI, CRC16_K128_LO);
            a3 = crc16_fold_neon(a3, crc16_load_bswap_neon(buf +  48), CRC16_K128_HI, CRC16_K128_LO);
            a4 = crc16_fold_neon(a4, crc16_load_bswap_neon(buf +  64), CRC16_K128_HI, CRC16_K128_LO);
            a5 = crc16_fold_neon(a5, crc16_load_bswap_neon(buf +  80), CRC16_K128_HI, CRC16_K128_LO);
            a6 = crc16_fold_neon(a6, crc16_load_bswap_neon(buf +  96), CRC16_K128_HI, CRC16_K128_LO);
            a7 = crc16_fold_neon(a7, crc16_load_bswap_neon(buf + 112), CRC16_K128_HI, CRC16_K128_LO);
            buf += 128;
            remaining -= 128;
        }

        /* Fold 8 accumulators down to 1 */
        uint64x2_t res = a7;
        res = crc16_fold_neon(a0, res, CRC16_FOLD112_HI, CRC16_FOLD112_LO);
        res = crc16_fold_neon(a1, res, CRC16_FOLD96_HI,  CRC16_FOLD96_LO);
        res = crc16_fold_neon(a2, res, CRC16_FOLD80_HI,  CRC16_FOLD80_LO);
        res = crc16_fold_neon(a3, res, CRC16_FOLD64_HI,  CRC16_FOLD64_LO);
        res = crc16_fold_neon(a4, res, CRC16_FOLD48_HI,  CRC16_FOLD48_LO);
        res = crc16_fold_neon(a5, res, CRC16_FOLD32_HI,  CRC16_FOLD32_LO);
        res = crc16_fold_neon(a6, res, CRC16_K_HI,       CRC16_K_LO);
        acc = res;
    } else {
        /* Single-accumulator fold for 64-255 bytes */
        acc = crc16_load_bswap_neon(buf);
        acc = veorq_u64(acc, vsetq_lane_u64((uint64_t)crc << 48, vdupq_n_u64(0), 1));
        buf += 16;
        remaining = len - 16;
    }

    /* Continue with single-accumulator fold for remaining 16-byte blocks */
    while (remaining >= 16) {
        acc = crc16_fold_neon(acc, crc16_load_bswap_neon(buf), CRC16_K_HI, CRC16_K_LO);
        buf += 16;
        remaining -= 16;
    }

    /* Store folded result (byte-swap back to original order) */
    uint8x16_t acc8 = crc16_bswap_128(vreinterpretq_u8_u64(acc));
    uint8_t final_buf[32];
    vst1q_u8(final_buf, acc8);

    if (remaining > 0) {
        memcpy(final_buf + 16, buf, remaining);
        return crc16speed(0, final_buf, 16 + remaining);
    } else {
        return crc16speed(0, final_buf, 16);
    }
}

/* ============================================================================
 * CRC16 with integrated 24-bit -> 32-bit unpacking (ARM64 NEON/PMULL)
 *
 * Same approach as the x86 version (see detailed comments there), adapted
 * for NEON intrinsics:
 *
 * - vqtbl1q_u8 replaces _mm_shuffle_epi8: performs byte table lookup from a
 *   16-byte table. Indices 0-15 select a source byte; indices >= 16 produce 0
 *   (on x86, bit 7 set in the mask zeroes the byte — same effect, different
 *   encoding).
 *
 * - Combined unpack+bswap lookup table (identical logical mapping as x86):
 *     {0xFF, 11, 10, 9, 0xFF, 8, 7, 6, 0xFF, 5, 4, 3, 0xFF, 2, 1, 0}
 *   0xFF (>= 16) zeroes the byte, producing the zero-extended upper byte.
 *
 * - Each iteration loads 12 packed bytes (via 16-byte vld1q_u8, upper 4
 *   bytes ignored by the lookup) and produces 16 unpacked+byte-swapped bytes,
 *   fed directly into the PMULL folding pipeline.
 * ============================================================================
 */

/* Scalar fallback for small buffers on ARM64 */
static uint16_t crc16_packed24_scalar(uint16_t crc, const uint8_t *buf,
                                      uint64_t packed_len) {
    extern uint16_t crc16speed(uint16_t crc, const void *s, const uint64_t l);
    uint8_t batch[16];
    uint64_t words = packed_len / 3;
    uint64_t i = 0;
    for (; i + 4 <= words; i += 4) {
        for (int j = 0; j < 4; j++) {
            batch[j * 4 + 0] = buf[(i + j) * 3 + 0];
            batch[j * 4 + 1] = buf[(i + j) * 3 + 1];
            batch[j * 4 + 2] = buf[(i + j) * 3 + 2];
            batch[j * 4 + 3] = 0;
        }
        crc = crc16speed(crc, batch, 16);
    }
    uint64_t rem = words - i;
    for (uint64_t j = 0; j < rem; j++) {
        batch[j * 4 + 0] = buf[(i + j) * 3 + 0];
        batch[j * 4 + 1] = buf[(i + j) * 3 + 1];
        batch[j * 4 + 2] = buf[(i + j) * 3 + 2];
        batch[j * 4 + 3] = 0;
    }
    if (rem > 0) {
        crc = crc16speed(crc, batch, rem * 4);
    }
    return crc;
}

uint16_t crc16_simd_packed24(uint16_t crc, const void *packed_data,
                             uint64_t packed_len) {
    extern uint16_t crc16speed(uint16_t crc, const void *s, const uint64_t l);
    const uint8_t *buf = (const uint8_t *)packed_data;
    const uint8_t *end = buf + packed_len;

    /* Scalar fallback for small buffers or no SIMD */
    if (!crc_simd_available() || packed_len < 48) {
        return crc16_packed24_scalar(crc, buf, packed_len);
    }

    /* Combined unpack (24->32 bit zero-extend) + byte-swap lookup table */
    static const uint8_t unpack_bswap_tbl[16] = {
        0xFF, 11, 10, 9,  0xFF,  8,  7, 6,
        0xFF,  5,  4, 3,  0xFF,  2,  1, 0
    };
    uint8x16_t unpack_bswap = vld1q_u8(unpack_bswap_tbl);

    #define UNPACK_LOAD_NEON(ptr) \
        vreinterpretq_u64_u8(vqtbl1q_u8(vld1q_u8(ptr), unpack_bswap))

    uint64x2_t acc;

    /* 8-way parallel folding for packed24 (>= 208 packed bytes) */
    if (packed_len >= 208) {
        uint64x2_t a0 = UNPACK_LOAD_NEON(buf +  0);
        uint64x2_t a1 = UNPACK_LOAD_NEON(buf + 12);
        uint64x2_t a2 = UNPACK_LOAD_NEON(buf + 24);
        uint64x2_t a3 = UNPACK_LOAD_NEON(buf + 36);
        uint64x2_t a4 = UNPACK_LOAD_NEON(buf + 48);
        uint64x2_t a5 = UNPACK_LOAD_NEON(buf + 60);
        uint64x2_t a6 = UNPACK_LOAD_NEON(buf + 72);
        uint64x2_t a7 = UNPACK_LOAD_NEON(buf + 84);

        a0 = veorq_u64(a0, vsetq_lane_u64((uint64_t)crc << 48, vdupq_n_u64(0), 1));
        buf += 96;

        /* 8-way folding loop: 96 packed bytes per iteration */
        while (buf + 100 <= end) {
            a0 = crc16_fold_neon(a0, UNPACK_LOAD_NEON(buf +  0), CRC16_K128_HI, CRC16_K128_LO);
            a1 = crc16_fold_neon(a1, UNPACK_LOAD_NEON(buf + 12), CRC16_K128_HI, CRC16_K128_LO);
            a2 = crc16_fold_neon(a2, UNPACK_LOAD_NEON(buf + 24), CRC16_K128_HI, CRC16_K128_LO);
            a3 = crc16_fold_neon(a3, UNPACK_LOAD_NEON(buf + 36), CRC16_K128_HI, CRC16_K128_LO);
            a4 = crc16_fold_neon(a4, UNPACK_LOAD_NEON(buf + 48), CRC16_K128_HI, CRC16_K128_LO);
            a5 = crc16_fold_neon(a5, UNPACK_LOAD_NEON(buf + 60), CRC16_K128_HI, CRC16_K128_LO);
            a6 = crc16_fold_neon(a6, UNPACK_LOAD_NEON(buf + 72), CRC16_K128_HI, CRC16_K128_LO);
            a7 = crc16_fold_neon(a7, UNPACK_LOAD_NEON(buf + 84), CRC16_K128_HI, CRC16_K128_LO);
            buf += 96;
        }

        /* Fold 8 accumulators down to 1 */
        uint64x2_t res = a7;
        res = crc16_fold_neon(a0, res, CRC16_FOLD112_HI, CRC16_FOLD112_LO);
        res = crc16_fold_neon(a1, res, CRC16_FOLD96_HI,  CRC16_FOLD96_LO);
        res = crc16_fold_neon(a2, res, CRC16_FOLD80_HI,  CRC16_FOLD80_LO);
        res = crc16_fold_neon(a3, res, CRC16_FOLD64_HI,  CRC16_FOLD64_LO);
        res = crc16_fold_neon(a4, res, CRC16_FOLD48_HI,  CRC16_FOLD48_LO);
        res = crc16_fold_neon(a5, res, CRC16_FOLD32_HI,  CRC16_FOLD32_LO);
        res = crc16_fold_neon(a6, res, CRC16_K_HI,       CRC16_K_LO);
        acc = res;
    } else {
        /* Single-accumulator fold for smaller inputs */
        acc = UNPACK_LOAD_NEON(buf);
        acc = veorq_u64(acc, vsetq_lane_u64((uint64_t)crc << 48, vdupq_n_u64(0), 1));
        buf += 12;
    }

    /* Continue with single-accumulator fold */
    while (buf + 16 <= end) {
        acc = crc16_fold_neon(acc, UNPACK_LOAD_NEON(buf), CRC16_K_HI, CRC16_K_LO);
        buf += 12;
    }

    #undef UNPACK_LOAD_NEON

    /* Final reduction via byte-swap + table lookup */
    uint8x16_t acc8 = crc16_bswap_128(vreinterpretq_u8_u64(acc));
    uint8_t final_buf[48];
    vst1q_u8(final_buf, acc8);

    uint64_t remaining_packed = end - buf;
    uint64_t remaining_words = remaining_packed / 3;
    for (uint64_t i = 0; i < remaining_words; i++) {
        final_buf[16 + i * 4 + 0] = buf[i * 3 + 0];
        final_buf[16 + i * 4 + 1] = buf[i * 3 + 1];
        final_buf[16 + i * 4 + 2] = buf[i * 3 + 2];
        final_buf[16 + i * 4 + 3] = 0;
    }

    return crc16speed(0, final_buf, 16 + remaining_words * 4);
}

void crc16_simd_init(void) {
    crc_simd_available();
}

#else /* No SIMD support */

bool crc_simd_available(void);

uint16_t crc16_simd(uint16_t crc, const void *data, uint64_t len) {
    extern uint16_t crc16speed(uint16_t crc, const void *s, const uint64_t l);
    return crc16speed(crc, data, len);
}

uint16_t crc16_simd_packed24(uint16_t crc, const void *packed_data,
                             uint64_t packed_len) {
    extern uint16_t crc16speed(uint16_t crc, const void *s, const uint64_t l);
    const uint8_t *buf = (const uint8_t *)packed_data;
    uint8_t batch[16];
    uint64_t words = packed_len / 3;
    uint64_t i = 0;
    for (; i + 4 <= words; i += 4) {
        for (int j = 0; j < 4; j++) {
            batch[j * 4 + 0] = buf[(i + j) * 3 + 0];
            batch[j * 4 + 1] = buf[(i + j) * 3 + 1];
            batch[j * 4 + 2] = buf[(i + j) * 3 + 2];
            batch[j * 4 + 3] = 0;
        }
        crc = crc16speed(crc, batch, 16);
    }
    uint64_t rem = words - i;
    for (uint64_t j = 0; j < rem; j++) {
        batch[j * 4 + 0] = buf[(i + j) * 3 + 0];
        batch[j * 4 + 1] = buf[(i + j) * 3 + 1];
        batch[j * 4 + 2] = buf[(i + j) * 3 + 2];
        batch[j * 4 + 3] = 0;
    }
    if (rem > 0) {
        crc = crc16speed(crc, batch, rem * 4);
    }
    return crc;
}

void crc16_simd_init(void) {
}

#endif /* Architecture selection */
