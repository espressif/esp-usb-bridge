/*
 * SPDX-FileCopyrightText: 2020-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#ifndef __STATIC_FORCEINLINE
#define __STATIC_FORCEINLINE    static inline __attribute__((always_inline))
#endif

#ifndef __STATIC_INLINE
#define __STATIC_INLINE         static inline __attribute__((always_inline))
#endif

#ifndef __WEAK
#define __WEAK                  __attribute__((weak))
#endif

#define __NOP()                 __asm volatile ("nop")
