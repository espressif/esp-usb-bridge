/*
 * SPDX-FileCopyrightText: 2020-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#if defined ( __GNUC__ )
#include "xtensa_gcc.h"
#else
#error Unknown compiler.
#endif
