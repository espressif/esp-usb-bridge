// Copyright 2020-2021 Espressif Systems (Shanghai) CO LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

# pragma once

#include <sys/param.h>

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*(x)))
#endif

#define GET_BYTE(n, b)          (((n) >> ((b) * 8)) & 0xFF)

#define EUB_ASSERT(condition)            do {                           \
                                                if (!(condition)) {     \
                                                    eub_abort();        \
                                                }                       \
                                            } while(0)

void __attribute__((noreturn)) eub_abort(void);
