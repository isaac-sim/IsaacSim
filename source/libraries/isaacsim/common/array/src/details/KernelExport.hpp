// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

/**
 * @brief Marks a kernel library entry point as resolvable by name at runtime.
 * @details
 * CudaKernel loads this library with `LoadLibrary`/`dlopen` and resolves its entry points with
 * `GetProcAddress`/`dlsym`, so they must appear in the binary's export table. Windows exports
 * nothing from a DLL without `__declspec(dllexport)`; the ELF attribute keeps the symbols visible
 * if the target ever adopts hidden visibility.
 */
#if defined(_WIN32)
#    define ISAACSIM_COMMON_ARRAY_KERNEL_EXPORT extern "C" __declspec(dllexport)
#else
#    define ISAACSIM_COMMON_ARRAY_KERNEL_EXPORT extern "C" __attribute__((visibility("default")))
#endif
