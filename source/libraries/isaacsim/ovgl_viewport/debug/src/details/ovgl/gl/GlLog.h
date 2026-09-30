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

/* Tiny leveled stderr sink for the OpenGL layer.
 *
 * gl historically wrote unconditional `fprintf(stderr, ...)` lines
 * ("egl_headless: ...", "gpu_gles: ..."). Callers may need sub-error chatter
 * suppressible while low-level OVGL tests retain the historical stderr bytes.
 * Therefore:
 *
 *   - the process-global threshold DEFAULTS TO GL_LOG_VERBOSE (everything
 *     prints, byte-identical to the historical fprintf output);
 *   - a consumer that owns a log-level contract lowers/raises it via
 *     gl_log_set_threshold();
 *   - an optional hook receives EVERY line regardless of the stderr
 *     threshold, so a log-callback owner can route sub-error traffic to its
 *     callback while stderr stays quiet (carb-like routing).
 *
 * Levels use plain int in the ABI so cross-TU C prototypes stay exact.
 */
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

    enum
    {
        GL_LOG_VERBOSE = 0, /* debug echo (NU_OPENGL_DEBUG_MODE, makeCurrent dump) */
        GL_LOG_INFO = 1, /* device discovery, GL_VERSION, uploads, HDR loading */
        GL_LOG_WARN = 2, /* recoverable (light truncation, RGBA8 fallback)     */
        GL_LOG_ERROR = 3, /* precedes a failure return                          */
        GL_LOG_FATAL = 4
    };

    /* Process-global stderr threshold: a line prints iff level >= threshold.
     * Default GL_LOG_VERBOSE (historical behavior). */
    void gl_log_set_threshold(int threshold);
    int gl_log_get_threshold(void);

    /* Optional process-global hook. Receives every line (regardless of the
     * stderr threshold) as (user, level, channel, full_line) where full_line is
     * "channel: message" without a trailing newline, valid only for the duration
     * of the call. Pass fn = NULL to remove. */
    typedef void (*gl_log_hook_fn)(void* user, int level, const char* channel, const char* line);
    void gl_log_set_hook(gl_log_hook_fn fn, void* user);

    /* Emit one line. stderr output is "channel: message\n" — byte-identical to
     * the historical fprintf(stderr, "channel: " fmt "\n", ...) lines. */
    void gl_log(int level, const char* channel, const char* fmt, ...)
#if defined(__GNUC__) || defined(__clang__)
        __attribute__((format(printf, 3, 4)))
#endif
        ;

#ifdef __cplusplus
}
#endif
