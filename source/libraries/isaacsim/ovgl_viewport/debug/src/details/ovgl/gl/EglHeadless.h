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

#include <stdint.h>

/*
 * egl_headless.h — Private offscreen OpenGL context abstraction.
 *
 * Linux uses EGL_EXT_device_enumeration +
 * EGL_EXT_platform_device to create an
 * OpenGL ES context without X11. Windows retains the legacy function names
 *
 * while implementing them with a hidden SDL desktop OpenGL context.
 */

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct EglHeadless EglHeadless;

    /* Create a headless EGL context with the given pbuffer dimensions.
     * Returns NULL on failure. */
    EglHeadless* egl_headless_create(int width, int height);

    /* Destroy the headless EGL context. */
    void egl_headless_destroy(EglHeadless* ctx);

    /* Resize the pbuffer. Destroys the old surface and allocates a new one
     * with the requested dimensions, then re-makes the context current.
     * Returns 1 on success, 0 on failure (context retains old surface). */
    int egl_headless_resize(EglHeadless* ctx, int width, int height);

    /* Returns 1 when this context is running on a SOFTWARE rasteriser -- i.e. the
     * host offered no usable GPU/Vulkan path and the tiered backend selection fell
     * through to D3D11 WARP or SwiftShader. The viewport still works; it is slower,
     * and callers that advertise a feature tier should say so rather than let a
     * user read a 3 FPS viewport as a bug. Always 0 off Windows/ANGLE. */
    int egl_headless_is_software(const EglHeadless* ctx);

    /* Make this context current on the calling thread. Useful for embedders
     * that share the process with another GL context (e.g. omni.ui running
     * its own GLFW window). Returns 1 on success. */
    int egl_headless_make_current(EglHeadless* ctx);

    /* Release this context if it is current on the calling thread, leaving
     * EGL_NO_CONTEXT current. Lets a sibling library (e.g. omni.ui) re-make
     * its own context current. */
    void egl_headless_release_current(EglHeadless* ctx);

    /* Returns 1 if some GL context (ours, the host's, anyone's) is already
     * current on the calling thread. Used by viewer_create to decide
     * whether to spin up a fresh headless context or piggy-back on one
     * the worker thread already pre-warmed. Portable wrapper around
     * eglGetCurrentContext / CGLGetCurrentContext. */
    int egl_headless_has_current(void);

    /* Opaque identity of the calling thread's current GL context, or 0 when
     * none is current. The value is only for equality/lifetime checks; callers
     * must not dereference it. Portable EGLContext/CGLContextObj wrapper. */
    uintptr_t egl_headless_current_context_id(void);

    /* ---- foreign (GLX) context suspend/restore -------------------------------
     *
     * On Linux, NVIDIA's libglvnd refuses to make an EGL context current while a
     * GLX context is current ANYWHERE IN THE PROCESS. Measured 2026-07-26 against
     * omni.ui (robot-ovui's viewer shell), which draws through GLFW on a GLX
     * context: eglMakeCurrent returns EGL_FALSE with EGL_BAD_ACCESS, so
     * egl_headless_create fails and the whole viewport renders nothing.
     *
     * Three measurements pin the mechanism, and rule out the two theories that
     * were in the code before:
     *   - after omni.ui.init(): eglGetCurrentContext() == NULL but
     *     glXGetCurrentContext() != NULL. The prior mitigation here was
     *     eglReleaseThread(), which only touches EGL state and therefore could
     *     never help.
     *   - creating the context on a FRESH thread, with neither EGL nor GLX current
     *     on it, still fails -- so this is NOT thread-local contention, and moving
     *     the render to a worker thread does not fix it.
     *   - glXMakeCurrent(dpy, None, NULL) on the thread holding GLX, then
     *     egl_headless_create -> succeeds.
     * Vulkan is NOT the trigger: vkCreateInstance + vkEnumeratePhysicalDevices
     * followed by egl_headless_create succeeds.
     *
     * suspend: if a GLX context is current on the calling thread, save it in
     * thread-local state, unbind it, and return 1. Returns 0 -- and touches
     * nothing at all -- when no GLX context is current, which is every headless
     * and usdview-adopted lane, so those stay bit-identical.
     * restore: undo a suspend that returned 1 (release our EGL context first, then
     * re-bind the saved GLX one). A no-op when suspend did not fire. */
    int egl_headless_suspend_foreign(void);
    void egl_headless_restore_foreign(void);

    /* Human-readable reason a suspend did or did not fire. Only meaningful right
     * after an EGL make-current failure; never allocates. */
    const char* egl_headless_foreign_state(void);

#ifdef __cplusplus
}
#endif
