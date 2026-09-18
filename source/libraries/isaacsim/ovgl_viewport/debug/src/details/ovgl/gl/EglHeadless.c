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

/*
 * EglHeadless.c — Headless EGL context via NVIDIA GPU device.
 *
 * Uses EGL_EXT_device_enumeration + EGL_EXT_platform_device to create
 * an OpenGL ES 3.1 context backed by an NVIDIA GPU, without X11.
 * The pbuffer surface acts as the default framebuffer for glReadPixels.
 */

#include "EglHeadless.h"

#include "GlLog.h"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#ifdef _WIN32
#    include <EGL/eglext_angle.h> /* EGL_PLATFORM_ANGLE_* tokens (ANGLE provides EGL on Windows) */
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef EGL_EXT_platform_device
#    define EGL_PLATFORM_DEVICE_EXT 0x313F
#endif

/* ---- foreign (GLX) context suspend/restore; see egl_headless.h ------------
 * GLX is reached through dlopen/dlsym, never linked: this file must keep
 * building and running on hosts with no GLX at all (pure-EGL containers,
 * Windows/ANGLE). Everything below compiles to a
 * pair of no-ops off Linux. */
/* Which API actually backs this context. EGL is the default and the only one
 * off Linux; GLX is the fallback for a process that already uses GLX, where EGL
 * make-current is refused outright (see egl_headless.h). Everything below
 * dispatches on it, so ovgl itself never has to know which one it got. */
enum
{
    OVGL_CTX_EGL = 0,
    OVGL_CTX_GLX = 1
};

struct EglHeadless
{
    int backend;
    EGLDisplay display;
    EGLContext context;
    EGLSurface surface;
    EGLConfig config;
    int width;
    int height;
    /* 1 when the context came up on a SOFTWARE rasteriser (D3D11 WARP or
     * SwiftShader) because no GPU/Vulkan path was available. Reported, not
     * acted on, here -- see egl_headless_is_software. Stays 0 on every backend
     * that is not the Windows/ANGLE one. */
    int software_rendering;
    /* GLES version actually granted (see the ladder in egl_headless_create).
     * minor < 1 means no compute/tessellation/geometry. 0 until a context is
     * created, and left 0 on the non-EGL backends. */
    int gles_major;
    int gles_minor;
    /* GLX backend only (opaque: Display*, GLXContext, GLXFBConfig are pointers
     * and GLXPbuffer is an XID, so no X11/GLX headers are needed here -- this
     * file must still compile on hosts with no X11 development packages). */
    void* glx_dpy;
    void* glx_ctx;
    void* glx_cfg;
    unsigned long glx_pbuf;
};

#if defined(__linux__)
#    include <dlfcn.h>
#    include <pthread.h>

typedef void* (*glx_get_ctx_fn)(void);
typedef void* (*glx_get_dpy_fn)(void);
typedef unsigned long (*glx_get_draw_fn)(void);
typedef int (*glx_make_current_fn)(void*, unsigned long, void*);

static int g_glx_probed = 0;
static glx_get_ctx_fn g_glXGetCurrentContext = NULL;
static glx_get_dpy_fn g_glXGetCurrentDisplay = NULL;
static glx_get_draw_fn g_glXGetCurrentDrawable = NULL;
static glx_make_current_fn g_glXMakeCurrent = NULL;

/* Thread-local: a suspend on one thread must not be restored by another. */
static __thread void* t_saved_glx_ctx = NULL;
static __thread void* t_saved_glx_dpy = NULL;
static __thread unsigned long t_saved_glx_draw = 0;

/* Guards the one-time probe below. This is NOT covered by the "OVGL serializes
 * owned-context create/destroy" contract further down: glx_probe runs from
 * egl_headless_suspend_foreign, which ovgl_render_frame calls on the RENDER
 * thread every frame, while an embedder can reach the same code from its UI
 * thread. */
static pthread_mutex_t g_glx_probe_mu = PTHREAD_MUTEX_INITIALIZER;

/* POSIX specifies that dlsym can return function symbols, while ISO C does not
 * define a direct conversion from void* to a function pointer. Copying the
 * representation avoids a non-portable cast and keeps strict builds clean. */
#    define OVGL_LOAD_FUNCTION(destination, handle, symbol_name)                                                       \
        do                                                                                                             \
        {                                                                                                              \
            void* ovgl_symbol = dlsym((handle), (symbol_name));                                                        \
            _Static_assert(                                                                                            \
                sizeof(destination) == sizeof(ovgl_symbol), "Function and data pointers must have equal size");        \
            memcpy(&(destination), &ovgl_symbol, sizeof(destination));                                                 \
        } while (0)

static void glx_probe(void)
{
    /* The published flag must be set only AFTER the pointers are stored. The
     * previous spelling set it first, so a second thread arriving mid-probe took
     * the early return, read NULL pointers, and concluded "GLX not loaded --
     * nothing to suspend". That silently skips the foreign-context suspend this
     * whole file exists to perform, and the symptom is a viewport that fails
     * make-current rather than an obvious crash. */
    pthread_mutex_lock(&g_glx_probe_mu);
    if (g_glx_probed)
    {
        pthread_mutex_unlock(&g_glx_probe_mu);
        return;
    }
    /* RTLD_NOLOAD: only bind if the host process ALREADY uses GLX. A process
     * that does not must not have libGLX pulled in as a side effect of ovgl
     * rendering -- that would hand a headless box a dependency it never had. */
    void* h = dlopen("libGLX.so.0", RTLD_LAZY | RTLD_NOLOAD);
    if (!h)
        h = dlopen("libGL.so.1", RTLD_LAZY | RTLD_NOLOAD);
    if (h)
    {
        OVGL_LOAD_FUNCTION(g_glXGetCurrentContext, h, "glXGetCurrentContext");
        OVGL_LOAD_FUNCTION(g_glXGetCurrentDisplay, h, "glXGetCurrentDisplay");
        OVGL_LOAD_FUNCTION(g_glXGetCurrentDrawable, h, "glXGetCurrentDrawable");
        OVGL_LOAD_FUNCTION(g_glXMakeCurrent, h, "glXMakeCurrent");
    }
    /* Published last: everything a reader needs is stored by now. A process with
     * no GLX still marks itself probed so the dlopen is attempted only once. */
    g_glx_probed = 1;
    pthread_mutex_unlock(&g_glx_probe_mu);
}

/* Why a suspend did not fire, for the make-current failure message. Callers
 * only ever print this when EGL has already failed, so it costs nothing on the
 * happy path and turns an opaque EGL_BAD_ACCESS into a first-line diagnosis. */
const char* egl_headless_foreign_state(void)
{
    glx_probe();
    if (!g_glXGetCurrentContext || !g_glXMakeCurrent)
        return "GLX not loaded in this process (nothing to suspend)";
    if (t_saved_glx_ctx)
        return "a foreign GLX context IS suspended by us";
    if (!g_glXGetCurrentContext())
        return "GLX is loaded but no GLX context is current ON THIS THREAD "
               "(it may be current on another thread, which blocks EGL "
               "process-wide and cannot be released from here)";
    return "a GLX context is current on this thread and was NOT suspended";
}

int egl_headless_suspend_foreign(void)
{
    glx_probe();
    if (!g_glXGetCurrentContext || !g_glXMakeCurrent)
        return 0;
    void* ctx = g_glXGetCurrentContext();
    if (!ctx)
        return 0; /* nothing foreign current: no-op */
    void* dpy = g_glXGetCurrentDisplay ? g_glXGetCurrentDisplay() : NULL;
    unsigned long draw = g_glXGetCurrentDrawable ? g_glXGetCurrentDrawable() : 0;
    if (!dpy)
        return 0; /* cannot restore it -> do not take it */
    if (!g_glXMakeCurrent(dpy, 0, NULL))
        return 0;
    t_saved_glx_ctx = ctx;
    t_saved_glx_dpy = dpy;
    t_saved_glx_draw = draw;
    gl_log(GL_LOG_VERBOSE, "egl_headless", "suspended foreign GLX context %p (drawable %lu)", ctx, draw);
    return 1;
}

/* ---- GLX-backed context (the fallback for a GLX-using process) -------------
 * A GLES 3.2 context created through GLX_ARB_create_context +
 * GLX_EXT_create_context_es2_profile is the SAME renderer ovgl gets from EGL
 * (measured: "OpenGL ES 3.2 NVIDIA 595.71.05" from both), so the GLES shaders
 * compile unchanged -- this is a context-creation swap, not a GL-flavour
 * change. Unlike EGL it coexists with a host GLX context on another thread,
 * which is exactly the robot-ovui case: omni.ui's GLFW window holds GLX on the
 * UI thread while ovgl renders on a backend thread. Verified on a worker thread
 * with omni.ui initialized on the main thread.
 *
 * ovgl renders into its own FBOs and reads back from resolve_fbo, never from
 * the default framebuffer, so the pbuffer here exists only to satisfy
 * glXMakeContextCurrent -- its size is irrelevant to output. */
#    define OVGL_GLX_RENDER_TYPE 0x8011
#    define OVGL_GLX_RGBA_BIT 0x0001
#    define OVGL_GLX_DRAWABLE_TYPE 0x8010
#    define OVGL_GLX_PBUFFER_BIT 0x0004
#    define OVGL_GLX_DOUBLEBUFFER 5
#    define OVGL_GLX_RED_SIZE 8
#    define OVGL_GLX_GREEN_SIZE 9
#    define OVGL_GLX_BLUE_SIZE 10
#    define OVGL_GLX_ALPHA_SIZE 11
#    define OVGL_GLX_DEPTH_SIZE 12
#    define OVGL_GLX_PBUFFER_WIDTH 0x8041
#    define OVGL_GLX_PBUFFER_HEIGHT 0x8042
#    define OVGL_GLX_CTX_MAJOR_ARB 0x2091
#    define OVGL_GLX_CTX_MINOR_ARB 0x2092
#    define OVGL_GLX_CTX_PROFILE_ARB 0x9126
#    define OVGL_GLX_CTX_ES_PROFILE 0x0004

typedef void* (*x_open_display_fn)(const char*);
typedef int (*x_default_screen_fn)(void*);
typedef int (*x_free_fn)(void*);
typedef const char* (*glx_query_ext_fn)(void*, int);
typedef void* (*glx_get_proc_fn)(const char*);
typedef void** (*glx_choose_fbconfig_fn)(void*, int, const int*, int*);
typedef unsigned long (*glx_create_pbuffer_fn)(void*, void*, const int*);
typedef void (*glx_destroy_pbuffer_fn)(void*, unsigned long);
typedef int (*glx_make_context_current_fn)(void*, unsigned long, unsigned long, void*);
typedef void (*glx_destroy_context_fn)(void*, void*);
typedef void* (*glx_create_ctx_attribs_fn)(void*, void*, void*, int, const int*);

static int g_glx_be_probed = 0;
static int g_glx_be_ok = 0;
static x_open_display_fn g_XOpenDisplay = NULL;
static x_default_screen_fn g_XDefaultScreen = NULL;
static x_free_fn g_XFree = NULL;
static glx_query_ext_fn g_glXQueryExtensionsString = NULL;
static glx_get_proc_fn g_glXGetProcAddress = NULL;
static glx_choose_fbconfig_fn g_glXChooseFBConfig = NULL;
static glx_create_pbuffer_fn g_glXCreatePbuffer = NULL;
static glx_destroy_pbuffer_fn g_glXDestroyPbuffer = NULL;
static glx_make_context_current_fn g_glXMakeContextCurrent = NULL;
static glx_destroy_context_fn g_glXDestroyContext = NULL;
static glx_create_ctx_attribs_fn g_glXCreateContextAttribsARB = NULL;

static void glx_backend_probe(void)
{
    /* Same publish-last discipline as glx_probe(). Callers may reach
     * egl_headless_create without the renderer's owned-context lock, so this
     * initializer provides its own serialization. */
    pthread_mutex_lock(&g_glx_probe_mu);
    if (g_glx_be_probed)
    {
        pthread_mutex_unlock(&g_glx_probe_mu);
        return;
    }
    /* RTLD_NOLOAD like glx_probe(): this fallback only exists for processes that
     * ALREADY use GLX. A pure-EGL host must not gain an X11/GLX dependency. */
    void* gh = dlopen("libGLX.so.0", RTLD_LAZY | RTLD_NOLOAD);
    if (!gh)
        gh = dlopen("libGL.so.1", RTLD_LAZY | RTLD_NOLOAD);
    void* xh = dlopen("libX11.so.6", RTLD_LAZY | RTLD_NOLOAD);
    if (!gh || !xh)
    {
        g_glx_be_probed = 1;
        pthread_mutex_unlock(&g_glx_probe_mu);
        return;
    }
    OVGL_LOAD_FUNCTION(g_XOpenDisplay, xh, "XOpenDisplay");
    OVGL_LOAD_FUNCTION(g_XDefaultScreen, xh, "XDefaultScreen");
    OVGL_LOAD_FUNCTION(g_XFree, xh, "XFree");
    OVGL_LOAD_FUNCTION(g_glXQueryExtensionsString, gh, "glXQueryExtensionsString");
    OVGL_LOAD_FUNCTION(g_glXGetProcAddress, gh, "glXGetProcAddress");
    OVGL_LOAD_FUNCTION(g_glXChooseFBConfig, gh, "glXChooseFBConfig");
    OVGL_LOAD_FUNCTION(g_glXCreatePbuffer, gh, "glXCreatePbuffer");
    OVGL_LOAD_FUNCTION(g_glXDestroyPbuffer, gh, "glXDestroyPbuffer");
    OVGL_LOAD_FUNCTION(g_glXMakeContextCurrent, gh, "glXMakeContextCurrent");
    OVGL_LOAD_FUNCTION(g_glXDestroyContext, gh, "glXDestroyContext");
    g_glx_be_ok = g_XOpenDisplay && g_XDefaultScreen && g_glXQueryExtensionsString && g_glXGetProcAddress &&
                  g_glXChooseFBConfig && g_glXCreatePbuffer && g_glXMakeContextCurrent;
    g_glx_be_probed = 1; /* published last, after g_glx_be_ok is final */
    pthread_mutex_unlock(&g_glx_probe_mu);
}

static int glx_backend_make_current(EglHeadless* ctx)
{
    if (!g_glXMakeContextCurrent)
        return 0;
    return g_glXMakeContextCurrent(ctx->glx_dpy, ctx->glx_pbuf, ctx->glx_pbuf, ctx->glx_ctx) ? 1 : 0;
}

/* Returns a filled-in GLX-backed context, or NULL (leaving `ctx` untouched). */
static int glx_backend_init(EglHeadless* ctx, int width, int height)
{
    glx_backend_probe();
    if (!g_glx_be_ok)
    {
        gl_log(GL_LOG_VERBOSE, "glx_headless", "GLX fallback unavailable (libGLX/libX11 not already loaded)");
        return 0;
    }
    void* dpy = g_XOpenDisplay(NULL);
    if (!dpy)
    {
        gl_log(GL_LOG_ERROR, "glx_headless", "XOpenDisplay failed (no DISPLAY?)");
        return 0;
    }
    const int screen = g_XDefaultScreen(dpy);
    const char* exts = g_glXQueryExtensionsString(dpy, screen);
    if (!exts || !strstr(exts, "GLX_ARB_create_context") || !strstr(exts, "GLX_EXT_create_context_es2_profile"))
    {
        gl_log(GL_LOG_ERROR, "glx_headless",
               "driver lacks GLX_ARB_create_context + "
               "GLX_EXT_create_context_es2_profile; cannot make a GLES context");
        return 0;
    }
    if (!g_glXCreateContextAttribsARB)
    {
        void* create_context_attribs = g_glXGetProcAddress("glXCreateContextAttribsARB");
        _Static_assert(sizeof(g_glXCreateContextAttribsARB) == sizeof(create_context_attribs),
                       "Function and data pointers must have equal size");
        memcpy(&g_glXCreateContextAttribsARB, &create_context_attribs, sizeof(g_glXCreateContextAttribsARB));
    }
    if (!g_glXCreateContextAttribsARB)
    {
        gl_log(GL_LOG_ERROR, "glx_headless", "glXCreateContextAttribsARB missing");
        return 0;
    }
    const int cfg_attribs[] = { OVGL_GLX_RENDER_TYPE,
                                OVGL_GLX_RGBA_BIT,
                                OVGL_GLX_DRAWABLE_TYPE,
                                OVGL_GLX_PBUFFER_BIT,
                                OVGL_GLX_RED_SIZE,
                                8,
                                OVGL_GLX_GREEN_SIZE,
                                8,
                                OVGL_GLX_BLUE_SIZE,
                                8,
                                OVGL_GLX_ALPHA_SIZE,
                                8,
                                OVGL_GLX_DEPTH_SIZE,
                                24,
                                OVGL_GLX_DOUBLEBUFFER,
                                0,
                                0 };
    int nconfigs = 0;
    void** cfgs = g_glXChooseFBConfig(dpy, screen, cfg_attribs, &nconfigs);
    if (!cfgs || nconfigs <= 0)
    {
        gl_log(GL_LOG_ERROR, "glx_headless", "glXChooseFBConfig found no pbuffer config");
        return 0;
    }
    void* cfg = cfgs[0];
    if (g_XFree)
        g_XFree(cfgs);

    const int w = width > 0 ? width : 256, h = height > 0 ? height : 256;
    const int pb_attribs[] = { OVGL_GLX_PBUFFER_WIDTH, w, OVGL_GLX_PBUFFER_HEIGHT, h, 0 };
    unsigned long pbuf = g_glXCreatePbuffer(dpy, cfg, pb_attribs);
    if (!pbuf)
    {
        gl_log(GL_LOG_ERROR, "glx_headless", "glXCreatePbuffer failed");
        return 0;
    }
    /* 3.2 then 3.1, mirroring the EGL path's fallback. */
    void* gctx = NULL;
    for (int minor = 2; minor >= 1 && !gctx; --minor)
    {
        const int ctx_attribs[] = { OVGL_GLX_CTX_MAJOR_ARB,  3, OVGL_GLX_CTX_MINOR_ARB, minor, OVGL_GLX_CTX_PROFILE_ARB,
                                    OVGL_GLX_CTX_ES_PROFILE, 0 };
        gctx = g_glXCreateContextAttribsARB(dpy, cfg, NULL, 1, ctx_attribs);
    }
    if (!gctx)
    {
        gl_log(GL_LOG_ERROR, "glx_headless", "glXCreateContextAttribsARB(GLES 3.2/3.1) failed");
        if (g_glXDestroyPbuffer)
            g_glXDestroyPbuffer(dpy, pbuf);
        return 0;
    }
    ctx->backend = OVGL_CTX_GLX;
    ctx->glx_dpy = dpy;
    ctx->glx_cfg = cfg;
    ctx->glx_pbuf = pbuf;
    ctx->glx_ctx = gctx;
    ctx->width = w;
    ctx->height = h;
    if (!glx_backend_make_current(ctx))
    {
        gl_log(GL_LOG_ERROR, "glx_headless", "glXMakeContextCurrent failed");
        g_glXDestroyContext(dpy, gctx);
        if (g_glXDestroyPbuffer)
            g_glXDestroyPbuffer(dpy, pbuf);
        ctx->backend = OVGL_CTX_EGL;
        return 0;
    }
    gl_log(GL_LOG_INFO, "glx_headless",
           "GLES context via GLX created (%dx%d) -- EGL is unusable in this "
           "process because it already holds a GLX context",
           w, h);
    return 1;
}

void egl_headless_restore_foreign(void)
{
    if (!t_saved_glx_ctx)
        return;
    /* Drop our EGL context first: leaving both bound lets glvnd dispatch a
     * later GL call to whichever it thinks is current, which is exactly the
     * ambiguity this pairing exists to remove. */
    EGLDisplay cur = eglGetCurrentDisplay();
    if (cur != EGL_NO_DISPLAY)
        eglMakeCurrent(cur, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    if (!g_glXMakeCurrent(t_saved_glx_dpy, t_saved_glx_draw, t_saved_glx_ctx))
    {
        gl_log(GL_LOG_ERROR, "egl_headless", "failed to restore foreign GLX context %p", t_saved_glx_ctx);
    }
    t_saved_glx_ctx = NULL;
    t_saved_glx_dpy = NULL;
    t_saved_glx_draw = 0;
}
#else
int egl_headless_suspend_foreign(void)
{
    return 0;
}
void egl_headless_restore_foreign(void)
{
}
const char* egl_headless_foreign_state(void)
{
    return "GLX suspend is Linux-only";
}
/* No GLX off Linux: the EGL/ANGLE (Windows) and CGL (macOS) paths have no such
 * coexistence problem, so the fallback is compiled out entirely. */
static int glx_backend_init(EglHeadless* c, int w, int h)
{
    (void)c;
    (void)w;
    (void)h;
    return 0;
}
static int glx_backend_make_current(EglHeadless* c)
{
    (void)c;
    return 0;
}
#endif


/* EGL returns the same per-device EGLDisplay to every renderer. NVIDIA's EGL
 * invalidates every context on that display when any owner calls eglTerminate,
 * so initialize it once and terminate it only after the last OVGL context is
 * gone. OVGL serializes complete owned-context create/destroy operations before
 * entering this file, so the shared state below cannot race. */
static EGLDisplay g_shared_display = EGL_NO_DISPLAY;
static unsigned int g_shared_display_refs = 0;
static EGLint g_shared_display_major = 0;
static EGLint g_shared_display_minor = 0;

static void retain_display(EGLDisplay display, EGLint major, EGLint minor)
{
    if (g_shared_display == EGL_NO_DISPLAY)
    {
        g_shared_display = display;
        g_shared_display_major = major;
        g_shared_display_minor = minor;
    }
    ++g_shared_display_refs;
}

static void release_display(EGLDisplay display)
{
    if (display == g_shared_display && g_shared_display_refs > 0)
    {
        --g_shared_display_refs;
        if (g_shared_display_refs == 0)
        {
            eglTerminate(display);
            g_shared_display = EGL_NO_DISPLAY;
            g_shared_display_major = 0;
            g_shared_display_minor = 0;
        }
        return;
    }
    eglTerminate(display);
}

EglHeadless* egl_headless_create(int width, int height)
{
    EglHeadless* ctx = (EglHeadless*)calloc(1, sizeof(EglHeadless));
    if (!ctx)
        return NULL;
    ctx->width = width;
    ctx->height = height;
    int display_initialized = 0;
    int display_retained = 0;
    EGLint initialized_major = 0;
    EGLint initialized_minor = 0;

    if (g_shared_display != EGL_NO_DISPLAY)
    {
        ctx->display = g_shared_display;
        display_initialized = 1;
        initialized_major = g_shared_display_major;
        initialized_minor = g_shared_display_minor;
        retain_display(ctx->display, initialized_major, initialized_minor);
        display_retained = 1;
    }

    if (!display_initialized)
    {
#ifdef _WIN32
        /* Windows has no native EGL/GLES; ANGLE (libEGL.dll/libGLESv2.dll) provides it.
         * ANGLE builds ship different backends -- vcpkg's is D3D, Chrome/Edge's may be
         * Vulkan -- and requesting one the build lacks fails getPlatformDisplay with
         * EGL_BAD_ATTRIBUTE (0x3004). ANGLE has no device-enumeration / EGL_PLATFORM_
         * DEVICE_EXT path, so select the display via platform attributes, and try
         * candidates in preference order, keeping the first that both yields a display
         * and eglInitializes.
         *
         * The order is HARDWARE-FIRST, THEN SOFTWARE, and both halves are load-bearing:
         * the editors ship to arbitrary machines, so a host with an NVIDIA GPU and a
         * Vulkan ICD must get the fast path (GLES 3.1 + EXT_tessellation_shader +
         * EXT_geometry_shader), and a host with no discrete GPU, no Vulkan runtime, or
         * no GPU at all -- a VM, an RDP session, a stock laptop with only a basic
         * display adapter -- must still open the viewport instead of failing to create
         * a context. Selection is entirely at runtime; there is deliberately no env var
         * for it, because an end user cannot be expected to know one.
         *
         * The software tiers, in order -- and the order is by FEATURE LEVEL, not by
         * speed, because a faster backend that cannot compile the shaders renders
         * nothing at all:
         *   Vulkan SwiftShader   SwiftShader is a complete software Vulkan ICD, so
         *                        ANGLE gives it the same GLES 3.1 + EXT_tessellation
         *                        _shader/EXT_geometry_shader it gives a real Vulkan
         *                        driver. The shaders are built at "#version 310 es"
         *                        with those extensions REQUIRED (shaders_gles.h,
         *                        OVGL_GLES31_EXT), so this tier runs the full
         *                        pipeline unmodified -- just slowly. It ships with
         *                        the ANGLE distribution as vk_swiftshader.dll +
         *                        vk_swiftshader_icd.json, both staged beside ovgl.
         *   D3D11 WARP           Microsoft's software D3D11 rasteriser: in every
         *                        Windows 10+ install, no driver and no GPU needed,
         *                        and faster than SwiftShader. But ANGLE's D3D11
         *                        backend tops out at a GLES 3.0 context on it
         *                        (measured on this box: "Microsoft Basic Render
         *                        Driver ... Direct3D11 vs_5_0 ps_5_0" -> GL_VERSION
         *                        "OpenGL ES 3.0"), where "#version 310 es" is
         *                        rejected and gpu_create_material_pipeline fails.
         *                        Kept as the last resort so a host that somehow has
         *                        no working Vulkan loader at all still gets a
         *                        context, but it is BELOW SwiftShader deliberately.
         * A DEVICE_TYPE the ANGLE build does not implement simply fails
         * getPlatformDisplay with EGL_BAD_ATTRIBUTE and costs one loop iteration, so
         * listing a tier that is absent is harmless.
         *
         * ctx->software_rendering records which half won, so callers can report the
         * tier and scale their own expectations (see egl_headless_is_software). */
        {
            PFNEGLGETPLATFORMDISPLAYEXTPROC getPlatformDisplay =
                (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
            if (!getPlatformDisplay)
            {
                gl_log(GL_LOG_ERROR, "egl_headless", "ANGLE eglGetPlatformDisplayEXT unavailable");
                free(ctx);
                return NULL;
            }
            static const struct
            {
                EGLint type;
                EGLint device_type;
                const char* name;
                int software;
            } angle_backends[] = {
                { EGL_PLATFORM_ANGLE_TYPE_VULKAN_ANGLE, EGL_PLATFORM_ANGLE_DEVICE_TYPE_HARDWARE_ANGLE, "Vulkan", 0 },
                { EGL_PLATFORM_ANGLE_TYPE_D3D11_ANGLE, EGL_PLATFORM_ANGLE_DEVICE_TYPE_HARDWARE_ANGLE, "D3D11", 0 },
                { EGL_PLATFORM_ANGLE_TYPE_D3D9_ANGLE, EGL_PLATFORM_ANGLE_DEVICE_TYPE_HARDWARE_ANGLE, "D3D9", 0 },
                { EGL_PLATFORM_ANGLE_TYPE_VULKAN_ANGLE, EGL_PLATFORM_ANGLE_DEVICE_TYPE_SWIFTSHADER_ANGLE,
                  "Vulkan-SwiftShader", 1 },
                { EGL_PLATFORM_ANGLE_TYPE_D3D11_ANGLE, EGL_PLATFORM_ANGLE_DEVICE_TYPE_D3D_WARP_ANGLE, "D3D11-WARP", 1 },
            };
            const int n_backends = (int)(sizeof(angle_backends) / sizeof(angle_backends[0]));
            EGLint last_err = EGL_SUCCESS;
            /* QA-ONLY: skip the hardware tiers so the software fallback can be
             * exercised on a machine that HAS a GPU. This is not a user-facing
             * setting and is not documented to end users -- degradation on a real
             * GPU-less host is automatic and needs no env var. It exists because a
             * fallback nobody can run is a fallback that silently rots: this box is
             * the only QA machine and it has an RTX, so without a way to force the
             * lower tier, "runs everywhere" would never actually be tested. */
            const char* force_sw = getenv("OVGL_FORCE_SOFTWARE");
            const int want_sw_only = (force_sw && *force_sw && *force_sw != '0');
            if (want_sw_only)
                gl_log(GL_LOG_INFO, "egl_headless", "OVGL_FORCE_SOFTWARE set -- skipping hardware backends (QA)");
            for (int bi = 0; bi < n_backends; ++bi)
            {
                if (want_sw_only && !angle_backends[bi].software)
                    continue;
                EGLint dpy_attribs[] = { EGL_PLATFORM_ANGLE_TYPE_ANGLE, angle_backends[bi].type,
                                         EGL_PLATFORM_ANGLE_DEVICE_TYPE_ANGLE, angle_backends[bi].device_type, EGL_NONE };
                EGLDisplay d = getPlatformDisplay(EGL_PLATFORM_ANGLE_ANGLE, EGL_DEFAULT_DISPLAY, dpy_attribs);
                if (d == EGL_NO_DISPLAY)
                {
                    last_err = eglGetError();
                    gl_log(GL_LOG_INFO, "egl_headless", "ANGLE %s getPlatformDisplay unavailable (0x%x)",
                           angle_backends[bi].name, last_err);
                    continue;
                }
                EGLint maj = 0, min = 0;
                if (eglInitialize(d, &maj, &min))
                {
                    ctx->display = d;
                    ctx->software_rendering = angle_backends[bi].software;
                    display_initialized = 1;
                    initialized_major = maj;
                    initialized_minor = min;
                    gl_log(GL_LOG_INFO, "egl_headless", "ANGLE %s backend selected (EGL %d.%d)%s",
                           angle_backends[bi].name, maj, min,
                           angle_backends[bi].software ? " -- SOFTWARE rendering: no GPU/Vulkan found, expect"
                                                         " reduced performance" :
                                                         "");
                    break;
                }
                last_err = eglGetError();
                gl_log(GL_LOG_INFO, "egl_headless", "ANGLE %s eglInitialize failed (0x%x)", angle_backends[bi].name,
                       last_err);
                eglTerminate(d);
            }
            if (!display_initialized)
            {
                gl_log(GL_LOG_ERROR, "egl_headless",
                       "no usable ANGLE backend (tried hardware Vulkan/D3D11/D3D9 then "
                       "software D3D11-WARP/SwiftShader); last EGL error 0x%x",
                       last_err);
                free(ctx);
                return NULL;
            }
        }
#else
        /* Check client extensions for device enumeration */
        const char* client_exts = eglQueryString(EGL_NO_DISPLAY, EGL_EXTENSIONS);
        if (!client_exts || !strstr(client_exts, "EGL_EXT_device_enumeration") ||
            !strstr(client_exts, "EGL_EXT_platform_device"))
        {
            gl_log(GL_LOG_ERROR, "egl_headless", "required EGL extensions not available");
            free(ctx);
            return NULL;
        }

        /* Load extension functions */
        PFNEGLQUERYDEVICESEXTPROC eglQueryDevicesEXT = (PFNEGLQUERYDEVICESEXTPROC)eglGetProcAddress("eglQueryDevicesEXT");
        PFNEGLGETPLATFORMDISPLAYEXTPROC eglGetPlatformDisplayEXT =
            (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
        PFNEGLQUERYDEVICESTRINGEXTPROC eglQueryDeviceStringEXT =
            (PFNEGLQUERYDEVICESTRINGEXTPROC)eglGetProcAddress("eglQueryDeviceStringEXT");

        if (!eglQueryDevicesEXT || !eglGetPlatformDisplayEXT)
        {
            gl_log(GL_LOG_ERROR, "egl_headless", "failed to load EGL extension functions");
            free(ctx);
            return NULL;
        }

        /* Enumerate EGL devices */
        EGLDeviceEXT devices[16];
        EGLint num_devices = 0;
        if (!eglQueryDevicesEXT(16, devices, &num_devices) || num_devices == 0)
        {
            gl_log(GL_LOG_ERROR, "egl_headless", "no EGL devices found");
            free(ctx);
            return NULL;
        }

        gl_log(GL_LOG_INFO, "egl_headless", "found %d EGL device(s)", num_devices);

        /* Find an NVIDIA device (prefer it over Mesa/llvmpipe) */
        EGLDeviceEXT nvidia_device = EGL_NO_DEVICE_EXT;
        EGLDeviceEXT fallback_device = EGL_NO_DEVICE_EXT;

        for (EGLint i = 0; i < num_devices; i++)
        {
            const char* exts = eglQueryDeviceStringEXT ? eglQueryDeviceStringEXT(devices[i], EGL_EXTENSIONS) : NULL;
            gl_log(GL_LOG_INFO, "egl_headless", "device %d extensions: %s", i, exts ? exts : "(none)");

            /* Try to get a display from this device to check the renderer */
            EGLDisplay test_dpy = eglGetPlatformDisplayEXT(EGL_PLATFORM_DEVICE_EXT, devices[i], NULL);
            if (test_dpy != EGL_NO_DISPLAY)
            {
                EGLint major, minor;
                if (eglInitialize(test_dpy, &major, &minor))
                {
                    /* Bind OpenGL ES API before querying configs */
                    eglBindAPI(EGL_OPENGL_ES_API);

                    /* Check if this device supports GLES rendering */
                    EGLint config_attribs[] = { EGL_SURFACE_TYPE,
                                                EGL_PBUFFER_BIT,
                                                EGL_RENDERABLE_TYPE,
                                                EGL_OPENGL_ES3_BIT,
                                                EGL_RED_SIZE,
                                                8,
                                                EGL_GREEN_SIZE,
                                                8,
                                                EGL_BLUE_SIZE,
                                                8,
                                                EGL_ALPHA_SIZE,
                                                8,
                                                EGL_DEPTH_SIZE,
                                                24,
                                                EGL_NONE };

                    EGLConfig config;
                    EGLint num_configs;
                    if (eglChooseConfig(test_dpy, config_attribs, &config, 1, &num_configs) && num_configs > 0)
                    {
                        const char* vendor = eglQueryString(test_dpy, EGL_VENDOR);
                        gl_log(GL_LOG_INFO, "egl_headless", "device %d vendor: %s", i, vendor ? vendor : "(unknown)");

                        if (vendor && strstr(vendor, "NVIDIA"))
                        {
                            nvidia_device = devices[i];
                            /* Keep the selected display initialized. Calling
                             * eglTerminate here invalidates contexts owned by an
                             * existing EglHeadless using the same per-device
                             * EGLDisplay on NVIDIA's implementation. */
                            ctx->display = test_dpy;
                            display_initialized = 1;
                            initialized_major = major;
                            initialized_minor = minor;
                            break;
                        }
                        if (fallback_device == EGL_NO_DEVICE_EXT)
                            fallback_device = devices[i];
                    }
                    eglTerminate(test_dpy);
                }
            }
        }

        EGLDeviceEXT chosen = (nvidia_device != EGL_NO_DEVICE_EXT) ? nvidia_device : fallback_device;
        if (chosen == EGL_NO_DEVICE_EXT)
        {
            gl_log(GL_LOG_ERROR, "egl_headless", "no suitable EGL device found");
            free(ctx);
            return NULL;
        }

        /* Create display from chosen device */
        if (!display_initialized)
        {
            ctx->display = eglGetPlatformDisplayEXT(EGL_PLATFORM_DEVICE_EXT, chosen, NULL);
        }
        if (ctx->display == EGL_NO_DISPLAY)
        {
            gl_log(GL_LOG_ERROR, "egl_headless", "eglGetPlatformDisplay failed");
            free(ctx);
            return NULL;
        }
#endif /* !_WIN32 */
    }

    EGLint major = initialized_major;
    EGLint minor = initialized_minor;
    if (!display_initialized)
    {
        if (!eglInitialize(ctx->display, &major, &minor))
        {
            gl_log(GL_LOG_ERROR, "egl_headless", "eglInitialize failed (0x%x)", eglGetError());
            free(ctx);
            return NULL;
        }
    }
    gl_log(GL_LOG_INFO, "egl_headless", "EGL %d.%d initialized", major, minor);

    eglBindAPI(EGL_OPENGL_ES_API);

    /* Choose config — request 4x MSAA for smoother curve silhouettes.
     * If the driver can't satisfy the multisample request we fall back
     * to a 1-sample config below. */
    EGLint msaa_attribs[] = { EGL_SURFACE_TYPE,
                              EGL_PBUFFER_BIT,
                              EGL_RENDERABLE_TYPE,
                              EGL_OPENGL_ES3_BIT,
                              EGL_RED_SIZE,
                              8,
                              EGL_GREEN_SIZE,
                              8,
                              EGL_BLUE_SIZE,
                              8,
                              EGL_ALPHA_SIZE,
                              8,
                              EGL_DEPTH_SIZE,
                              24,
                              EGL_SAMPLE_BUFFERS,
                              1,
                              EGL_SAMPLES,
                              4,
                              EGL_NONE };
    EGLint plain_attribs[] = { EGL_SURFACE_TYPE,
                               EGL_PBUFFER_BIT,
                               EGL_RENDERABLE_TYPE,
                               EGL_OPENGL_ES3_BIT,
                               EGL_RED_SIZE,
                               8,
                               EGL_GREEN_SIZE,
                               8,
                               EGL_BLUE_SIZE,
                               8,
                               EGL_ALPHA_SIZE,
                               8,
                               EGL_DEPTH_SIZE,
                               24,
                               EGL_NONE };

    EGLConfig config;
    EGLint num_configs = 0;
    /* Don't ask a software rasteriser for 4x MSAA. It will happily provide it,
     * and then shade every pixel four times on the CPU -- measured on this box
     * with SwiftShader, that is most of a ~10 s frame. Smooth silhouettes are
     * exactly the sort of quality the reduced tier is supposed to trade away to
     * stay usable, so the software path takes the 1-sample config directly
     * rather than falling back to it only when MSAA is unavailable. */
    if (!ctx->software_rendering && eglChooseConfig(ctx->display, msaa_attribs, &config, 1, &num_configs) &&
        num_configs > 0)
    {
        gl_log(GL_LOG_INFO, "egl_headless", "4x MSAA enabled");
    }
    else if (!eglChooseConfig(ctx->display, plain_attribs, &config, 1, &num_configs) || num_configs == 0)
    {
        gl_log(GL_LOG_ERROR, "egl_headless", "eglChooseConfig failed");
        release_display(ctx->display);
        free(ctx);
        return NULL;
    }

    ctx->config = config;

    /* Create pbuffer surface */
    EGLint pbuf_attribs[] = { EGL_WIDTH, width, EGL_HEIGHT, height, EGL_NONE };
    ctx->surface = eglCreatePbufferSurface(ctx->display, config, pbuf_attribs);
    if (ctx->surface == EGL_NO_SURFACE)
    {
        gl_log(GL_LOG_ERROR, "egl_headless", "eglCreatePbufferSurface failed (0x%x)", eglGetError());
        release_display(ctx->display);
        free(ctx);
        return NULL;
    }

    /* Create an OpenGL ES 3.2 context (tessellation + geometry shaders are core in
     * 3.2, required by the curve tube renderer). Some backends -- notably ANGLE's
     * Vulkan backend on Windows -- cap at GLES 3.1 but expose those capabilities as
     * the EXT_tessellation_shader / EXT_geometry_shader extensions, so fall back to
     * a 3.1 context there (the shaders are compiled to match under OVGL_GLES31_EXT).
     *
     * 3.0 is the last rung, and it is what makes the software tier reachable:
     * ANGLE's D3D11 backend on WARP advertises no ES 3.1, so a 3.2/3.1-only
     * ladder ends at eglCreateContext -> EGL_BAD_MATCH (0x3009) and the editor
     * cannot open at all on a GPU-less host. A 3.0 context loses compute
     * shaders and the tessellation/geometry path the curve tube renderer uses;
     * that is the "reduced feature set" the tier is for, and it beats no
     * viewport. The achieved version is logged and kept on the context so
     * callers can branch on it rather than rediscover it from GL_VERSION. */
    static const struct
    {
        EGLint major, minor;
    } gles_ladder[] = {
        { 3, 2 },
        { 3, 1 },
        { 3, 0 },
    };
    for (size_t li = 0; li < sizeof(gles_ladder) / sizeof(gles_ladder[0]); ++li)
    {
        EGLint ctx_attribs[] = { EGL_CONTEXT_MAJOR_VERSION, gles_ladder[li].major, EGL_CONTEXT_MINOR_VERSION,
                                 gles_ladder[li].minor, EGL_NONE };
        ctx->context = eglCreateContext(ctx->display, config, EGL_NO_CONTEXT, ctx_attribs);
        if (ctx->context != EGL_NO_CONTEXT)
        {
            ctx->gles_major = gles_ladder[li].major;
            ctx->gles_minor = gles_ladder[li].minor;
            gl_log(GL_LOG_INFO, "egl_headless", "GLES %d.%d context created%s", gles_ladder[li].major,
                   gles_ladder[li].minor,
                   gles_ladder[li].minor < 1 ? " -- no compute/tessellation/geometry (reduced feature set)" : "");
            break;
        }
    }
    if (ctx->context == EGL_NO_CONTEXT)
    {
        gl_log(GL_LOG_ERROR, "egl_headless", "eglCreateContext failed for GLES 3.2/3.1/3.0 (0x%x)", eglGetError());
        eglDestroySurface(ctx->display, ctx->surface);
        release_display(ctx->display);
        free(ctx);
        return NULL;
    }

    /* If some other library (e.g. omni.ui's GLFW) already has a GL context
     * current, eglMakeCurrent below fails with EGL_BAD_ACCESS. eglReleaseThread
     * covers the case where that context is an EGL one; a GLX one needs
     * egl_headless_suspend_foreign (see egl_headless.h -- omni.ui's is GLX, and
     * eglReleaseThread alone provably could not help it). The caller pairs the
     * suspend with egl_headless_restore_foreign once it is done rendering. */
    eglReleaseThread();
    (void)egl_headless_suspend_foreign();
    eglBindAPI(EGL_OPENGL_ES_API);

    gl_log(GL_LOG_VERBOSE, "egl_headless", "pre-makeCurrent: display=%p surface=%p context=%p api=0x%x",
           (void*)ctx->display, (void*)ctx->surface, (void*)ctx->context, (unsigned)eglQueryAPI());

    /* Make current */
    EGLBoolean mc_ok = eglMakeCurrent(ctx->display, ctx->surface, ctx->surface, ctx->context);
    EGLint mc_err = eglGetError();
    if (!mc_ok)
    {
        gl_log(GL_LOG_ERROR, "egl_headless", "eglMakeCurrent failed (rv=%d, err=0x%x); foreign-GL state: %s",
               (int)mc_ok, (unsigned)mc_err, egl_headless_foreign_state());
        eglDestroyContext(ctx->display, ctx->context);
        eglDestroySurface(ctx->display, ctx->surface);
        release_display(ctx->display);
        /* EGL is unusable in this process. That is not fatal: a GLES context
         * obtained through GLX is the same renderer and DOES coexist with the
         * host's GLX context (see glx_backend_init). Reuse the same allocation
         * so the caller still gets one opaque handle. */
        memset(ctx, 0, sizeof(*ctx));
        ctx->width = width;
        ctx->height = height;
        if (glx_backend_init(ctx, width, height))
            return ctx;
        free(ctx);
        return NULL;
    }

    if (!display_retained)
    {
        retain_display(ctx->display, major, minor);
    }
    gl_log(GL_LOG_INFO, "egl_headless", "context created (%dx%d)", width, height);
    return ctx;
}

void egl_headless_destroy(EglHeadless* ctx)
{
    if (!ctx)
        return;
    if (ctx->backend == OVGL_CTX_GLX)
    {
#if defined(__linux__)
        if (g_glXMakeContextCurrent)
            g_glXMakeContextCurrent(ctx->glx_dpy, 0, 0, NULL);
        if (g_glXDestroyContext && ctx->glx_ctx)
            g_glXDestroyContext(ctx->glx_dpy, ctx->glx_ctx);
        if (g_glXDestroyPbuffer && ctx->glx_pbuf)
            g_glXDestroyPbuffer(ctx->glx_dpy, ctx->glx_pbuf);
            /* The Display* is NOT closed: the host (omni.ui/GLFW) owns X connections
             * in this process, and XCloseDisplay on a connection we opened while
             * theirs is live has been a reliable way to take Xlib down with us. One
             * leaked connection per renderer teardown is the cheaper trade. */
#endif
        free(ctx);
        return;
    }
    eglMakeCurrent(ctx->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(ctx->display, ctx->context);
    eglDestroySurface(ctx->display, ctx->surface);
    release_display(ctx->display);
    free(ctx);
}

int egl_headless_is_software(const EglHeadless* ctx)
{
    return ctx ? ctx->software_rendering : 0;
}

int egl_headless_make_current(EglHeadless* ctx)
{
    if (!ctx)
        return 0;
    /* A GLX-backed context must NOT suspend GLX -- GLX is what it runs on. */
    if (ctx->backend == OVGL_CTX_GLX)
        return glx_backend_make_current(ctx);
    /* Same reason as in create: a foreign GLX context current anywhere in the
     * process makes this call fail with EGL_BAD_ACCESS. No-op unless one is. */
    (void)egl_headless_suspend_foreign();
    if (!eglMakeCurrent(ctx->display, ctx->surface, ctx->surface, ctx->context))
    {
        EGLint err = eglGetError();
        /* EGL_NOT_INITIALIZED (0x3001) means the shared, process-global display was
         * already terminated by another context's teardown -- benign at shutdown, where
         * ovgl_destroy_renderer skips GL cleanup on this false return, so it is reported
         * silently. Only a genuine make-current failure while rendering is worth flagging. */
        if (err != EGL_NOT_INITIALIZED)
            gl_log(GL_LOG_ERROR, "egl_headless", "make_current failed (0x%x); foreign-GL state: %s", err,
                   egl_headless_foreign_state());
        return 0;
    }
    return 1;
}

void egl_headless_release_current(EglHeadless* ctx)
{
    if (!ctx)
        return;
    if (ctx->backend == OVGL_CTX_GLX)
    {
#if defined(__linux__)
        if (g_glXMakeContextCurrent)
            g_glXMakeContextCurrent(ctx->glx_dpy, 0, 0, NULL);
#endif
        return;
    }
    eglMakeCurrent(ctx->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
}

int egl_headless_has_current(void)
{
    return egl_headless_current_context_id() != 0;
}

uintptr_t egl_headless_current_context_id(void)
{
    /* ovgl uses this for has_current() and for pinning an adopted context, so it
     * must see a GLX-backed context too -- eglGetCurrentContext() is NULL then. */
    uintptr_t id = (uintptr_t)eglGetCurrentContext();
#if defined(__linux__)
    if (!id)
    {
        glx_probe();
        if (g_glXGetCurrentContext)
            id = (uintptr_t)g_glXGetCurrentContext();
    }
#endif
    return id;
}

int egl_headless_resize(EglHeadless* ctx, int width, int height)
{
    if (!ctx || width <= 0 || height <= 0)
        return 0;
    if (width == ctx->width && height == ctx->height)
        return 1;
    if (ctx->backend == OVGL_CTX_GLX)
    {
#if defined(__linux__)
        /* ovgl draws into its own FBOs and reads back from resolve_fbo, so the
         * pbuffer is never a render target -- resizing it would change nothing
         * observable. Record the new size and keep the context current. */
        ctx->width = width;
        ctx->height = height;
        return glx_backend_make_current(ctx);
#else
        return 0;
#endif
    }

    EGLint pbuf_attribs[] = { EGL_WIDTH, width, EGL_HEIGHT, height, EGL_NONE };
    EGLSurface new_surface = eglCreatePbufferSurface(ctx->display, ctx->config, pbuf_attribs);
    if (new_surface == EGL_NO_SURFACE)
    {
        gl_log(GL_LOG_ERROR, "egl_headless", "resize eglCreatePbufferSurface failed (0x%x)", eglGetError());
        return 0;
    }
    if (!eglMakeCurrent(ctx->display, new_surface, new_surface, ctx->context))
    {
        gl_log(GL_LOG_ERROR, "egl_headless", "resize eglMakeCurrent failed (0x%x)", eglGetError());
        eglDestroySurface(ctx->display, new_surface);
        return 0;
    }
    eglDestroySurface(ctx->display, ctx->surface);
    ctx->surface = new_surface;
    ctx->width = width;
    ctx->height = height;
    return 1;
}
