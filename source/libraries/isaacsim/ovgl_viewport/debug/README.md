# OVGL Viewport

`isaacsim.ovgl_viewport.debug` is a concrete OVGL viewport for tests and debugging. It borrows a populated OVStage,
renders one authored RenderProduct, presents its `LdrColor` output through an SDL OpenGL window, and provides
first-person debug camera navigation. Linux uses OpenGL ES, and Windows uses desktop OpenGL. Invisible rendering uses
the existing offscreen context on each platform.

The application owns scene authoring and its simulation loop. The Python `author_viewport()` helper adds the standard
camera, RenderVar, and RenderProduct to an OpenUSD Foundation stage. `Viewport` then retains the populated Foundation
OVStage, publishes interactive camera poses, and releases its native resources when its context closes:

```python
author_viewport(openusd_stage)

with Viewport(
    ovstage_stage,
    title="Simulation",
    visible=not headless,
    camera=Camera(target=(0.0, 0.0, 0.75), distance=7.0),
) as viewport:
    while viewport.poll_events():
        update_application()
        frame = viewport.render()
```

Call `poll_events()`, update the application, then call `render()` on the application main thread. A visible viewport
renders and presents directly on the GPU; its returned `Frame` contains dimensions and ordinals but an empty `rgba`
payload. Set `visible=False` when the application needs captured RGBA8 pixels. On Linux, invisible rendering uses EGL
and does not require X11 or Wayland. On Windows, it uses a hidden SDL desktop OpenGL window and requires a
desktop-capable graphics session with OpenGL 4.1 support. The returned `Frame` is borrowed and reused by the next
`render()` call; copy `frame.rgba` before rendering another invisible frame when comparing captures.

The native C++ viewport remains independent of Foundation and accepts an OVStage pointer plus a camera-pose writer.
The high-level Python interface integrates those native pieces with Foundation and accepts a custom
`camera_pose_writer` when an application needs to replace the default camera publication behavior.

Stage population and USD composition remain the application's responsibility. For resolved material images and
dome-light HDRs, the implementation privately uses its pinned OmniClient dependency to obtain cache-local files. This
does not add asset loading or storage concepts to the viewport's public API.

In a visible viewport, press `H` to toggle the window-side debug HUD. Its initial metric is viewport frames per second.
The HUD is composed into the presentation framebuffer after the scene render.

This module intentionally does not define a rendering-provider registry, backend-neutral renderer, rendering manager,
generic output model, or asynchronous render API. Those abstractions are deferred until production viewport and
rendering requirements are known.

## Source layout

- `include/isaacsim/ovgl_viewport/debug` contains the supported C++ API.
- `src` contains SDL presentation and viewport orchestration; `src/details` contains all private OVStage and OVGL
  implementation code.
- `src/details/ovgl/gl` contains the internal OpenGL rasterizer. The root Pixi manifest and lock supply the pinned
  `stb_image` headers and SDL SDK; SDL is linked privately as a shared implementation detail. The private Packman
  manifest supplies the pinned OmniClient SDK. Their licenses are installed with the runtime package.
- `bindings/python` exposes the native API, while `python` provides the Foundation-integrated Python interface.
- `tests/cpp` and `tests/python` contain language-specific tests.
