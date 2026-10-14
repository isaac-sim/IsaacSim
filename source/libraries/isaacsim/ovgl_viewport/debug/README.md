# OVGL viewport

`isaacsim.ovgl_viewport.debug` provides a lightweight OVGL viewport for tests and debugging. It displays one
RenderProduct from an existing OVStage in an SDL OpenGL window and provides first-person camera controls. Linux uses
OpenGL ES; Windows uses desktop OpenGL. Hidden rendering uses each platform's offscreen context.

Your application remains responsible for creating the scene and running the simulation loop. The Python
`author_viewport()` helper adds a standard camera, RenderVar, and RenderProduct to a Foundation OpenUSD stage.
`Viewport` keeps the resulting OVStage, publishes camera movement, and releases native resources when its context
closes:

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

On the application's main thread, call `poll_events()`, update the application, and then call `render()`. A visible
viewport presents directly from the GPU, so its returned `Frame` has dimensions and version numbers but no `rgba`
data. Set `visible=False` to capture RGBA8 pixels. Linux uses EGL and does not require X11 or Wayland. Windows uses a
hidden SDL OpenGL window and requires a desktop graphics session with OpenGL 4.1. The next `render()` call reuses the
returned frame data, so copy `frame.rgba` before rendering again if you need to compare captures.

The native C++ viewport does not depend on Foundation. It accepts an OVStage pointer and a function that writes camera
poses. The higher-level Python API connects it to Foundation and accepts a custom `camera_pose_writer` when you need to
replace the default behavior.

The application remains responsible for populating the stage and composing USD. The implementation privately uses the
OmniClient runtime supplied by its pinned OVStage dependency to obtain local cached copies of material images and
dome-light HDR files; the viewport wheel does not carry another copy. Asset loading and storage are not part of the
public viewport API.

In a visible viewport, press `H` to toggle the window-side debug HUD. Its initial metric is viewport frames per second.
The HUD is composed into the presentation framebuffer after the scene render.

This debug module does not define a renderer registry, backend-neutral renderer, rendering manager, generic output
model, or asynchronous API. Add those abstractions only when production requirements are clear.

## Source layout

- `include/isaacsim/ovgl_viewport/debug` contains the supported C++ API.
- `src` contains SDL presentation and viewport orchestration; `src/details` contains all private OVStage and OVGL
  implementation code.
- `src/details/ovgl/gl` contains the internal OpenGL rasterizer. The root Pixi manifest and lock supply the pinned
  `stb_image` headers and SDL SDK; SDL is linked privately as a shared implementation detail. The private Packman
  manifest supplies the pinned OmniClient headers and link libraries for native builds, while Python installations use
  the runtime from the pinned `ovstage` package. Their licenses are installed with the runtime package.
- `bindings/python` exposes the native API, while `python` provides the Foundation-integrated Python interface.
- `tests/cpp` and `tests/python` contain language-specific tests.
