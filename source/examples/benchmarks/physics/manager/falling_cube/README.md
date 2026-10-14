# Foundation Falling Cube benchmark

One cube falls onto a ground plane. Each frame completes physics and reads its position on the host. `--render` adds transform publication and a completed 1280 × 720 offscreen render inside the measured frame. Headless mode does not publish transforms.

From this directory, with the dependencies in `example.toml` installed:

```bash
python main.py --physics-device cpu --output /tmp/cube-cpu.json
```

Defaults: CPU physics, 0.005 s timestep, 100 excluded warmup frames, and 1,000 measured frames. Use `--physics-device gpu` for GPU dynamics.
