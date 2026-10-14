# Articulation State Reads benchmark

The SimpleArticulation asset runs its revolute and prismatic velocity drives at 0.5 rad/s and 0.5 m/s. Each frame completes physics and reads joint positions, velocities, and projected joint forces on the host. The slider eventually reaches its limit.

From this directory, with the dependencies in `example.toml` installed:

```bash
python main.py --physics-device cpu --output /tmp/joint-cpu.json
```

Defaults: CPU physics, 0.005 s timestep, 100 excluded warmup frames, and 1,000 measured frames. Use `--physics-device gpu` for GPU dynamics.
