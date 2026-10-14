# Rigid Body State Reads benchmark

A spinning cube falls onto a ground plane. Each frame completes physics and reads position, orientation, linear velocity, and angular velocity on the host. This measures the public rigid-body path used by an IMU; it does not include tutorial-local filtering or sensor-frame algorithms.

From this directory, with the dependencies in `example.toml` installed:

```bash
python main.py --physics-device cpu --output /tmp/imu-cpu.json
```

Defaults: CPU physics, 0.005 s timestep, 100 excluded warmup frames, and 1,000 measured frames. Use `--physics-device gpu` for GPU dynamics.
