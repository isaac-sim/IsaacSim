# Holonomic Drive benchmark

The controller sets constant holonomic wheel targets during setup. One frame
completes a physics step and transform publication. Controller setup is excluded.
`--render` also completes a 1280 × 720 offscreen render and pixel readback.

From this directory, with the dependencies in `example.toml` installed:

```bash
python main.py --output /tmp/holonomic_drive.json --physics-device cpu
```

Defaults: CPU physics, rendering off, 0.005 s timestep, 100 excluded warmup frames, and 1,000 measured frames.
