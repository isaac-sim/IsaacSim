# Follow Target benchmark

One frame completes a controller update, a physics step, and transform publication.
`--render` also completes a 1280 × 720 offscreen render and pixel readback.

From this directory, with the dependencies in `example.toml` installed:

```bash
python main.py --output /tmp/follow_target.json --physics-device cpu
```

Defaults: CPU physics, rendering off, 0.005 s timestep, 100 excluded warmup frames, and 1,000 measured frames.
