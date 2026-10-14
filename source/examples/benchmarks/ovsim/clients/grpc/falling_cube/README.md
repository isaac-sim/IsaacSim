# OV Sim falling cube over gRPC

A frame completes one physics step and reads the cube position back to the caller.
Rendering is disabled; CPU physics is the default. Defaults: 0.005 s physics timestep (200 Hz simulated time),
100 excluded warmup frames and 1,000 measured frames. Wall-clock FPS is unpaced.

From this directory, with the dependencies in `example.toml` installed:

```bash
python main.py --physics-device cpu --output /tmp/cube-grpc-cpu.json
```

Use `--physics-device gpu` for GPU dynamics. `--warmup-frames`, `--measured-frames`,
and `--time-step` select the measurement size and simulation timestep.

A temporary loopback server starts before measurement and stops after it. Use
`--endpoint 127.0.0.1:50051` to connect to an existing server, started with the
`isaacsim-ovsim-grpc-server --listen-address 127.0.0.1:50051` command. Timings include both gRPC round trips; process resource
metrics describe the client, not the server.
