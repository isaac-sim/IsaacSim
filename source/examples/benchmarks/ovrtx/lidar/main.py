# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Profile ovrtx running a ring of RTX lidars.

Places ``--num-sensors`` ``OmniLidar`` prims around the authored environment, steps their ``PointCloud`` render
products for ``--num-frames`` frames, and reads the valid returns back each frame.

A lidar sweeps over time, so by default it only hands out a point cloud once a scan completes, and each cloud covers
whichever ticks landed in that scan. Both defaults here override that so a step is a fixed unit of work: instant lidar
makes every emission a full scan, and partial outputs make the sensor emit on every step rather than only at scan
boundaries. Together they give one full scan per step, which is what makes step times comparable across runs. Turn
either off with ``--no-instant-lidar`` or ``--no-partial-outputs`` to profile the sensor's real cadence instead.

``--frame-rate`` defaults to the 10 Hz the generic lidar model is specified at. Raising it degenerates the model's
scan pattern -- the sensor keeps emitting frames but they carry no returns -- so the run fails rather than reporting a
time for empty work.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import tempfile
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

sys.dont_write_bytecode = True

from isaacsim.performance.benchmarking import benchmark  # noqa: E402

DEFAULT_OUTPUT_DIR = Path(
    os.environ.get("ISAACSIM_BENCHMARK_OUTPUT_DIR") or Path(tempfile.gettempdir()) / "isaacsim-benchmarks"
).expanduser()


def _positive_int(text: str) -> int:
    """Convert one argument to an integer of at least one."""
    value = int(text)
    if value < 1:
        raise argparse.ArgumentTypeError(f"{text} is not an integer of at least 1")
    return value


def positive_float(text: str) -> float:
    """Convert one argument to a number greater than zero."""
    value = float(text)
    if value <= 0.0:
        raise argparse.ArgumentTypeError(f"{text} is not a number greater than zero")
    return value


def parse_arguments(
    identifier: str,
    description: str,
    *,
    sensor_count: int = 1,
    frame_count: int = 1000,
    resolution: tuple[int, int] = (1280, 720),
    extend: Callable[[argparse.ArgumentParser], None] | None = None,
    argv: list[str] | None = None,
) -> argparse.Namespace:
    """Parse the benchmark command line."""
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--num-sensors", type=_positive_int, default=sensor_count, help="Number of sensors to place.")
    parser.add_argument("--num-frames", type=_positive_int, default=frame_count, help="Number of measured frames.")
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=100,
        help="Frames rendered before measurement starts, to settle shader compilation and streaming.",
    )
    parser.add_argument(
        "--resolution",
        nargs=2,
        type=_positive_int,
        default=list(resolution),
        metavar=("WIDTH", "HEIGHT"),
        help="Render product resolution in pixels.",
    )
    parser.add_argument(
        "--time-step",
        dest="delta_time",
        type=positive_float,
        default=1.0 / 60.0,
        help="Simulated time between rendered frames, in seconds.",
    )
    parser.add_argument(
        "--readback",
        choices=("none", "cpu", "cuda"),
        default="cpu",
        help="Device the sensor output is mapped to each frame. 'none' renders without reading results back.",
    )
    parser.add_argument(
        "--gpus",
        dest="active_cuda_gpus",
        default=None,
        metavar="INDICES",
        help="Comma-separated CUDA device indices for the renderer, for example '0,1'.",
    )
    parser.add_argument(
        "--renderer-profiling",
        action="store_true",
        help="Let the renderer emit its own internal profiling events alongside the benchmark's. Adds overhead.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT_DIR / f"{identifier.rsplit('.', 1)[-1]}_result.json",
        help="Result JSON path. The renderer log is written beside it.",
    )
    parser.add_argument(
        "--scene",
        default=None,
        metavar="PATH_OR_URL",
        help="USD file or URL sublayered under the authored environment. Sensors are always authored on top.",
    )
    parser.add_argument(
        "--print-stage",
        action="store_true",
        help="Print the composed USDA and exit without creating a renderer.",
    )
    parser.add_argument(
        "--export-stage",
        nargs="?",
        const="",
        default=None,
        metavar="PATH",
        help="Write the composed USDA to PATH, or to <output-parent>/<name>_stage.usda when PATH is omitted, then "
        "carry on with the run. The written layer is openable in usdview.",
    )
    # Capture common options before adding sensor-specific ones.
    shared = frozenset(vars(parser.parse_args([])))
    if extend is not None:
        extend(parser)
    parsed = parser.parse_args(argv)

    if parsed.warmup_frames < 0:
        parser.error("--warmup-frames cannot be negative")
    # Check defaults too: argparse skips type conversion for non-string defaults.
    output = parsed.output.expanduser()
    if output.parent.is_symlink():
        parser.error(f"--output parent must not be a symbolic link: {output.parent}")

    name = identifier.rsplit(".", 1)[-1]
    export_stage = None
    if parsed.export_stage is not None:
        export_stage = Path(parsed.export_stage) if parsed.export_stage else output.parent / f"{name}_stage.usda"

    sensor_options = ", ".join(
        f"{key}=" + ("+".join(str(item) for item in value) if isinstance(value, (list, tuple)) else str(value))
        for key, value in sorted(vars(parsed).items())
        if key not in shared
    )

    parsed.output = output
    parsed.identifier = identifier
    parsed.name = name
    parsed.sensor_count = vars(parsed).pop("num_sensors")
    parsed.frame_count = vars(parsed).pop("num_frames")
    parsed.resolution = tuple(parsed.resolution)
    parsed.export_stage = export_stage
    parsed.sensor_options = sensor_options
    return parsed


def render_vars_by_name(frame: Any) -> dict[str, Any]:
    """Map one frame's render variables by the name authored on each RenderVar prim."""
    return {str(path).rsplit("/", 1)[-1]: render_var for path, render_var in frame.render_vars.items()}


def is_host_device(device: Any) -> bool:
    """Report whether a mapping device puts the data in host memory."""
    import ovrtx

    return device == ovrtx.Device.CPU


def to_array(source: Any, device: Any, *, host: bool = False) -> Any:
    """View DLPack data as an array, optionally copying CUDA data to the host."""
    import ovrtx

    if device == ovrtx.Device.CUDA:
        import warp

        array = warp.from_dlpack(source)
        return array.numpy() if host else array
    import numpy

    return numpy.from_dlpack(source)


_OBSTACLE_RADII = (6.0, 11.0, 17.0, 24.0)


_OBSTACLES_PER_RING = 9


_OBSTACLE_MATERIALS = ("Concrete", "Steel", "Plastic")


def sensor_ring_poses(count: int, *, radius: float = 0.0, height: float = 1.5) -> list[tuple[str, str]]:
    """Place sensors on a ring around the origin, each looking outward."""
    poses: list[tuple[str, str]] = []
    for index in range(count):
        yaw = index * 360.0 / count
        radians = math.radians(yaw)
        translate = (radius * math.cos(radians), radius * math.sin(radians), height)
        poses.append((_vector(translate), _vector((90.0, 0.0, -90.0 + yaw))))
    return poses


def environment_usda() -> str:
    """Author the ground, lighting, and obstacles every benchmark renders against."""
    obstacles: list[str] = []
    index = 0
    for ring, radius in enumerate(_OBSTACLE_RADII):
        for step in range(_OBSTACLES_PER_RING):
            # Offset each ring so obstacles do not shadow each other along a single bearing.
            radians = math.radians(step * 360.0 / _OBSTACLES_PER_RING + ring * 18.0)
            width = 0.8 + 0.3 * (step % 3)
            tall = 1.2 + 0.6 * ((step + ring) % 4)
            material = _OBSTACLE_MATERIALS[index % len(_OBSTACLE_MATERIALS)]
            obstacles.append(f"""    def Cube "Obstacle_{index:02d}" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    )
    {{
        double size = 1
        double3 xformOp:translate = {_vector((radius * math.cos(radians), radius * math.sin(radians), tall / 2.0))}
        double3 xformOp:scale = {_vector((width, width, tall))}
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        rel material:binding = </World/Materials/{material}>
    }}""")
            index += 1
    materials = "\n\n".join(_nonvisual_material(name) for name in ("Asphalt", *_OBSTACLE_MATERIALS))
    obstacle_block = "\n\n".join(obstacles)
    return f"""    def Scope "Materials"
    {{
{materials}
    }}

    def DistantLight "SunLight"
    {{
        float inputs:angle = 1
        float inputs:intensity = 3000
        double3 xformOp:rotateXYZ = (55, 0, 20)
        uniform token[] xformOpOrder = ["xformOp:rotateXYZ"]
    }}

    def DomeLight "SkyLight"
    {{
        float inputs:intensity = 400
    }}

    def Cube "Ground" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    )
    {{
        double size = 1
        double3 xformOp:translate = (0, 0, -0.5)
        double3 xformOp:scale = (80, 80, 1)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        rel material:binding = </World/Materials/Asphalt>
    }}

{obstacle_block}
"""


def compose_stage(sensors: str, render: str, *, scene: str | None = None) -> str:
    """Compose one benchmark stage from its sensors, its render products, and its environment."""
    if scene is None:
        header = """#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "World"
{
"""
        world = header + environment_usda() + "}\n"
    else:
        # Anchor local scenes so running and exporting resolve the same layer; preserve URLs.
        target = scene if "://" in scene else Path(scene).expanduser().resolve().as_posix()
        # The supplied layer keeps its own stage metadata, including its up axis and units.
        world = f"""#usda 1.0
(
    subLayers = [
        {_asset_path(target)}
    ]
)
"""
    return f"""{world}
def Scope "Sensors"
{{
{sensors}}}

def Scope "Render"
{{
{render}}}
"""


def _asset_path(target: str) -> str:
    """Write one file path or URL as a USDA asset reference."""
    if "@" not in target:
        return f"@{target}@"
    # Use triple delimiters for paths containing `@`, escaping embedded `@@@`.
    return "@@@" + target.replace("@@@", r"\@@@") + "@@@"


def _nonvisual_material(name: str) -> str:
    """Author a material whose only purpose is to label a surface for the non-visual sensors."""
    base = name.lower()
    return f"""        def Material "{name}"
        {{
            token omni:simready:nonvisual:base = "{base}"
            token omni:simready:nonvisual:coating = "none"
            token[] omni:simready:nonvisual:attributes = ["none"]
            custom string inputs:nonvisual:base = "{base}"
            custom string inputs:nonvisual:coating = "none"
            custom string inputs:nonvisual:attributes = "none"
        }}"""


def _vector(values: tuple[float, ...]) -> str:
    """Format a numeric tuple as a USDA vector literal."""
    # Trigonometry on the sensor ring yields negative zeros, which format as an "-0" literal.
    return "(" + ", ".join(f"{value if value else 0.0:g}" for value in values) + ")"


FrameReader = Callable[[Any, Any | None], Mapping[str, float]]


_POPULATION_ORDINAL = 1


_DEFAULT_COUNTER_UNIT = "count"


def run_benchmark(
    arguments: argparse.Namespace,
    *,
    sensors: str,
    render: str,
    product_paths: tuple[str, ...],
    read_frame: FrameReader,
    renderer_config: Mapping[str, Any] | None = None,
    payload_counter: str | None = None,
    counter_units: Mapping[str, str] | None = None,
) -> int:
    """Measure a sensor rig over the requested number of frames and write the run's result."""
    stage_text = compose_stage(sensors, render, scene=arguments.scene)
    if arguments.export_stage is not None:
        arguments.export_stage.parent.mkdir(parents=True, exist_ok=True)
        arguments.export_stage.write_text(stage_text)
        print(f"Stage written to {arguments.export_stage}", flush=True)
    if arguments.print_stage:
        print(stage_text, end="")
        return 0

    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    document = benchmark.run(
        _define_benchmark(
            arguments,
            sensors=sensors,
            render=render,
            product_paths=product_paths,
            read_frame=read_frame,
            renderer_config=renderer_config,
            payload_counter=payload_counter,
            counter_units=counter_units,
            stage_text=stage_text,
        ),
        output=arguments.output,
    )
    _report(document)
    return 0 if document["status"]["valid"] else 1


def _define_benchmark(
    arguments: argparse.Namespace,
    *,
    sensors: str,
    render: str,
    product_paths: tuple[str, ...],
    read_frame: FrameReader,
    renderer_config: Mapping[str, Any] | None = None,
    payload_counter: str | None = None,
    counter_units: Mapping[str, str] | None = None,
    stage_text: str,
) -> type:
    """Declare the rig's measured workflow to the benchmarking library."""

    class SensorBenchmark:
        """Render one ovrtx sensor rig for one frame per measured sample."""

        def __init__(
            self,
            sensor_count: int = arguments.sensor_count,
            width: int = arguments.resolution[0],
            height: int = arguments.resolution[1],
            delta_time_s: float = arguments.delta_time,
            readback: str = arguments.readback,
            active_cuda_gpus: str | None = arguments.active_cuda_gpus,
            renderer_profiling: bool = arguments.renderer_profiling,
            scene: str | None = arguments.scene,
            sensor_options: str = arguments.sensor_options,
        ) -> None:
            self._delta_time_s = delta_time_s
            self._readback = readback
            self._active_cuda_gpus = active_cuda_gpus
            self._renderer_profiling = renderer_profiling
            self._products = set(product_paths)
            self._renderer: Any = None
            self._stage: Any = None
            self._device: Any = None
            self._steps = 0
            self._frames_produced = 0
            self._payload_recorded = False
            self._payload_total = 0.0

        def setup(self) -> None:
            """Create the renderer and populate it with the composed stage."""
            import ovrtx
            import ovstage

            # Register sensor schemas before populating the stage.
            ovrtx.register_schema_paths()
            self._device = {"cpu": ovrtx.Device.CPU, "cuda": ovrtx.Device.CUDA, "none": None}[self._readback]
            config = ovrtx.RendererConfig(
                log_file_path=str(arguments.output.parent / f"{arguments.name}_ovrtx.log"),
                active_cuda_gpus=self._active_cuda_gpus,
                enable_profiling=self._renderer_profiling or None,
                **(renderer_config or {}),
            )
            with benchmark.measure("renderer_create"):
                self._renderer = ovrtx.Renderer(config=config)
                self._stage = ovstage.Stage(f"benchmark.{arguments.name}")
                self._renderer.attach_ovstage(self._stage)

            with benchmark.measure("stage_population"):
                ovstage.population.open_usd_from_string(
                    self._stage,
                    stage_text,
                    ordinal=_POPULATION_ORDINAL,
                    domains=ovstage.PopulationDomain.RENDERING,
                )
                # Publish the populated stage before stepping the renderer.
                self._stage.advance_write_floor(ordinal=_POPULATION_ORDINAL).wait()

        def run(self) -> None:
            """Render one frame through every sensor and read back what it produced."""
            self._steps += 1
            # Exclude warmup output so validation cannot pass on scans produced only during warmup.
            measured = self._steps > arguments.warmup_frames
            with benchmark.measure("step"):
                stepped = self._renderer.step(
                    render_products=self._products,
                    delta_time=self._delta_time_s,
                    ordinal=_POPULATION_ORDINAL,
                )
            read = 0
            with benchmark.measure("readback"):
                for product in stepped.values():
                    for frame in product.frames:
                        read += 1
                        for label, value in read_frame(frame, self._device).items():
                            benchmark.record(label, value, (counter_units or {}).get(label, _DEFAULT_COUNTER_UNIT))
                            if measured and label == payload_counter:
                                self._payload_recorded = True
                                self._payload_total += value
            if measured:
                self._frames_produced += read
            benchmark.record("sensor_frames", float(read), _DEFAULT_COUNTER_UNIT)
            # Release the step result and its output buffers before renderer teardown.
            del stepped

        def validate(self) -> bool:
            """Report whether the measured frames carried sensor data."""
            if self._frames_produced == 0:
                raise RuntimeError(
                    f"The renderer produced no sensor frame in {arguments.frame_count} steps. A sensor that scans "
                    f"slower than the step rate needs more steps to complete one, so raise --num-frames or the "
                    f"sensor's own rate."
                )
            if payload_counter is not None and self._payload_recorded and self._payload_total == 0.0:
                raise RuntimeError(
                    f"The renderer produced {self._frames_produced} sensor frames but every one was empty "
                    f"({payload_counter} stayed at zero). The sensor configuration is not producing data, so "
                    f"the run would measure empty work. Check the sensor's rate against what its model supports."
                )
            return True

        def teardown(self) -> None:
            """Release the renderer and the stage."""
            # Destroy the renderer first so it can detach the stage.
            if self._renderer is not None:
                self._renderer.destroy()
                self._renderer = None
            if self._stage is not None:
                self._stage.destroy()
                self._stage = None

    return benchmark.workflow(
        arguments.identifier,
        component="ovrtx",
        resources=("cpu", "gpu"),
        metrics=("gpu", "process", "simulation"),
        warmup_frames=arguments.warmup_frames,
        measured_frames=arguments.frame_count,
        # Resolve simulated time from the run parameter, not its default.
        simulated_time="delta_time_s",
    )(SensorBenchmark)


def _report(document: Mapping[str, Any]) -> None:
    """Print the measured frame-time detail and any error the run recorded."""
    statistics = document["metrics"].get("duration_ms", {}).get("statistics", {})
    if statistics:
        print(
            f"{statistics['count']} frames, p50 {statistics['p50']:.2f} ms, p99 {statistics['p99']:.2f} ms, "
            f"min {statistics['minimum']:.2f} ms, max {statistics['maximum']:.2f} ms",
            flush=True,
        )
    for error in document["status"].get("errors", ()):
        print(f"Error: {error}", flush=True)


CHANNELS = ("Coordinates", "Intensity", "Counts", "Flags", "TimeOffsetNs")


VALID_FLAG = 0x40


def _extend(parser: argparse.ArgumentParser) -> None:
    """Add the lidar-specific options."""
    parser.add_argument(
        "--frame-rate",
        type=positive_float,
        default=10.0,
        help="Scan rate in Hz. The generic lidar model is specified at 10 Hz and degenerates above it.",
    )
    parser.add_argument(
        "--coords-type",
        choices=("CARTESIAN", "SPHERICAL"),
        default="CARTESIAN",
        help="Coordinate representation of the returned points.",
    )
    parser.add_argument(
        "--frame-of-reference",
        choices=("SENSOR", "WORLD"),
        default="SENSOR",
        help="Frame the returned points are expressed in.",
    )
    parser.add_argument(
        "--no-instant-lidar",
        dest="instant_lidar",
        action="store_false",
        help="Scan at the sensor's own rate instead of emitting one full scan per step.",
    )
    parser.add_argument(
        "--no-partial-outputs",
        dest="partial_outputs",
        action="store_false",
        help="Emit only at scan boundaries instead of on every step.",
    )
    parser.add_argument(
        "--include-invalid-points",
        action="store_true",
        help="Keep invalid returns in the point tensors, so the reader filters them by Flags.",
    )
    parser.add_argument(
        "--motion-bvh",
        choices=("auto", "enable", "disable"),
        default="auto",
        help="Motion BVH mode, which governs how the sensor pipeline handles moving geometry.",
    )


def _author(count: int, options: argparse.Namespace, frame_rate: float) -> tuple[str, str, tuple[str, ...]]:
    """Author the lidar prims and their point cloud render products."""
    channels = ",\n".join(f'                "{channel}"' for channel in CHANNELS)
    sensors: list[str] = []
    products: list[str] = []
    paths: list[str] = []
    for index, (translate, rotate) in enumerate(sensor_ring_poses(count)):
        sensors.append(f"""    def OmniLidar "Lidar_{index:03d}" (
        prepend apiSchemas = ["OmniSensorGenericLidarCoreAPI"]
    )
    {{
        token omni:sensor:Core:elementsCoordsType = "{options.coords_type}"
        token omni:sensor:Core:outputFrameOfReference = "{options.frame_of_reference}"
        bool omni:sensor:Core:instantLidar = {int(options.instant_lidar)}
        bool omni:sensor:Core:partialOutputs = {int(options.partial_outputs)}
        bool omni:sensor:Core:includeInvalidPoints = {int(options.include_invalid_points)}
        double2 omni:sensor:frameRate = ({frame_rate:g}, 1)
        double3 xformOp:translate = {translate}
        double3 xformOp:rotateXYZ = {rotate}
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ"]
    }}
""")
        products.append(f"""    def RenderProduct "LidarProduct_{index:03d}"
    {{
        rel camera = </Sensors/Lidar_{index:03d}>
        rel orderedVars = [<PointCloud>]

        def RenderVar "PointCloud"
        {{
            uniform string sourceName = "PointCloud"
            token[] channels = [
{channels}
            ]
        }}
    }}
""")
        paths.append(f"/Render/LidarProduct_{index:03d}")
    return "\n".join(sensors), "\n".join(products), tuple(paths)


def _read_frame(frame: Any, device: Any | None) -> Mapping[str, float]:
    """Map one lidar point cloud and report the returns it carries."""
    if device is None:
        return {}
    with render_vars_by_name(frame)["PointCloud"].map(device=device) as pointcloud:
        # Counts is a single int32 and bounds every per-point channel, which are allocated for the sensor's worst case.
        delivered = int(to_array(pointcloud["Counts"], device, host=True)[0])
        coordinates = to_array(pointcloud["Coordinates"], device)
        intensity = to_array(pointcloud["Intensity"], device)
        counters = {"points": float(delivered), "coordinate_components": float(coordinates.shape[0] * delivered)}
        if is_host_device(device):
            flags = to_array(pointcloud["Flags"], device)[:delivered]
            counters["valid_points"] = float((flags & VALID_FLAG).astype(bool).sum())
            if delivered > 0:
                counters["mean_intensity"] = float(intensity[:delivered].mean())
        return counters


def main() -> int:
    """Run the lidar benchmark."""
    arguments = parse_arguments("benchmark.ovrtx.lidar", __doc__ or "", extend=_extend)
    sensors, render, paths = _author(arguments.sensor_count, arguments, arguments.frame_rate)
    return run_benchmark(
        arguments,
        sensors=sensors,
        render=render,
        product_paths=paths,
        read_frame=_read_frame,
        renderer_config={"motion_bvh": arguments.motion_bvh},
        payload_counter="points",
        counter_units={"mean_intensity": "intensity"},
    )


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        pass
