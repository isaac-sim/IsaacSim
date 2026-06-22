# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Verify ROS 2 RTX sensor publishing.

Covers lidar and radar point cloud or laser scan output, simulation and system
timestamps, metadata gating, frame transform behavior, and optional intensity fields.
"""

import asyncio
import json
import math
import time
from typing import Any
from uuid import uuid4

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.sensors.experimental.rtx.generic_model_output as gmo_utils
import numpy as np
import omni
import omni.graph.core as og
import omni.kit
import omni.kit.test
import omni.replicator.core as rep
import rclpy
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase
from isaacsim.sensors.experimental.rtx import Lidar, parse_generic_model_output_data
from omni.replicator.core import Writer
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String

from .common import create_sarcophagus, get_qos_profile


class _GmoCollectorWriter(Writer):
    """Replicator Writer that captures every ``GenericModelOutput`` in arrival order.

    Frames are deduped by ``gmo.timestampNs`` (the lidar's authoritative scan-start time).

    A Writer is used instead of per-frame annotator polling so every GMO produced by
    the SD pipeline is observed in lockstep with the publisher writers on the same
    render product.
    """

    def __init__(self) -> None:
        # initialize() re-invokes __init__, so per-test state lives here.
        self.version = "0.0.1"
        self.data_structure = "renderProduct"
        self.annotators = [rep.annotators.get("GenericModelOutput")]
        self.snapshots: list[dict] = []
        self._seen_timestamp_ns: set = set()

    def write(self, data: dict) -> None:
        rps = data.get("renderProducts") if isinstance(data, dict) else None
        if not rps:
            return
        for rp_data in rps.values():
            entry = rp_data.get("GenericModelOutput")
            if isinstance(entry, dict):
                entry = entry.get("data")
            if entry is None:
                continue
            try:
                gmo = parse_generic_model_output_data(entry)
            except Exception:
                continue
            if gmo.magicNumber != gmo_utils.getMagicNumberGMO() or gmo.numElements == 0:
                continue
            ts_ns = int(gmo.timestampNs)
            if ts_ns in self._seen_timestamp_ns:
                continue
            self._seen_timestamp_ns.add(ts_ns)
            self.snapshots.append(
                {
                    "x": gmo.x.copy(),
                    "y": gmo.y.copy(),
                    "z": gmo.z.copy(),
                    "scalar": gmo.scalar.copy(),
                    "numElements": int(gmo.numElements),
                    "elementsCoordsType": gmo.elementsCoordsType,
                    "timestampNs": ts_ns,
                    "timeOffsetNs": gmo.timeOffsetNs.copy(),
                }
            )


class TestROS2SensorMsgRTX(ROS2TestCase):
    """Base fixture for RTX sensor publisher tests using ROS 2 subscriptions."""

    _ros_msg_type = None
    _helper_type = None

    async def setUp(self) -> None:
        """Create the test scene, ROS 2 subscriptions, and object ID map listener."""
        await super().setUp()

        self._sensor_prim_path = None
        self._render_product = None
        self._render_product_path = None
        self._annotator_rtx = None
        self._annotator_timestamp = None
        self._mapped_annotator_data = {}

        # Add cubes to the scene
        self._cubes = create_sarcophagus()

        # Configure the ROS2 subscriber to capture the messsge

        self._ros_topic = f"topic_{uuid4().hex}"
        self._ros_msg_data = None
        self._ros_node = self.create_node(f"subscriber_{self._ros_topic}")
        self._ros_msg_count = 0
        self._ros_msg_timestamp_prev = None
        self._is_full_scan = False

        def ros_callback(data: Any) -> None:
            self._ros_msg_data = data
            self._ros_msg_count += 1

            # Validate the message timestamp
            if self._ros_msg_timestamp_prev is not None:
                current_timestamp = self._ros_msg_data.header.stamp.sec + self._ros_msg_data.header.stamp.nanosec / 1e9
                expected_diff = 1 / 10 if self._is_full_scan else 1 / 60
                self.assertAlmostEqual(current_timestamp, self._ros_msg_timestamp_prev + expected_diff)
                self._ros_msg_timestamp_prev = current_timestamp

        self._ros_sub = self.create_subscription(
            self._ros_node, self._ros_msg_type, self._ros_topic, ros_callback, get_qos_profile(depth=10)
        )

        # Configure the ROS2 subscriber to capture the Object-ID-to-prim-path map
        self._ros_object_id_map_topic = f"object_id_map_{uuid4().hex}"
        self._ros_object_id_map_data = None
        self._ros_object_id_map_node = self.create_node(f"subscriber_{self._ros_object_id_map_topic}")
        self._ros_object_id_map_count = 0
        self._ros_object_id_map_timestamp_prev = None

        def ros_callback_object_id_map(data: Any) -> None:
            self._ros_object_id_map_data = data
            self._ros_object_id_map_count += 1
            prim_paths = json.loads(data.data)["id_to_labels"].values()
            for i in range(15):
                self.assertIn(f"/World/cube_{i}", prim_paths)

        self._ros_sub_object_id_map = self.create_subscription(
            self._ros_object_id_map_node,
            String,
            self._ros_object_id_map_topic,
            ros_callback_object_id_map,
            get_qos_profile(depth=10),
        )

    async def tearDown(self) -> None:
        """Detach RTX annotators before the ROS 2 test case cleanup runs."""
        if self._annotator_rtx is not None:
            self._annotator_rtx.detach()
        if self._annotator_timestamp is not None:
            self._annotator_timestamp.detach()
        await super().tearDown()

    async def _create_sensor(
        self,
        sensor_type: str,
        config: str = None,
        variant: str = None,
        aux_output_level: str = "NONE",
        accumulate_outputs: bool = True,
        **kwargs: Any,
    ) -> None:
        self._sensor_type = sensor_type

        if sensor_type == "radar":
            from isaacsim.sensors.experimental.rtx import Radar

            radar = Radar(
                f"/{sensor_type}",
                aux_output_level=aux_output_level,
                attributes=kwargs if kwargs else None,
            )
            prim = radar.prims[0]
            self.assertEqual(prim.GetTypeName(), "OmniRadar", f"Failed to create {sensor_type}.")
            self._sensor_prim_path = radar.paths[0]
        else:
            # Create an RTX Lidar sensor using the Lidar authoring class
            lidar = Lidar.create(
                f"/{sensor_type}",
                config=config,
                variant=variant,
                aux_output_level=aux_output_level,
                accumulate_outputs=accumulate_outputs,
                attributes=kwargs if kwargs else None,
            )
            prim = lidar.prims[0]
            self.assertEqual(prim.GetTypeName(), "OmniLidar", f"Failed to create {sensor_type}.")
            self._sensor_prim_path = lidar.paths[0]

        # Create a render product for the sensor
        self._render_product = rep.create.render_product(self._sensor_prim_path, resolution=(128, 128))
        self._render_product_path = self._render_product.path

    async def _create_omnigraph(
        self, enable_full_scan: bool = False, use_system_time: bool = False, metadata: list[str] | None = None
    ) -> None:
        if metadata is None:
            metadata = []
        # Create the basic Omnigraph for the sensor
        create_nodes = [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("PCLPublish", f"isaacsim.ros2.bridge.ROS2Rtx{self._sensor_type.capitalize()}Helper"),
        ]
        set_values = [
            ("PCLPublish.inputs:renderProductPath", self._render_product_path),
            ("PCLPublish.inputs:topicName", self._ros_topic),
            ("PCLPublish.inputs:resetSimulationTimeOnStop", True),
            ("PCLPublish.inputs:useSystemTime", use_system_time),
        ]
        # Object ID map is only supported for lidar
        if self._sensor_type == "lidar":
            set_values.append(("PCLPublish.inputs:enableObjectIdMap", "objectId" in metadata))
            set_values.append(("PCLPublish.inputs:objectIdMapTopicName", self._ros_object_id_map_topic))
        connections = [
            ("OnPlaybackTick.outputs:tick", "PCLPublish.inputs:execIn"),
        ]

        # Specify metadata based on sensor type
        if self._sensor_type == "lidar":
            # Use OgnROS2RtxLidarPointCloudConfig to specify metadata for Lidar
            create_nodes.append(("PCLLidarConfig", "isaacsim.ros2.bridge.ROS2RtxLidarPointCloudConfig"))
            set_values.append(("PCLPublish.inputs:type", self._helper_type))
            for metadata_item in metadata:
                set_values.append((f"PCLLidarConfig.inputs:output{metadata_item[0].upper()}{metadata_item[1:]}", True))
            connections.append(("PCLLidarConfig.outputs:selectedMetadata", "PCLPublish.inputs:selectedMetadata"))
        elif self._sensor_type == "radar":
            # Enable metadata directly on OgnROS2RtxRadarHelper
            for metadata_item in metadata:
                set_values.append((f"PCLPublish.inputs:output{metadata_item[0].upper()}{metadata_item[1:]}", True))

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: create_nodes,
                    og.Controller.Keys.SET_VALUES: set_values,
                    og.Controller.Keys.CONNECT: connections,
                },
            )
        except Exception as e:
            self.fail(f"Failed to create OmniGraph for {self._sensor_type}: {e}")

    def _get_expected_message_count(self, full_scan: bool = False, test_duration_s: float = 1.5) -> Any:
        if self._sensor_type == "lidar":
            return test_duration_s * 10 * (1 if full_scan else 6) - 1
        elif self._sensor_type == "radar":
            # The first 4 frames are warmup frames, so we don't get messages from them
            return test_duration_s * 60 - 4

    def _get_closest_timestamp(self, message_timestamp: Any) -> Any:
        """Return the snapshot whose annotator-observed wall-clock is closest to *message_timestamp*.

        Snapshots are keyed by ``gmo.timestampNs`` (scan identity) and each entry tracks
        every annotator-side timestamp at which the scan was observed; we search across
        all observations so a snapshot still matches if the GMO annotator and the
        publisher latched the same scan on different simulation frames.

        Args:
            message_timestamp: Message timestamp to match.

        Returns:
            Closest matching timestamp.
        """
        best_key = None
        best_diff = float("inf")
        for scan_key, snap in self._mapped_annotator_data.items():
            for ts in snap["annotator_timestamps"]:
                diff = abs(ts - message_timestamp)
                if diff < best_diff:
                    best_diff = diff
                    best_key = scan_key
        return self._mapped_annotator_data[best_key]

    async def _test_message_data(
        self, full_scan: bool = False, test_duration_s: float = 1.5, metadata: list[str] | None = None
    ) -> None:
        raise NotImplementedError("Subclasses must implement this method.")

    async def _test_sensor(
        self,
        sensor_type: str,
        aux_output_level: str = "NONE",
        full_scan: bool = False,
        use_system_time: bool = False,
        metadata: list[str] | None = None,
        test_duration_s: float = 1.5,
        **kwargs: Any,
    ) -> Any:
        if metadata is None:
            metadata = []
        self._is_full_scan = full_scan
        await self._create_sensor(
            sensor_type, aux_output_level=aux_output_level, accumulate_outputs=full_scan, **kwargs
        )
        await self._create_omnigraph(enable_full_scan=full_scan, use_system_time=use_system_time, metadata=metadata)

        # Start the timeline and advance by 1 frame to create the post-process graph
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # Retrieve the annotator and attach it to the render product
        annotator_name = "GenericModelOutput"
        self._annotator_rtx = rep.AnnotatorRegistry.get_annotator(annotator_name)
        self.assertIsNotNone(
            self._annotator_rtx,
            f"Failed to retrieve annotator {annotator_name} for {self._sensor_type}.",
        )
        self._annotator_rtx.attach([self._render_product_path])

        # Retrieve the timestamp annotator and attach it to the render product
        annotator_timestamp_name = "IsaacRead" + ("SystemTime" if use_system_time else "SimulationTime")
        self._annotator_timestamp = rep.AnnotatorRegistry.get_annotator(annotator_timestamp_name)
        self.assertIsNotNone(
            self._annotator_timestamp,
            f"Failed to retrieve {annotator_timestamp_name} annotator for {self._sensor_type}.",
        )
        self._annotator_timestamp.attach([self._render_product_path])

        # Define the callback function for the simulation
        timestamp_output = "systemTime" if use_system_time else "simulationTime"

        # Auxiliary fields available at each GMO aux level (cumulative).
        _AUX_FIELDS = {
            gmo_utils.AuxType.BASIC: [
                ("emitterId", "emitterId"),
                ("channelId", "channelId"),
                ("tickId", "tickId"),
                ("echoId", "echoId"),
                ("tickStates", "tickStates"),
            ],
            gmo_utils.AuxType.EXTRA: [("matId", "matId"), ("objId", "objId")],
            gmo_utils.AuxType.FULL: [("hitNormals", "hitNormals"), ("velocities", "velocities")],
        }
        if self._sensor_type == "radar":
            _AUX_FIELDS[gmo_utils.AuxType.BASIC] = [("radialVelocityMS", "rv_ms")]

        def _aux_fields_for_level(aux_type: Any) -> Any:
            """Return the list of (snapshot_key, gmo_attr) pairs available at *aux_type*.

            Args:
                aux_type: Auxiliary metadata level.

            Returns:
                Snapshot key and GMO attribute pairs.
            """
            fields = []
            for level in (gmo_utils.AuxType.BASIC, gmo_utils.AuxType.EXTRA, gmo_utils.AuxType.FULL):
                if aux_type == gmo_utils.AuxType.NONE:
                    break
                fields += _AUX_FIELDS[level]
                if aux_type == level:
                    break
            return fields

        def spin() -> None:
            # Drive ROS callbacks and snapshot the latest GMO, deduping repeat frames
            # by gmo.timestampNs (scan identity) while still recording every observed
            # annotator wall-clock for _get_closest_timestamp(). Arrays are copied
            # eagerly because the GMO struct points into a buffer that gets overwritten
            # on the next frame.
            rclpy.spin_once(self._ros_node, timeout_sec=0.01)
            rclpy.spin_once(self._ros_object_id_map_node, timeout_sec=0.01)

            raw_data = self._annotator_rtx.get_data(do_array_copy=True)
            timestamp_data = self._annotator_timestamp.get_data()
            timestamp = timestamp_data.get(timestamp_output) if timestamp_data is not None else None
            if timestamp is None or raw_data is None or raw_data.size == 0:
                return
            gmo = parse_generic_model_output_data(raw_data)
            if gmo.magicNumber != gmo_utils.getMagicNumberGMO() or gmo.numElements == 0:
                return
            scan_key = int(gmo.timestampNs)
            existing = self._mapped_annotator_data.get(scan_key)
            if existing is not None:
                existing["annotator_timestamps"].append(timestamp)
                return
            snapshot = {
                "x": gmo.x.copy(),
                "y": gmo.y.copy(),
                "z": gmo.z.copy(),
                "scalar": gmo.scalar.copy(),
                "numElements": gmo.numElements,
                "elementsCoordsType": gmo.elementsCoordsType,
                "timestampNs": gmo.timestampNs,
                "timeOffsetNs": gmo.timeOffsetNs.copy(),
                "annotator_timestamps": [timestamp],
            }
            # Aux fields are only safe to access at or below the reported auxType
            # (otherwise the C++ GMO extension segfaults on a null auxiliaryData).
            for field, attr in _aux_fields_for_level(gmo.auxType):
                arr = getattr(gmo, attr)
                if arr is not None and hasattr(arr, "copy"):
                    snapshot[field] = arr.copy()
            self._mapped_annotator_data[scan_key] = snapshot

        system_time_start = time.time() if use_system_time else None

        def message_ready() -> Any:
            if self._ros_msg_data is None:
                return False

            stamp = self._ros_msg_data.header.stamp.sec + self._ros_msg_data.header.stamp.nanosec / 1e9
            if stamp < 0.2:
                return False
            if system_time_start is not None and stamp < system_time_start:
                return False

            # For LaserScan we must also wait past warm-up frames where some fields may be unset (e.g. inf).
            # Otherwise assertions in TestROS2LaserScanRTX can run on an invalid scan.
            if isinstance(self._ros_msg_data, LaserScan):
                if not math.isfinite(self._ros_msg_data.scan_time) or self._ros_msg_data.scan_time <= 0.0:
                    return False
                if not math.isfinite(self._ros_msg_data.time_increment) or self._ros_msg_data.time_increment <= 0.0:
                    return False
                if self._ros_msg_data.ranges is None or len(self._ros_msg_data.ranges) == 0:
                    return False
                # At least one range sample should be finite.
                if not any(math.isfinite(r) for r in self._ros_msg_data.ranges):
                    return False

            # Ensure annotator data is available for lookup in _get_closest_timestamp().
            if not self._mapped_annotator_data:
                return False

            # If object-id map publishing is enabled, wait until we receive it too.
            if self._sensor_type == "lidar" and "objectId" in metadata and self._ros_object_id_map_data is None:
                return False

            return True

        condition_met = await self.simulate_until_condition(
            message_ready,
            max_frames=600,
            per_frame_callback=spin,
        )
        self.assertTrue(
            condition_met,
            f"No valid message received for {self._sensor_type} after {test_duration_s} simulated seconds.",
        )

        # self.assertEqual(
        #     self._ros_msg_count,
        #     expected_message_count,
        #     f"Expected {expected_message_count} messages, but received {self._ros_msg_count}.",
        # )

        # Get the annotator data for the latest received message

        await self._test_message_data(full_scan=full_scan, test_duration_s=test_duration_s, metadata=metadata)


class TestROS2PointCloudRTX(TestROS2SensorMsgRTX):
    """Verify RTX lidar PointCloud2 output against GenericModelOutput data."""

    _ros_msg_type = PointCloud2
    _helper_type = "point_cloud"

    @staticmethod
    def _snapshot_to_cartesian(snapshot: Any) -> Any:
        """Convert a GMO snapshot dict to Cartesian xyz, handling both coordinate types.

        Args:
            snapshot: GMO snapshot to convert.

        Returns:
            Cartesian point coordinates.
        """
        x_data = snapshot["x"]
        y_data = snapshot["y"]
        z_data = snapshot["z"]
        if snapshot["elementsCoordsType"] == gmo_utils.CoordsType.CARTESIAN:
            return np.stack([x_data, y_data, z_data], axis=1)
        # Spherical to Cartesian
        az = np.radians(x_data)
        el = np.radians(y_data)
        r = z_data
        rxy = r * np.cos(el)
        cart_x = rxy * np.cos(az)
        cart_y = rxy * np.sin(az)
        cart_z = r * np.sin(el)
        mask = r < 1e-6
        cart_x[mask] = 0.0
        cart_y[mask] = 0.0
        cart_z[mask] = 0.0
        return np.stack([cart_x, cart_y, cart_z], axis=1)

    async def _test_message_data(
        self, full_scan: bool = False, test_duration_s: float = 1.5, metadata: list[str] | None = None
    ) -> None:
        self.assertIsNotNone(self._ros_msg_data)
        message_timestamp = self._ros_msg_data.header.stamp.sec + self._ros_msg_data.header.stamp.nanosec / 1e9
        snap = self._get_closest_timestamp(message_timestamp)

        self.assertGreater(
            self._ros_msg_count,
            0,
            f"No message received for {self._sensor_type} after {test_duration_s} simulated seconds.",
        )

        from sensor_msgs_py.point_cloud2 import read_points

        msg_points = read_points(self._ros_msg_data)

        def test_match(
            message_data: list[np.ndarray],
            annotator_data: np.ndarray,
            field_name: str = None,
        ) -> None:
            message_data = np.concatenate([m[..., None] for m in message_data], axis=1)
            if len(annotator_data.shape) == 1:
                annotator_data = annotator_data[..., None]
            self.assertGreater(message_data.shape[0], 0, f"Empty message data for {field_name}")
            self.assertGreater(annotator_data.shape[0], 0, f"Empty annotator data for {field_name}")
            self.assertEqual(message_data.shape, annotator_data.shape, f"Shape mismatch for {field_name}")
            self.assertEqual(message_data.size, annotator_data.size, f"Size mismatch for {field_name}")
            self.assertTrue(
                np.allclose(message_data, annotator_data, atol=1e-6),
                f"Data mismatch for {field_name}: {message_data} != {annotator_data}",
            )

        # Test point cloud position data
        cartesian = self._snapshot_to_cartesian(snap)
        test_match([msg_points["x"], msg_points["y"], msg_points["z"]], cartesian, "position")

        if "intensity" in metadata:
            test_match([msg_points["intensity"]], snap["scalar"], "intensity")
        if "timestamp" in metadata:
            msg_timestamp = (
                np.left_shift(msg_points["timestamp_1"].astype(np.uint64), 32)[..., None]
                + msg_points["timestamp_0"].astype(np.uint64)[..., None]
            ).squeeze(axis=1)
            expected_ts = snap["timeOffsetNs"].astype(np.uint64) + snap["timestampNs"]
            test_match([msg_timestamp], expected_ts, "timestamp")
        if "emitterId" in metadata:
            test_match([msg_points["emitter_id"]], snap["emitterId"], "emitterId")
        if "channelId" in metadata:
            test_match([msg_points["channel_id"]], snap["channelId"], "channelId")
        if "materialId" in metadata:
            test_match([msg_points["material_id"]], snap["matId"], "materialId")
        if "tickId" in metadata:
            test_match([msg_points["tick_id"]], snap["tickId"], "tickId")
        if "hitNormal" in metadata:
            test_match(
                [msg_points["nx"], msg_points["ny"], msg_points["nz"]],
                snap["hitNormals"].reshape(-1, 3),
                "hitNormal",
            )
        if "velocity" in metadata:
            test_match(
                [msg_points["vx"], msg_points["vy"], msg_points["vz"]],
                snap["velocities"].reshape(-1, 3),
                "velocity",
            )
        if "objectId" in metadata:
            msg_objectId = np.stack(
                [
                    msg_points["object_id_0"],
                    msg_points["object_id_1"],
                    msg_points["object_id_2"],
                    msg_points["object_id_3"],
                ],
                axis=1,
            ).flatten()
            annotator_objectId = snap["objId"].view(np.uint32)
            test_match([msg_objectId], annotator_objectId, "objectId")
        if "echoId" in metadata:
            test_match([msg_points["echo_id"]], snap["echoId"], "echoId")
        if "tickState" in metadata:
            test_match([msg_points["tick_state"]], snap["tickStates"], "tickState")
        if "radialVelocityMS" in metadata:
            self.fail("Radial velocity MS is not supported for Lidar")

    async def test_rtx_lidar_full_scan_simulation_time_no_metadata(self) -> None:
        """Verify full-scan lidar PointCloud2 output with simulation time and no metadata."""
        await self._test_sensor(
            sensor_type="lidar", full_scan=True, use_system_time=False, metadata=[], test_duration_s=0.5
        )

    async def test_rtx_lidar_full_scan_simulation_time_partial_metadata(self) -> None:
        """Verify full-scan lidar PointCloud2 output with simulation time and echo metadata."""
        await self._test_sensor(
            sensor_type="lidar",
            full_scan=True,
            aux_output_level="BASIC",
            use_system_time=False,
            metadata=["echoId"],
            test_duration_s=0.5,
        )

    async def test_rtx_lidar_full_scan_simulation_time_full_metadata(self) -> None:
        """Verify full-scan lidar PointCloud2 output with simulation time and all metadata fields."""
        await self._test_sensor(
            sensor_type="lidar",
            full_scan=True,
            aux_output_level="FULL",
            use_system_time=False,
            metadata=[
                "intensity",
                "timestamp",
                "emitterId",
                "channelId",
                "materialId",
                "tickId",
                "hitNormal",
                "velocity",
                "objectId",
                "echoId",
                "tickState",
            ],
            test_duration_s=0.5,
        )

    async def test_rtx_lidar_full_scan_system_time_full_metadata(self) -> None:
        """Verify full-scan lidar PointCloud2 output with system time and all metadata fields."""
        await self._test_sensor(
            sensor_type="lidar",
            full_scan=True,
            use_system_time=True,
            aux_output_level="FULL",
            metadata=[
                "intensity",
                "timestamp",
                "emitterId",
                "channelId",
                "materialId",
                "tickId",
                "hitNormal",
                "velocity",
                "objectId",
                "echoId",
                "tickState",
            ],
            test_duration_s=0.5,
        )


class TestROS2LaserScanRTX(TestROS2SensorMsgRTX):
    """Verify RTX lidar LaserScan output against captured GenericModelOutput scans."""

    _ros_msg_type = LaserScan
    _helper_type = "laser_scan"

    _writer_registered = False

    async def setUp(self) -> None:
        """Replace the base subscriber with a buffered LaserScan subscriber and GMO writer."""
        await super().setUp()

        # Re-subscribe with a list-appending callback routed through a background
        # MultiThreadedExecutor. start_async_spinning must come before
        # create_subscription so the new sub binds to the executor's
        # ReentrantCallbackGroup; the base tearDown shuts the executor down.
        self.destroy_subscription(self._ros_node, self._ros_sub)
        self._ros_messages: list[LaserScan] = []
        self.start_async_spinning(self._ros_node)

        def ros_callback(data: Any) -> None:
            self._ros_messages.append(data)
            self._ros_msg_count += 1
            self._ros_msg_data = data  # keep for base-class introspection

        self._ros_sub = self.create_subscription(
            self._ros_node,
            self._ros_msg_type,
            self._ros_topic,
            ros_callback,
            get_qos_profile(depth=100),
        )

        # Attached in _test_sensor, detached in tearDown so assertions can bubble.
        self._gmo_writer = None

        if not TestROS2LaserScanRTX._writer_registered:
            rep.WriterRegistry.register(_GmoCollectorWriter)
            TestROS2LaserScanRTX._writer_registered = True

    async def tearDown(self) -> None:
        """Detach the GMO writer before shared RTX sensor cleanup runs."""
        if self._gmo_writer is not None:
            self._gmo_writer.detach()
            self._gmo_writer = None
        await super().tearDown()

    def _assert_match_with_bin_tolerance(self, expected: Any, actual: Any, atol: Any, name: Any) -> Any:
        """Elementwise match with ±1 slot tolerance.

        The publisher's per-ray binning uses the original float32 prim attributes
        (``OgnROS2PublishLaserScan::publishFromGMO``) while the test recovers them
        from the message's deg→rad-converted ``angle_min`` / ``angle_increment``. The
        round-trip can shift the integer bin by 1 for rays within one ULP of a
        boundary; the tolerance accepts that without letting data go missing.

        Args:
            expected: Expected per-bin values.
            actual: Actual per-bin values.
            atol: Absolute tolerance for comparisons.
            name: Field name used in assertion messages.

        Returns:
            None.
        """
        expected = np.asarray(expected)
        actual = np.asarray(actual)
        n = len(expected)
        self.assertEqual(len(actual), n, f"{name}: array length mismatch")

        def _find_mismatches(a: Any, b: Any) -> Any:
            mismatches = []
            for i in range(n):
                lo = max(i - 1, 0)
                hi = min(i + 1, n - 1)
                if not np.any(np.isclose(a[i], b[lo : hi + 1], atol=atol)):
                    mismatches.append((i, float(a[i]), [float(x) for x in b[lo : hi + 1]]))
            return mismatches

        forward = _find_mismatches(expected, actual)
        reverse = _find_mismatches(actual, expected)
        if forward or reverse:
            self.fail(
                f"{name}: bin-tolerant comparison failed -- "
                f"{len(forward)} expected->actual and {len(reverse)} actual->expected mismatches. "
                f"First 5 expected->actual (idx, expected, actual_neighbours): {forward[:5]}. "
                f"First 5 actual->expected (idx, actual, expected_neighbours): {reverse[:5]}."
            )

    async def _test_sensor(
        self,
        sensor_type: str,
        aux_output_level: str = "NONE",
        full_scan: bool = False,
        use_system_time: bool = False,
        metadata: list[str] | None = None,
        test_duration_s: float = 1.5,
        **kwargs: Any,
    ) -> None:
        if metadata is None:
            metadata = []
        """Writer-driven LaserScan validation.

        A ``_GmoCollectorWriter`` captures every GMO from the SD pipeline and the
        list-appending ROS callback captures every published message. The writer is
        attached only after the publisher discovers our subscriber so both streams
        start at the same frame. ``SICK_nanoScan3`` is a non-rotational scanner over
        a static scene, so the tail (snapshot, message) pair compared by
        :meth:`_test_message_data` is representative.

        ``accumulate_outputs`` is forced True regardless of the ``full_scan`` kwarg
        because the LaserScan publisher's per-message contract is one complete scan;
        feeding it interleaved partial-scan slices would mismatch the per-frame ray
        set between snapshots[-1] and messages[-1].
        """
        self.assertEqual(sensor_type, "lidar", "Writer-based path only supports lidar")
        self._is_full_scan = True

        await self._create_sensor(sensor_type, aux_output_level=aux_output_level, accumulate_outputs=True, **kwargs)
        await self._create_omnigraph(enable_full_scan=True, use_system_time=use_system_time, metadata=metadata)

        app_utils.play()
        await app_utils.update_app_async()

        # Publisher returns early when subscription_count == 0; wait for handshake
        # then drop any pre-discovery messages so both streams start at the same frame.
        await self.wait_for_publishers_on_topic(self._ros_node, self._ros_topic, count=1, timeout_sec=5.0)
        self._ros_messages.clear()

        # WriterRegistry.get() only runs __new__ on Writer subclasses, so initialize()
        # must be called explicitly to populate __init__ state. Stash on self so
        # tearDown detaches unconditionally and assertions below can bubble up.
        self._gmo_writer = rep.WriterRegistry.get("_GmoCollectorWriter")
        self._gmo_writer.initialize()
        self._gmo_writer.attach([self._render_product_path])

        await self.simulate_until_condition(
            condition_func=lambda: (len(self._gmo_writer.snapshots) >= 1 and len(self._ros_messages) >= 1),
            max_frames=max(int(test_duration_s * 60) + 30, 60),
        )
        # Yield wall-clock to let DDS deliver any in-flight messages from the final
        # frames; the background executor picks them up automatically.
        await asyncio.sleep(0.2)

        self.assertGreater(len(self._gmo_writer.snapshots), 0, "Writer captured no GMO scans")
        self.assertGreater(len(self._ros_messages), 0, "No LaserScan messages received")

        await self._test_message_data(
            snapshots=self._gmo_writer.snapshots,
            messages=self._ros_messages,
            full_scan=full_scan,
            test_duration_s=test_duration_s,
            metadata=metadata,
        )

    async def _test_message_data(
        self,
        snapshots: list[dict],
        messages: list[LaserScan],
        full_scan: bool = False,
        test_duration_s: float = 1.5,
        metadata: list[str] | None = None,
    ) -> None:
        """Compare the final published LaserScan against the final captured GMO.

        Args:
            snapshots: Captured GMO snapshots.
            messages: Received LaserScan messages.
            full_scan: Whether the helper is configured for full scans.
            test_duration_s: Test duration.
            metadata: Selected metadata fields.
        """
        if metadata is None:
            metadata = []
        msg = messages[-1]
        snap = snapshots[-1]

        if snap["elementsCoordsType"] == gmo_utils.CoordsType.CARTESIAN:
            x_data, y_data, z_data = snap["x"], snap["y"], snap["z"]
            azimuth_data = np.degrees(np.arctan2(y_data, x_data))
            distance_data = np.sqrt(x_data**2 + y_data**2 + z_data**2)
        else:
            azimuth_data = snap["x"]
            distance_data = snap["z"]
        intensity_data = (snap["scalar"] * 255.0).astype(np.uint8)

        h_res = np.degrees(msg.angle_increment)
        az_start = np.degrees(msg.angle_min)
        num_output = len(msg.ranges)

        # Reproduce publishFromGMO's float32 binning. The np.where call mirrors the
        # publisher's size_t cast for negative diffs (wraps then clamps to last slot).
        az_start_f32 = np.float32(az_start)
        h_res_f32 = np.float32(h_res)
        azimuth_f32 = np.asarray(azimuth_data, dtype=np.float32)
        diff_f32 = azimuth_f32 - az_start_f32
        bin_indices = (diff_f32 / h_res_f32).astype(np.int64)
        bin_indices = np.where(diff_f32 < 0, num_output - 1, bin_indices)
        bin_indices = np.clip(bin_indices, 0, num_output - 1)

        expected_ranges = np.full(num_output, -1.0, dtype=np.float32)
        expected_intensities = np.zeros(num_output, dtype=np.float32)
        for i in range(snap["numElements"]):
            expected_ranges[bin_indices[i]] = distance_data[i]
            expected_intensities[bin_indices[i]] = float(intensity_data[i])

        ros_ranges = np.array(msg.ranges, dtype=np.float32)
        ros_intensities = np.array(msg.intensities, dtype=np.float32)

        self._assert_match_with_bin_tolerance(expected_ranges, ros_ranges, atol=1e-5, name="ranges")
        self._assert_match_with_bin_tolerance(expected_intensities, ros_intensities, atol=1e-5, name="intensities")

    async def test_rtx_lidar_full_scan_simulation_time(self) -> None:
        """Verify SICK nanoScan3 LaserScan output with simulation time."""
        await self._test_sensor(
            sensor_type="lidar", use_system_time=False, test_duration_s=0.5, config="SICK_nanoScan3"
        )

    async def test_rtx_lidar_full_scan_system_time(self) -> None:
        """Verify SICK nanoScan3 LaserScan output with system time."""
        await self._test_sensor(sensor_type="lidar", use_system_time=True, test_duration_s=0.5, config="SICK_nanoScan3")


class TestROS2RadarPointCloudRTX(TestROS2SensorMsgRTX):
    """Test RTX Radar point cloud publishing via ROS2RtxRadarHelper."""

    _ros_msg_type = PointCloud2
    _helper_type = "point_cloud"

    async def setUp(self) -> None:
        """Enable Motion BVH before creating RTX radar sensors."""
        await super().setUp()
        # Enable Motion BVH required by radar
        carb.settings.get_settings().set("/renderer/raytracingMotion/enabled", True)

    async def _test_message_data(
        self, full_scan: bool = False, test_duration_s: float = 1.5, metadata: list[str] | None = None
    ) -> None:
        self.assertIsNotNone(self._ros_msg_data)

        self.assertGreater(
            self._ros_msg_count,
            0,
            f"No message received for {self._sensor_type} after {test_duration_s} simulated seconds.",
        )

        from sensor_msgs_py.point_cloud2 import read_points

        msg_points = read_points(self._ros_msg_data)

        # Verify basic point cloud data exists
        self.assertGreater(len(msg_points), 0, "Point cloud should have points")

        # Verify radial velocity field if requested
        if "radialVelocityMS" in metadata:
            self.assertIn(
                "radial_velocity_ms",
                msg_points.dtype.names,
                "PointCloud2 should contain radial_velocity_ms field",
            )
            rv_data = msg_points["radial_velocity_ms"]
            # For a static scene, radial velocity should be near zero
            self.assertTrue(
                np.all(np.abs(rv_data) < 1.0),
                f"Radial velocity should be near zero for static scene, max={np.max(np.abs(rv_data)):.4f}",
            )

    async def test_rtx_radar_point_cloud_no_metadata(self) -> None:
        """Test basic radar point cloud publishing without metadata."""
        kwargs = {"omni:sensor:WpmDmat:outputFrameOfReference": "WORLD"}
        await self._test_sensor(
            sensor_type="radar",
            use_system_time=False,
            metadata=[],
            test_duration_s=0.5,
            aux_output_level="NONE",
            **kwargs,
        )

    async def test_rtx_radar_point_cloud_basic_metadata(self) -> None:
        """Test radar point cloud with radial velocity metadata."""
        kwargs = {"omni:sensor:WpmDmat:outputFrameOfReference": "WORLD"}
        await self._test_sensor(
            sensor_type="radar",
            use_system_time=False,
            metadata=["radialVelocityMS"],
            test_duration_s=0.5,
            aux_output_level="BASIC",
            **kwargs,
        )


class TestROS2RtxHelperDoTransform(ROS2TestCase):
    """Verify the lidar/radar helpers pass the correct ``doTransform`` value.

    The debug-draw writer is configured based on the sensor's ``outputFrameOfReference`` attribute.

    The expected mapping is:
      * ``WORLD`` frame  -> ``doTransform=False`` (points are already in world coords)
      * any other frame  -> ``doTransform=True``  (writer must apply sensor->world)
    """

    @staticmethod
    def _find_debug_writer(state: Any, marker: str) -> Any:
        """Return the writer in ``state._writers`` whose node_type_id contains *marker*.

        Compares against ``DebugDrawPointCloud`` (the underlying OG node type) so we
        match both ``RtxLidarDebugDrawPointCloudBuffer`` and ``RtxSensorDebugDrawPointCloud``.

        Args:
            state: Helper internal state containing writers.
            marker: Node type marker to find.

        Returns:
            Matching writer, or None if no writer matches.
        """
        return next(
            (w for w in getattr(state, "_writers", []) if marker in getattr(w, "node_type_id", "")),
            None,
        )

    async def _setup_lidar_helper(self, frame_of_reference: str | None) -> None:
        """Create a Lidar with the given outputFrameOfReference and a helper OG node.

        Args:
            frame_of_reference: Output frame of reference to configure.
        """
        attributes = {}
        if frame_of_reference is not None:
            attributes["omni:sensor:Core:outputFrameOfReference"] = frame_of_reference

        lidar = Lidar.create("/lidar", attributes=attributes or None)
        sensor_path = lidar.paths[0]
        render_product = rep.create.render_product(sensor_path, resolution=(64, 64))

        og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PCLPublish", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PCLPublish.inputs:renderProductPath", render_product.path),
                    ("PCLPublish.inputs:type", "point_cloud"),
                    ("PCLPublish.inputs:showDebugView", True),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PCLPublish.inputs:execIn"),
                ],
            },
        )

        # Drive the helper's compute() once so it sets up its writers.
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

    async def _setup_radar_helper(self, frame_of_reference: str) -> None:
        """Create a Radar with the given outputFrameOfReference and a helper OG node.

        Args:
            frame_of_reference: Output frame of reference to configure.
        """
        from isaacsim.sensors.experimental.rtx import Radar

        # Radar requires Motion BVH.
        carb.settings.get_settings().set("/renderer/raytracingMotion/enabled", True)

        radar = Radar(
            "/radar",
            attributes={"omni:sensor:WpmDmat:outputFrameOfReference": frame_of_reference},
        )
        sensor_path = radar.paths[0]
        render_product = rep.create.render_product(sensor_path, resolution=(64, 64))

        og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PCLPublish", "isaacsim.ros2.bridge.ROS2RtxRadarHelper"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PCLPublish.inputs:renderProductPath", render_product.path),
                    ("PCLPublish.inputs:showDebugView", True),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PCLPublish.inputs:execIn"),
                ],
            },
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

    async def _assert_lidar_debug_dotransform(self, frame_of_reference: str | None, expected: bool) -> None:
        from isaacsim.ros2.nodes.ogn.OgnROS2RtxLidarHelperDatabase import OgnROS2RtxLidarHelperDatabase

        await self._setup_lidar_helper(frame_of_reference)
        node = og.Controller.node("/ActionGraph/PCLPublish")
        state = OgnROS2RtxLidarHelperDatabase.per_instance_internal_state(node)
        debug_writer = self._find_debug_writer(state, "DebugDrawPointCloud")
        self.assertIsNotNone(
            debug_writer,
            f"Expected the lidar helper to register a debug-draw writer for frame={frame_of_reference!r}.",
        )
        self.assertEqual(
            debug_writer._kwargs.get("doTransform"),
            expected,
            f"Expected doTransform={expected} for outputFrameOfReference={frame_of_reference!r}",
        )

    async def _assert_radar_debug_dotransform(self, frame_of_reference: str, expected: bool) -> None:
        from isaacsim.ros2.nodes.ogn.OgnROS2RtxRadarHelperDatabase import OgnROS2RtxRadarHelperDatabase

        await self._setup_radar_helper(frame_of_reference)
        node = og.Controller.node("/ActionGraph/PCLPublish")
        state = OgnROS2RtxRadarHelperDatabase.per_instance_internal_state(node)
        debug_writer = self._find_debug_writer(state, "DebugDrawPointCloud")
        self.assertIsNotNone(
            debug_writer,
            f"Expected the radar helper to register a debug-draw writer for frame={frame_of_reference!r}.",
        )
        self.assertEqual(
            debug_writer._kwargs.get("doTransform"),
            expected,
            f"Expected doTransform={expected} for outputFrameOfReference={frame_of_reference!r}",
        )

    async def test_lidar_world_frame_disables_transform(self) -> None:
        """Verify lidar debug draw leaves WORLD-frame points untransformed."""
        await self._assert_lidar_debug_dotransform("WORLD", expected=False)

    async def test_lidar_sensor_frame_enables_transform(self) -> None:
        """Verify lidar debug draw transforms SENSOR-frame points into world space."""
        await self._assert_lidar_debug_dotransform("SENSOR", expected=True)

    async def test_lidar_unauthored_frame_enables_transform(self) -> None:
        # When the attribute is not authored, GetAttribute().Get() returns None,
        # which is `!= "WORLD"` -> doTransform must default to True (matches the
        # writer's registered default and avoids points landing at the origin).
        """Verify lidar debug draw transforms points when no frame attribute is authored."""
        await self._assert_lidar_debug_dotransform(None, expected=True)

    async def test_radar_world_frame_disables_transform(self) -> None:
        """Verify radar debug draw leaves WORLD-frame points untransformed."""
        await self._assert_radar_debug_dotransform("WORLD", expected=False)

    async def test_radar_sensor_frame_enables_transform(self) -> None:
        """Verify radar debug draw transforms SENSOR-frame points into world space."""
        await self._assert_radar_debug_dotransform("SENSOR", expected=True)


class TestROS2PCLMetadataGating(TestROS2SensorMsgRTX):
    """Verify ``ROS2PublishPointCloud`` optional field gating.

    Optional fields are included only when both the pointer input *and* the matching ``output*`` flag are provided.

    Coverage:
      * Helper-driven path: when ``selectedMetadata`` is empty, the helper does not
        set ``output*`` flags on the publisher even though the upstream
        ``IsaacExtractRTXSensorPointCloud`` annotator wires the ``intensityPtr``
        and ``timestampPtr`` outputs unconditionally. The published PointCloud2
        must therefore contain only ``x``/``y``/``z``.
      * When ``selectedMetadata`` is non-empty, the helper sets the flags and the
        corresponding fields appear in the message.
    """

    _ros_msg_type = PointCloud2
    _helper_type = "point_cloud"

    async def _test_message_data(
        self, full_scan: bool = False, test_duration_s: float = 1.5, metadata: list[str] | None = None
    ) -> None:
        if metadata is None:
            metadata = []
        self.assertIsNotNone(self._ros_msg_data)
        self.assertGreater(self._ros_msg_count, 0, "No PointCloud2 messages received")

        # Build the set of expected ROS field names from the requested metadata.
        # Only the fields driven by this test are listed; xyz is always present.
        metadata_to_fields = {
            "intensity": {"intensity"},
            "timestamp": {"timestamp_0", "timestamp_1"},
            "emitterId": {"emitter_id"},
            "channelId": {"channel_id"},
            "materialId": {"material_id"},
            "tickId": {"tick_id"},
            "echoId": {"echo_id"},
            "tickState": {"tick_state"},
        }
        expected_optional_fields = set()
        for item in metadata:
            expected_optional_fields.update(metadata_to_fields.get(item, set()))

        msg_field_names = {f.name for f in self._ros_msg_data.fields}
        # xyz is always expected.
        self.assertTrue({"x", "y", "z"}.issubset(msg_field_names), f"Missing xyz in fields {msg_field_names}")

        # Every metadata field that was *not* selected must be absent from the message,
        # even though the upstream annotator wires the matching pointer input.
        all_optional_fields = set().union(*metadata_to_fields.values())
        unexpected = (all_optional_fields - expected_optional_fields) & msg_field_names
        self.assertFalse(
            unexpected,
            f"Fields {unexpected} appeared in the PointCloud2 even though they were not in metadata={metadata}",
        )

        # Every selected field must be present.
        missing = expected_optional_fields - msg_field_names
        self.assertFalse(
            missing,
            f"Fields {missing} were selected (metadata={metadata}) but missing from the PointCloud2 ({msg_field_names})",
        )

    async def test_intensity_excluded_when_flag_not_set(self) -> None:
        """metadata=[] means outputIntensity stays False, so the message has no intensity."""
        await self._test_sensor(
            sensor_type="lidar",
            full_scan=True,
            aux_output_level="NONE",
            use_system_time=False,
            metadata=[],
            test_duration_s=0.5,
        )

    async def test_intensity_included_when_flag_set(self) -> None:
        """When intensity is requested, it must appear in the published PointCloud2."""
        await self._test_sensor(
            sensor_type="lidar",
            full_scan=True,
            aux_output_level="BASIC",
            use_system_time=False,
            metadata=["intensity"],
            test_duration_s=0.5,
        )
