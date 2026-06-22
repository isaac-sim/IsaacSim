# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0

"""Self-contained tests for teleop coordinate conversion helpers."""

from __future__ import annotations

import numpy as np
import omni.kit.test
from isaacsim.replicator.teleop import (
    OXR_TO_ISS_QUAT,
    OXR_TO_ISS_ROTATION,
    CoordinateSystem,
    transform_pose,
    transform_pose_openxr_to_isaacsim,
)


class TestCoordinateUtils(omni.kit.test.AsyncTestCase):
    """Test TestCoordinateUtils behavior."""

    async def test_openxr_to_isaacsim_position_and_identity_orientation(self) -> None:
        """Run the openxr to isaacsim position and identity orientation test."""
        position, orientation = transform_pose_openxr_to_isaacsim(
            (1.0, 2.0, 3.0),
            (0.0, 0.0, 0.0, 1.0),
        )

        self.assertEqual(position, (-3.0, -1.0, 2.0))
        self.assertEqual(orientation, OXR_TO_ISS_QUAT)

    async def test_openxr_to_isaacsim_rotation_matrix_matches_axis_mapping(self) -> None:
        """Run the openxr to isaacsim rotation matrix matches axis mapping test."""
        openxr_axes = np.eye(3)
        converted_axes = (OXR_TO_ISS_ROTATION @ openxr_axes.T).T

        np.testing.assert_allclose(converted_axes[0], np.array([0.0, -1.0, 0.0]))
        np.testing.assert_allclose(converted_axes[1], np.array([0.0, 0.0, 1.0]))
        np.testing.assert_allclose(converted_axes[2], np.array([-1.0, 0.0, 0.0]))

    async def test_raw_transform_passthrough(self) -> None:
        """Run the raw transform passthrough test."""
        position = (0.25, -0.5, 1.5)
        orientation = (0.1, 0.2, 0.3, 0.4)

        transformed_position, transformed_orientation = transform_pose(position, orientation, CoordinateSystem.RAW)

        self.assertEqual(transformed_position, position)
        self.assertEqual(transformed_orientation, orientation)
