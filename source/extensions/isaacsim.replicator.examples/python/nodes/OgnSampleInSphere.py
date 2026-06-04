# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import numpy as np
import omni.graph.core as og
import omni.usd
from pxr import Sdf, UsdGeom


class OgnSampleInSphere:
    @staticmethod
    def compute(db) -> bool:
        prim_paths = db.inputs.prims
        if len(prim_paths) == 0:
            db.outputs.execOut = og.ExecutionAttributeState.DISABLED
            return False

        stage = omni.usd.get_context().get_stage()
        prims = [stage.GetPrimAtPath(str(path)) for path in prim_paths]

        radius = db.inputs.radius

        try:
            for prim in prims:
                if not UsdGeom.Xformable(prim):
                    prim_type = prim.GetTypeName()
                    raise ValueError(
                        f"Expected prim at {prim.GetPath()} to be an Xformable prim but got type {prim_type}"
                    )
                if not prim.HasAttribute("xformOp:translate"):
                    UsdGeom.Xformable(prim).AddTranslateOp()
            if radius <= 0:
                raise ValueError(f"Radius must be positive, got {radius}")

        except Exception as error:
            db.log_error(str(error))
            db.outputs.execOut = og.ExecutionAttributeState.DISABLED
            return False

        samples = []
        for _ in range(len(prims)):
            # Generate a random direction by spherical coordinates (phi, theta)
            phi = np.random.uniform(0, 2 * np.pi)
            # Sample costheta to ensure uniform distribution of points on the sphere (surface is proportional to sin(theta))
            costheta = np.random.uniform(-1, 1)
            theta = np.arccos(costheta)

            # Scale the radius uniformly within the sphere, applying the cube root to a random value
            # to account for volume's cubic growth with radius (r^3), ensuring spatial uniformity.
            r = radius * (np.random.random() ** (1 / 3))

            # Convert from spherical to Cartesian coordinates
            x = r * np.sin(theta) * np.cos(phi)
            y = r * np.sin(theta) * np.sin(phi)
            z = r * np.cos(theta)

            samples.append((x, y, z))

        with Sdf.ChangeBlock():
            for prim, sample in zip(prims, samples):
                prim.GetAttribute("xformOp:translate").Set(sample)

        db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        return True
