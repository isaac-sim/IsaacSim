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

"""Rule for converting a PhysX asset to MuJoCo/Newton schemas."""

from __future__ import annotations

import os

from isaacsim.asset.importer.utils.impl import physx_asset_to_mjc
from isaacsim.asset.transformer import RuleConfigurationParam, RuleInterface
from pxr import Usd

from .. import utils


class PhysxToMjcConversionRule(RuleInterface):
    """Convert PhysX joints and articulation schemas to MuJoCo/Newton equivalents.

    Revolute/prismatic joints get ``MjcJointAPI`` + ``MjcActuator``, mimic joints
    become ``NewtonMimicAPI``, and articulation roots gain
    ``NewtonArticulationRootAPI``. Multi-DOF D6/spherical joints are warned about
    and left untouched (converting them would change the articulation structure).

    The conversion edits the working stage in place. If the rule's ``destination``
    is a USD file, the results are instead authored into that overlay layer (which
    sublayers the working stage), leaving the source untouched.
    """

    def get_configuration_parameters(self) -> list[RuleConfigurationParam]:
        """Return the configuration parameters for this rule.

        Returns:
            Empty list; the rule uses the standard input stage, output directory,
            and destination provided by the transformer framework.

        Example:

        .. code-block:: python

            params = rule.get_configuration_parameters()

        """
        return []

    def _log_actuator_readiness(self, stage: Usd.Stage, existing_paths: set[str]) -> None:
        """Distinguish newly generated actuators without gains from existing motors.

        Args:
            stage: Converted stage containing MuJoCo actuators.
            existing_paths: Actuator paths present before this conversion.
        """
        actuators = [prim for prim in stage.Traverse() if prim.GetTypeName() == "MjcActuator"]
        generated = [prim for prim in actuators if str(prim.GetPath()) not in existing_paths]
        missing = [
            prim
            for prim in generated
            if not prim.GetAttribute("mjc:gainPrm").HasAuthoredValue()
            or not prim.GetAttribute("mjc:biasPrm").HasAuthoredValue()
        ]
        self.log_operation(
            f"MuJoCo actuators generated: {len(generated)}; new actuators without authored gain/bias: {len(missing)}"
        )
        if missing:
            examples = ", ".join(str(prim.GetPath()) for prim in missing[:3])
            self.log_operation(
                f"Control readiness warning: {len(missing)} new actuator(s) lack authored gain/bias "
                f"parameters (for example, {examples}). The PhysX source drives supplied no convertible gains. "
                "Author mjc:gainPrm and mjc:biasPrm for the intended controller and validate commanded motion. "
                "Structural conversion completed."
            )

    def process_rule(self) -> str | None:
        """Convert PhysX joints and articulation schemas to MuJoCo/Newton.

        Returns:
            The output layer path when authoring into a separate overlay
            (``destination`` is a USD file), otherwise ``None`` (working stage
            edited in place; the manager persists it).

        Example:

        .. code-block:: python

            rule.process_rule()

        """
        self.log_operation("PhysxToMjcConversionRule start")

        source_stage = self.source_stage
        destination_is_layer = bool(self.destination_path) and (
            os.path.splitext(self.destination_path)[1].lower() in utils.USD_EXTENSIONS
        )

        existing_paths = {
            str(prim.GetPath()) for prim in source_stage.Traverse() if prim.GetTypeName() == "MjcActuator"
        }

        if not destination_is_layer:
            # Edit the working stage in place; the manager saves it after all rules.
            physx_asset_to_mjc.convert_physx_asset_to_mjc(source_stage)
            self._log_actuator_readiness(source_stage, existing_paths)
            self.log_operation("PhysxToMjcConversionRule completed")
            return None

        # Author the conversion into the destination overlay layer.
        destination_path = os.path.join(self.package_root, self.destination_path)
        output_layer = utils.find_or_create_layer(destination_path, source_stage)
        if not output_layer:
            self.log_operation(f"Failed to open output layer: {destination_path}")
            return None

        source_id = source_stage.GetRootLayer().identifier
        if source_id not in output_layer.subLayerPaths:
            output_layer.subLayerPaths.append(source_id)

        overlay_stage = Usd.Stage.Open(output_layer)
        overlay_stage.SetEditTarget(overlay_stage.GetEditTargetForLocalLayer(output_layer))
        source_default = source_stage.GetDefaultPrim()
        if source_default:
            overlay_stage.SetDefaultPrim(overlay_stage.GetPrimAtPath(source_default.GetPath()))

        physx_asset_to_mjc.convert_physx_asset_to_mjc(source_stage, delta_stage=overlay_stage)
        self._log_actuator_readiness(overlay_stage, existing_paths)
        output_layer.Save()
        self.add_affected_stage(self.destination_path)

        self.log_operation("PhysxToMjcConversionRule completed")
        return destination_path
