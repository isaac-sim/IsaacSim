# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""High level wrapper for creating/configuring position-based-dynamics (PBD) particle materials for simulating fluids, cloth and inflatables."""

from __future__ import annotations

import carb

# isaac-core
import isaacsim.core.utils.stage as stage_utils

# omniverse
from isaacsim.core.api.materials.particle_material_view import ParticleMaterialView
from isaacsim.core.api.simulation_context.simulation_context import SimulationContext
from pxr import Usd, UsdShade


class ParticleMaterial:
    """A wrapper around position-based-dynamics (PBD) material for particles used to simulate fluids, cloth and inflatables.

    Applies the `PhysxSchema.PhysxPBDMaterialAPI` to a material prim.

    Note:
        Currently, only a single material per particle system is supported which applies
        to all objects that are associated with the system.
        If a prim does not exist at specified path, then a new UsdShade.Material prim is created.

    Args:
        prim_path: The prim path to create/apply PBD material properties.
        name: Name given to the prim when instantiating it.
        friction: The friction coefficient.
        particle_friction_scale: The coefficient that scales friction for solid particle-particle interactions.
        damping: The global velocity damping coefficient.
        viscosity: The viscosity of fluid particles.
        vorticity_confinement: The vorticity confinement for fluid particles.
        surface_tension: The surface tension.
        cohesion: The cohesion for interaction between fluid particles.
        adhesion: The adhesion for interaction between particles (solid or fluid), and rigid or deformable objects.
        particle_adhesion_scale: The coefficient that scales adhesion for solid particle-particle iterations.
        adhesion_offset_scale: The offset scale defines at which adhesion ceases to take effect.
        gravity_scale: The gravitational acceleration scaling factor. It can be used to approximate
            lighter-than-air inflatables.
        lift: The lift coefficient for cloth and inflatable particle objects.
        drag: The drag coefficient for cloth and inflatable particle objects.

    """

    def __init__(
        self,
        prim_path: str,
        name: str | None = "particle_material",
        friction: float | None = None,
        particle_friction_scale: float | None = None,
        damping: float | None = None,
        viscosity: float | None = None,
        vorticity_confinement: float | None = None,
        surface_tension: float | None = None,
        cohesion: float | None = None,
        adhesion: float | None = None,
        particle_adhesion_scale: float | None = None,
        adhesion_offset_scale: float | None = None,
        gravity_scale: float | None = None,
        lift: float | None = None,
        drag: float | None = None,
    ) -> None:
        stage = stage_utils.get_current_stage()
        self._name = name
        self._prim_path = prim_path
        self._prim = stage.GetPrimAtPath(prim_path)

        if SimulationContext.instance() is not None:
            self._backend = SimulationContext.instance().backend
            self._device = SimulationContext.instance().device
            self._backend_utils = SimulationContext.instance().backend_utils
        else:
            import isaacsim.core.utils.numpy as np_utils

            self._backend = "numpy"
            self._device = "cpu"
            self._backend_utils = np_utils

        if stage.GetPrimAtPath(prim_path).IsValid():
            if not self._prim.IsA(UsdShade.Material):
                raise ValueError(f"A prim at path '{prim_path}' exists but is not a Usd.Material prim.")
            else:
                carb.log_warn(f"A material prim already defined at path: {prim_path}.")
                self._material = UsdShade.Material(stage.GetPrimAtPath(prim_path))
        else:
            self._material = UsdShade.Material.Define(stage, prim_path)

        # set properties
        if friction is not None:
            friction = self._backend_utils.create_tensor_from_list([friction], dtype="float32", device=self._device)
        if particle_friction_scale is not None:
            particle_friction_scale = self._backend_utils.create_tensor_from_list(
                [particle_friction_scale], dtype="float32", device=self._device
            )
        if damping is not None:
            damping = self._backend_utils.create_tensor_from_list([damping], dtype="float32", device=self._device)
        if viscosity is not None:
            viscosity = self._backend_utils.create_tensor_from_list([viscosity], dtype="float32", device=self._device)
        if vorticity_confinement is not None:
            vorticity_confinement = self._backend_utils.create_tensor_from_list(
                [vorticity_confinement], dtype="float32", device=self._device
            )
        if surface_tension is not None:
            surface_tension = self._backend_utils.create_tensor_from_list(
                [surface_tension], dtype="float32", device=self._device
            )
        if cohesion is not None:
            cohesion = self._backend_utils.create_tensor_from_list([cohesion], dtype="float32", device=self._device)
        if adhesion is not None:
            adhesion = self._backend_utils.create_tensor_from_list([adhesion], dtype="float32", device=self._device)
        if particle_adhesion_scale is not None:
            particle_adhesion_scale = self._backend_utils.create_tensor_from_list(
                [particle_adhesion_scale], dtype="float32", device=self._device
            )
        if adhesion_offset_scale is not None:
            adhesion_offset_scale = self._backend_utils.create_tensor_from_list(
                [adhesion_offset_scale], dtype="float32", device=self._device
            )
        if gravity_scale is not None:
            gravity_scale = self._backend_utils.create_tensor_from_list(
                [gravity_scale], dtype="float32", device=self._device
            )
        if lift is not None:
            carb.log_warn(
                "ParticleMaterial: 'lift' parameter is ignored — physxPBDMaterial:lift was deprecated by PhysX."
            )
        if drag is not None:
            carb.log_warn(
                "ParticleMaterial: 'drag' parameter is ignored — physxPBDMaterial:drag was deprecated by PhysX."
            )

        self._particle_material_view = ParticleMaterialView(
            prim_paths_expr=prim_path,
            name=name,
            frictions=friction,
            particle_friction_scales=particle_friction_scale,
            dampings=damping,
            viscosities=viscosity,
            vorticity_confinements=vorticity_confinement,
            surface_tensions=surface_tension,
            cohesions=cohesion,
            adhesions=adhesion,
            particle_adhesion_scales=particle_adhesion_scale,
            adhesion_offset_scales=adhesion_offset_scale,
            gravity_scales=gravity_scale,
        )

    """
    Properties.
    """

    @property
    def prim_path(self) -> str:
        """Stage path to the material.

        Returns:
            The stage path to the material.

        """
        return self._prim_path

    @property
    def prim(self) -> Usd.Prim:
        """USD prim present.

        Returns:
            The USD prim present.

        """
        return self._prim

    @property
    def material(self) -> UsdShade.Material:
        """USD Material object.

        Returns:
            The USD Material object.

        """
        return self._material

    @property
    def name(self) -> str | None:
        """Name given to the prim when instantiating it.

        Returns:
            Name given to the prim when instantiating it. Otherwise None.

        """
        return self._name

    def initialize(self, physics_sim_view: object = None) -> None:
        """Initializes the particle material.

        Args:
            physics_sim_view: Physics simulation view to use for initialization.

        """
        self._particle_material_view.initialize(physics_sim_view=physics_sim_view)
        return

    def is_valid(self) -> bool:
        """Whether the current prim path corresponds to a valid prim in stage.

        Returns:
            True is the current prim path corresponds to a valid prim in stage. False otherwise.

        """
        return self._particle_material_view.is_valid()

    def post_reset(self) -> None:
        """Resets the prim to its default state."""
        self._particle_material_view.post_reset()
        return

    """
    Operations - Setters.
    """

    def set_friction(self, value: float) -> None:
        """Sets the friction coefficient.

        The friction takes effect in all interactions between particles and rigids or deformables.
        For solid particle-particle interactions it is multiplied by the particle friction scale.

        Args:
            value: The friction coefficient.
                Range: [0, inf), Units: dimensionless

        """
        if value < 0:
            carb.log_error("The valid range of friction coefficient is [0. inf).")
        self._particle_material_view.set_frictions(
            self._backend_utils.create_tensor_from_list([value], dtype="float32")
        )

    def set_particle_friction_scale(self, value: float) -> None:
        """Sets the particle friction scale.

        The coefficient that scales friction for solid particle-particle interaction.

        Args:
            value: The particle friction scale.
                Range: [0, inf), Units: dimensionless

        """
        if value < 0:
            carb.log_error("The valid range of particle friction scale is [0. inf).")
        self._particle_material_view.set_particle_friction_scales(
            self._backend_utils.create_tensor_from_list([value], dtype="float32")
        )

    def set_damping(self, value: float) -> None:
        """Sets the global velocity damping coefficient.

        Args:
            value: The damping coefficient.
                Range: [0, inf), Units: dimensionless

        """
        if value < 0:
            carb.log_error("The valid range of damping coefficient is [0. inf).")
        self._particle_material_view.set_dampings(self._backend_utils.create_tensor_from_list([value], dtype="float32"))

    def set_viscosity(self, value: float) -> None:
        """Sets the viscosity for fluid particles.

        Args:
            value: The viscosity.
                Range: [0, inf), Units: dimensionless

        """
        if value < 0:
            carb.log_error("The valid range of viscosity is [0. inf).")
        self._particle_material_view.set_viscosities(
            self._backend_utils.create_tensor_from_list([value], dtype="float32")
        )

    def set_vorticity_confinement(self, value: float) -> None:
        """Sets the vorticity confinement for fluid particles.

        This helps prevent energy loss due to numerical solver by adding vortex-like
        accelerations to the particles.

        Args:
            value: The vorticity confinement.
                Range: [0, inf), Units: dimensionless

        """
        if value < 0:
            carb.log_error("The valid range of vorticity confinement is [0. inf).")
        self._particle_material_view.set_vorticity_confinements(
            self._backend_utils.create_tensor_from_list([value], dtype="float32")
        )

    def set_surface_tension(self, value: float) -> None:
        """Sets the surface tension for fluid particles.

        Args:
            value: The surface tension.
                Range: [0, inf), Units: 1 / (distance * distance * distance)

        """
        if value < 0:
            carb.log_error("The valid range of damping coefficient is [0. inf).")
        self._particle_material_view.set_surface_tensions(
            self._backend_utils.create_tensor_from_list([value], dtype="float32")
        )

    def set_cohesion(self, value: float) -> None:
        """Sets the cohesion for interaction between fluid particles.

        Args:
            value: The cohesion.
                Range: [0, inf), Units: dimensionless

        """
        if value < 0:
            carb.log_error("The valid range of cohesion is [0. inf).")
        self._particle_material_view.set_cohesions(
            self._backend_utils.create_tensor_from_list([value], dtype="float32")
        )

    def set_adhesion(self, value: float) -> None:
        """Sets the adhesion for interaction between particles (solid or fluid), and rigid or deformable objects.

        Note:
            Adhesion also applies to solid-solid particle interactions, but is multiplied with the
            particle adhesion scale.

        Args:
            value: The adhesion.
                Range: [0, inf), Units: dimensionless

        """
        if value < 0:
            carb.log_error("The valid range of adhesion is [0. inf).")
        self._particle_material_view.set_adhesions(
            self._backend_utils.create_tensor_from_list([value], dtype="float32")
        )

    def set_particle_adhesion_scale(self, value: float) -> None:
        """Sets the particle adhesion scale.

        This coefficient scales the adhesion for solid particle-particle interaction.

        Args:
            value: The adhesion scale.
                Range: [0, inf), Units: dimensionless

        """
        if value < 0:
            carb.log_error("The valid range of particle adhesion scale is [0. inf).")
        self._particle_material_view.set_particle_adhesion_scales(
            self._backend_utils.create_tensor_from_list([value], dtype="float32")
        )

    def set_adhesion_offset_scale(self, value: float) -> None:
        """Sets the adhesion offset scale.

        It defines the offset at which adhesion ceases to take effect. For interactions between
        particles (fluid or solid), and rigids or deformables, the adhesion offset is defined
        relative to the rest offset. For solid particle-particle interactions, the adhesion
        offset is defined relative to the solid rest offset.

        Args:
            value: The adhesion offset scale.
                Range: [0, inf), Units: dimensionless

        """
        if value < 0:
            carb.log_error("The valid range of adhesion offset scale is [0. inf).")
        self._particle_material_view.set_adhesion_offset_scales(
            self._backend_utils.create_tensor_from_list([value], dtype="float32")
        )

    def set_gravity_scale(self, value: float) -> None:
        """Sets the gravitational acceleration scaling factor.

        It can be used to approximate lighter-than-air inflatable.
        For example (-1.0 would invert gravity).

        Args:
            value: The gravity scale.
                Range: (-inf , inf), Units: dimensionless

        """
        self._particle_material_view.set_gravity_scales(
            self._backend_utils.create_tensor_from_list([value], dtype="float32")
        )

    def set_lift(self, value: float) -> None:
        """Sets the lift coefficient, i.e. basic aerodynamic lift model coefficient.

        .. deprecated::
            physxPBDMaterial:lift was deprecated by PhysX. This method is a no-op.

        Args:
            value: The lift coefficient (ignored).

        """
        carb.log_warn("ParticleMaterial.set_lift is a no-op — physxPBDMaterial:lift was removed by PhysX.")

    def set_drag(self, value: float) -> None:
        """Sets the drag coefficient, i.e. basic aerodynamic drag model coefficient.

        .. deprecated::
            physxPBDMaterial:drag was deprecated by PhysX. This method is a no-op.

        Args:
            value: The drag coefficient (ignored).

        """
        carb.log_warn("ParticleMaterial.set_drag is a no-op — physxPBDMaterial:drag was removed by PhysX.")

    """
    Operations - Getters.
    """

    def get_friction(self) -> float:
        """Friction coefficient.

        Returns:
            The friction coefficient.

        """
        return self._particle_material_view.get_frictions()[0]

    def get_particle_friction_scale(self) -> float:
        """Particle friction scale.

        Returns:
            The particle friction scale.

        """
        return self._particle_material_view.get_particle_friction_scales()[0]

    def get_damping(self) -> float:
        """Global velocity damping coefficient.

        Returns:
            The global velocity damping coefficient.

        """
        return self._particle_material_view.get_dampings()[0]

    def get_viscosity(self) -> float:
        """Viscosity for fluid particles.

        Returns:
            The viscosity.

        """
        return self._particle_material_view.get_viscosities()[0]

    def get_vorticity_confinement(self) -> float:
        """Vorticity confinement for fluid particles.

        Returns:
            The vorticity confinement for fluid particles.

        """
        return self._particle_material_view.get_vorticity_confinements()[0]

    def get_surface_tension(self) -> float:
        """Surface tension for fluid particles.

        Returns:
            The surface tension for fluid particles.

        """
        return self._particle_material_view.get_surface_tensions()[0]

    def get_cohesion(self) -> float:
        """Cohesion for interaction between fluid particles.

        Returns:
            The cohesion for interaction between fluid particles.

        """
        return self._particle_material_view.get_cohesions()[0]

    def get_adhesion(self) -> float:
        """Adhesion for interaction between particles and rigid or deformable objects.

        Returns:
            The adhesion for interaction between particles (solid or fluid), and rigids or deformables.

        """
        return self._particle_material_view.get_adhesions()[0]

    def get_particle_adhesion_scale(self) -> float:
        """Particle adhesion scale.

        Returns:
            The particle adhesion scale.

        """
        return self._particle_material_view.get_particle_adhesion_scales()[0]

    def get_adhesion_offset_scale(self) -> float:
        """Adhesion offset scale.

        Returns:
            The adhesion offset scale.

        """
        return self._particle_material_view.get_adhesion_offset_scales()[0]

    def get_gravity_scale(self) -> float:
        """Gravitational acceleration scaling factor.

        Returns:
            The gravitational acceleration scaling factor.

        """
        return self._particle_material_view.get_gravity_scales()[0]

    def get_lift(self) -> float:
        """Lift coefficient for the basic aerodynamic lift model.

        .. deprecated::
            physxPBDMaterial:lift was deprecated by PhysX. Always returns 0.0.

        Returns:
            Always 0.0 since the lift attribute was removed by PhysX.

        """
        return 0.0

    def get_drag(self) -> float:
        """Drag coefficient for the basic aerodynamic drag model.

        .. deprecated::
            physxPBDMaterial:drag was deprecated by PhysX. Always returns 0.0.

        Returns:
            Always 0.0 since the drag attribute was removed by PhysX.

        """
        return 0.0
