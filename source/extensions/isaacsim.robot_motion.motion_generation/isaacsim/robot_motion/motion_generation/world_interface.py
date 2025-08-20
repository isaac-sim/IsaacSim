# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from typing import List, Optional, Union

import carb
import isaacsim.core.api.objects
from isaacsim.core.api.objects import capsule, cone, cuboid, cylinder, ground_plane, sphere


class WorldInterface:
    """Interface for translating USD world to a generic world-aware algorithm such as a MotionPolicy"""

    def __init__(self) -> None:
        pass

    def update_world(self, updated_obstacles: Optional[List] = None) -> None:
        """Applies all necessary updates to the internal world representation.

        Args:
            updated_obstacles (list, optional): If provided, only the given obstacles will have their poses updated.
                For motion policies that use obstacle poses relative to the robot base (e.g. Lula based policies),
                this list will be ignored if the robot base has moved because all object poses will have changed
                relative to the robot. Defaults to None.

        Returns:
            None
        """
        pass

    def add_obstacle(self, obstacle: isaacsim.core.api.objects, static: Optional[bool] = False) -> bool:
        """Add an obstacle

        Args:
            obstacle (isaacsim.core.api.objects): An obstacle from the package isaacsim.core.api.obstacles
                            The type of the obstacle will be checked, and the appropriate add function will be called \n
            static (Optional[bool]): When True, the obstacle will be assumed to remain stationary relative to the USD global frame over time

        Returns:
            success (bool): Returns True if the obstacle type is valid and the appropriate add function has been implemented
        """

        if (
            isinstance(obstacle, cuboid.DynamicCuboid)
            or isinstance(obstacle, cuboid.VisualCuboid)
            or isinstance(obstacle, cuboid.FixedCuboid)
        ):
            return self.add_cuboid(obstacle, static=static)

        elif isinstance(obstacle, cylinder.DynamicCylinder) or isinstance(obstacle, cylinder.VisualCylinder):
            return self.add_cylinder(obstacle, static=static)

        elif isinstance(obstacle, sphere.DynamicSphere) or isinstance(obstacle, sphere.VisualSphere):
            return self.add_sphere(obstacle, static=static)

        elif isinstance(obstacle, capsule.DynamicCapsule) or isinstance(obstacle, capsule.VisualCapsule):
            return self.add_capsule(obstacle, static=static)

        elif isinstance(obstacle, cone.DynamicCone) or isinstance(obstacle, cone.VisualCone):
            return self.add_cone(obstacle, static=static)

        elif isinstance(obstacle, ground_plane.GroundPlane):
            return self.add_ground_plane(obstacle)

        else:
            carb.log_warn(
                "Obstacle added with unsuported type: "
                + str(type(obstacle))
                + "\nObstacle should be from the package isaacsim.core.api.objects"
            )
            return False

    def add_cuboid(
        self, cuboid: Union[cuboid.DynamicCuboid, cuboid.FixedCuboid, cuboid.VisualCuboid], static: bool = False
    ) -> bool:
        """Add a block obstacle.

        Args:
            cuboid (core.objects.cuboid): Wrapper object for handling rectangular prism Usd Prims.
            static (bool, optional): If True, indicate that cuboid will never change pose, and may be ignored in internal
                world updates. Defaults to False.

        Returns:
            bool: Return True if underlying WorldInterface has implemented add_cuboid()
        """
        carb.log_warn("Function add_cuboid() has not been implemented for this WorldInterface")
        return False

    def add_sphere(self, sphere: Union[sphere.DynamicSphere, sphere.VisualSphere], static: bool = False) -> bool:
        """Add a sphere obstacle.

        Args:
            sphere (core.objects.sphere): Wrapper object for handling sphere Usd Prims.
            static (bool, optional): If True, indicate that sphere will never change pose, and may be ignored in internal
                world updates. Defaults to False.

        Returns:
            bool: Return True if underlying WorldInterface has implemented add_sphere()
        """
        carb.log_warn("Function add_sphere() has not been implemented for this WorldInterface")
        return False

    def add_capsule(self, capsule: Union[capsule.DynamicCapsule, capsule.VisualCapsule], static: bool = False) -> bool:
        """Add a capsule obstacle.

        Args:
            capsule (core.objects.capsule): Wrapper object for handling capsule Usd Prims.
            static (bool, optional): If True, indicate that capsule will never change pose, and may be ignored in internal
                world updates. Defaults to False.

        Returns:
            bool: Return True if underlying WorldInterface has implemented add_capsule()
        """
        carb.log_warn("Function add_capsule() has not been implemented for this WorldInterface")
        return False

    def add_cylinder(
        self, cylinder: Union[cylinder.DynamicCylinder, cylinder.VisualCylinder], static: bool = False
    ) -> bool:
        """Add a cylinder obstacle.

        Args:
            cylinder (core.objects.cylinder): Wrapper object for handling rectangular prism Usd Prims.
            static (bool, optional): If True, indicate that cuboid will never change pose, and may be ignored in internal
                world updates. Defaults to False.

        Returns:
            bool: Return True if underlying WorldInterface has implemented add_cylinder()
        """
        carb.log_warn("Function add_cylinder() has not been implemented for this WorldInterface")
        return False

    def add_cone(self, cone: Union[cone.DynamicCone, cone.VisualCone], static: bool = False) -> bool:
        """Add a cone obstacle.

        Args:
            cone (core.objects.cone): Wrapper object for handling cone Usd Prims.
            static (bool, optional): If True, indicate that cone will never change pose, and may be ignored in internal
                world updates. Defaults to False.

        Returns:
            bool: Return True if underlying WorldInterface has implemented add_cone()
        """
        carb.log_warn("Function add_cone() has not been implemented for this WorldInterface")
        return False

    def add_ground_plane(self, ground_plane: ground_plane.GroundPlane) -> bool:
        """Add a ground_plane

        Args:
            ground_plane (core.objects.ground_plane.GroundPlane): Wrapper object for handling ground_plane Usd Prims.

        Returns:
            bool: Return True if underlying WorldInterface has implemented add_ground_plane()
        """
        carb.log_warn("Function add_ground_plane() has not been implemented for this WorldInterface")
        return False

    def disable_obstacle(self, obstacle: isaacsim.core.api.objects) -> bool:
        """Disable collision avoidance for obstacle.

        Args:
            obstacle (core.object): obstacle to be disabled.

        Returns:
            bool: Return True if obstacle was identified and successfully disabled.
        """
        carb.log_warn("Function disable_obstacle() has not been implemented for this WorldInterface")
        return False

    def enable_obstacle(self, obstacle: isaacsim.core.api.objects) -> bool:
        """Enable collision avoidance for obstacle.

        Args:
            obstacle (core.object): obstacle to be enabled.

        Returns:
            bool: Return True if obstacle was identified and successfully enabled.
        """
        carb.log_warn("Function enable_obstacle() has not been implemented for this WorldInterface")
        return False

    def remove_obstacle(self, obstacle: isaacsim.core.api.objects) -> bool:
        """Remove obstacle from collision avoidance. Obstacle cannot be re-enabled via enable_obstacle() after
        removal.

        Args:
            obstacle (core.object): obstacle to be removed.

        Returns:
            bool: Return True if obstacle was identified and successfully removed.
        """
        carb.log_warn("Function remove_obstacle() has not been implemented for this WorldInterface")
        return False

    def reset(self) -> None:
        """Reset all state inside the WorldInterface to its initial values"""
        pass
