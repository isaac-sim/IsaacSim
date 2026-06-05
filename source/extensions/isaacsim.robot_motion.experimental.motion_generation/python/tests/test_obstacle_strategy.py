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

# Import extension python module we are testing with absolute import path, as if we are an external user (i.e. a different extension)

"""Test suite for validating the ObstacleStrategy functionality in robot motion generation."""

import omni.kit.test
from isaacsim.core.experimental.objects import Capsule, Cone, Cube, Cylinder, Mesh, Plane, Sphere
from isaacsim.core.experimental.utils.stage import create_new_stage_async
from isaacsim.robot_motion.experimental.motion_generation import (
    ObstacleConfiguration,
    ObstacleRepresentation,
    ObstacleStrategy,
)


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of the module will make it auto-discoverable by omni.kit.test
class TestObstacleStrategy(omni.kit.test.AsyncTestCase):
    """Test suite for validating the ObstacleStrategy functionality in robot motion generation.

    This class provides comprehensive test coverage for the ObstacleStrategy system, which manages obstacle
    representations and safety tolerances for different geometric shapes in robotic motion planning scenarios.
    The tests verify configuration management, validation rules, and error handling across various obstacle types
    including spheres, cones, cubes, planes, capsules, cylinders, and meshes.

    The test suite validates:
    - Setting and retrieving default obstacle configurations for different shape types
    - Applying shape-specific configuration overrides
    - Safety tolerance management including negative values
    - Validation of legal and illegal obstacle representation combinations
    - Error handling for invalid configurations and non-existent prims
    - All-or-nothing behavior for batch configuration updates

    Each test method focuses on a specific aspect of the ObstacleStrategy functionality, ensuring that the
    system correctly handles various geometric primitives and their corresponding obstacle representations
    while maintaining proper validation and error reporting.
    """

    # Before running each test
    async def setUp(self):
        """Set up test fixtures before each test method is run."""
        pass

    # After running each test
    async def tearDown(self):
        """Clean up after each test method is run."""
        pass

    async def test_representation_as_string(self):
        """Test setting obstacle representation using a string value."""
        obstacle_strategy = ObstacleStrategy()
        obstacle_strategy.set_default_configuration(Mesh, ObstacleConfiguration("obb", 0.05))

    async def test_sphere_strategies(self):
        """Test obstacle strategy configurations for sphere objects.

        Verifies that valid configurations can be set for spheres and that invalid configurations
        are properly rejected. Tests both default configurations and specific prim overrides.
        """
        # can create an ObstacleStrategy:
        obstacle_strategy = ObstacleStrategy()

        stage = await create_new_stage_async()

        # create a sphere in the current scene:
        sphere_path = "/World/Sphere"
        sphere_core_object = Sphere(paths=sphere_path)

        # We can set valid default configurations:
        obstacle_strategy.set_default_configuration(
            Sphere, ObstacleConfiguration(representation="sphere", safety_tolerance=0.0)
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(sphere_path).representation, ObstacleRepresentation.SPHERE
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(sphere_path).safety_tolerance, 0.0)

        obstacle_strategy.set_default_configuration(
            Sphere, ObstacleConfiguration(representation="obb", safety_tolerance=1.0)
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(sphere_path).representation, ObstacleRepresentation.OBB
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(sphere_path).safety_tolerance, 1.0)

        obstacle_strategy.set_default_configuration(
            Sphere,
            ObstacleConfiguration(representation=ObstacleRepresentation.OBB, safety_tolerance=-1.0),
            allow_negative_tolerance=True,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(sphere_path).representation, ObstacleRepresentation.OBB
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(sphere_path).safety_tolerance, -1.0)

        # Cannot set illegal default configurations:
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Sphere,
            ObstacleConfiguration(representation="capsule", safety_tolerance=0.0),
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Sphere,
            ObstacleConfiguration(representation=ObstacleRepresentation.CONE, safety_tolerance=1.0),
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Sphere,
            ObstacleConfiguration(representation="cone", safety_tolerance=-1.0),
            allow_negative_tolerance=False,
        )

        specific_override_sphere_path = "/World/NewSphere"
        specific_override_sphere_core_object = Sphere(paths=specific_override_sphere_path)

        # we can set a valid override for a specific prim:
        obstacle_strategy.set_default_configuration(
            Sphere, ObstacleConfiguration(representation=ObstacleRepresentation.SPHERE, safety_tolerance=0.0)
        )
        obstacle_strategy.set_configuration_overrides(
            {
                specific_override_sphere_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.OBB, safety_tolerance=1.0
                )
            }
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_sphere_path).representation,
            ObstacleRepresentation.OBB,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_sphere_path).safety_tolerance, 1.0
        )

        # The override is only applied to the specific prim:
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(sphere_path).representation, ObstacleRepresentation.SPHERE
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(sphere_path).safety_tolerance, 0.0)

        # we can set a valid override for a specific prim:
        obstacle_strategy.set_configuration_overrides(
            {
                specific_override_sphere_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.OBB, safety_tolerance=-1.0
                )
            },
            allow_negative_tolerance=True,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_sphere_path).representation,
            ObstacleRepresentation.OBB,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_sphere_path).safety_tolerance, -1.0
        )

        # Cannot set an illegal override:
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_sphere_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.CAPSULE, safety_tolerance=0.0
                )
            },
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_sphere_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.CONE, safety_tolerance=1.0
                )
            },
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_sphere_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.CONE, safety_tolerance=-1.0
                )
            },
            allow_negative_tolerance=True,
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_sphere_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.SPHERE, safety_tolerance=-1.0
                )
            },
            allow_negative_tolerance=False,
        )

    async def test_cone_strategies(self):
        """Test obstacle strategy configurations for cone objects.

        Verifies that valid configurations can be set for cones and that invalid configurations
        are properly rejected. Tests both default configurations and specific prim overrides.
        """
        # can create an ObstacleStrategy:
        obstacle_strategy = ObstacleStrategy()

        stage = await create_new_stage_async()

        # create a cone in the current scene:
        cone_path = "/World/Cone"
        cone_core_object = Cone(paths=cone_path)

        # We can set valid default configurations:
        obstacle_strategy.set_default_configuration(
            Cone, ObstacleConfiguration(representation="cone", safety_tolerance=0.0)
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(cone_path).representation, ObstacleRepresentation.CONE
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(cone_path).safety_tolerance, 0.0)

        obstacle_strategy.set_default_configuration(
            Cone, ObstacleConfiguration(representation="obb", safety_tolerance=1.0)
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(cone_path).representation, ObstacleRepresentation.OBB
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(cone_path).safety_tolerance, 1.0)

        obstacle_strategy.set_default_configuration(
            Cone,
            ObstacleConfiguration(representation="obb", safety_tolerance=-1.0),
            allow_negative_tolerance=True,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(cone_path).representation, ObstacleRepresentation.OBB
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(cone_path).safety_tolerance, -1.0)

        # Cannot set illegal default configurations:
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Cone,
            ObstacleConfiguration(representation=ObstacleRepresentation.CAPSULE, safety_tolerance=0.0),
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Cone,
            ObstacleConfiguration(representation=ObstacleRepresentation.SPHERE, safety_tolerance=1.0),
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Cone,
            ObstacleConfiguration(representation=ObstacleRepresentation.SPHERE, safety_tolerance=-1.0),
            allow_negative_tolerance=False,
        )

        specific_override_cone_path = "/World/NewCone"
        specific_override_cone_core_object = Cone(paths=specific_override_cone_path)

        # we can set a valid override for a specific prim:
        obstacle_strategy.set_default_configuration(
            Cone, ObstacleConfiguration(representation=ObstacleRepresentation.CONE, safety_tolerance=0.0)
        )
        obstacle_strategy.set_configuration_overrides(
            {
                specific_override_cone_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.OBB, safety_tolerance=1.0
                )
            }
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_cone_path).representation,
            ObstacleRepresentation.OBB,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_cone_path).safety_tolerance, 1.0
        )

        # The override is only applied to the specific prim:
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(cone_path).representation, ObstacleRepresentation.CONE
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(cone_path).safety_tolerance, 0.0)

        # we can set a valid override for a specific prim:
        obstacle_strategy.set_configuration_overrides(
            {
                specific_override_cone_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.OBB, safety_tolerance=-1.0
                )
            },
            allow_negative_tolerance=True,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_cone_path).representation,
            ObstacleRepresentation.OBB,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_cone_path).safety_tolerance, -1.0
        )

        # Cannot set an illegal override:
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_cone_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.CAPSULE, safety_tolerance=0.0
                )
            },
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_cone_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.SPHERE, safety_tolerance=1.0
                )
            },
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_cone_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.PLANE, safety_tolerance=-1.0
                )
            },
            allow_negative_tolerance=True,
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_cone_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.CONE, safety_tolerance=-1.0
                )
            },
            allow_negative_tolerance=False,
        )

    async def test_cube_strategies(self):
        """Test obstacle strategy configurations for cube objects.

        Verifies that valid configurations can be set for cubes and that invalid configurations
        are properly rejected. Tests both default configurations and specific prim overrides.
        """
        # can create an ObstacleStrategy:
        obstacle_strategy = ObstacleStrategy()

        stage = await create_new_stage_async()

        # create a cube in the current scene:
        cube_path = "/World/Cube"
        cube_core_object = Cube(paths=cube_path)

        # We can set valid default configurations:
        obstacle_strategy.set_default_configuration(
            Cube, ObstacleConfiguration(representation=ObstacleRepresentation.CUBE, safety_tolerance=0.0)
        )

        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(cube_path).representation, ObstacleRepresentation.CUBE
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(cube_path).safety_tolerance, 0.0)

        obstacle_strategy.set_default_configuration(
            Cube, ObstacleConfiguration(representation=ObstacleRepresentation.OBB, safety_tolerance=1.0)
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(cube_path).representation, ObstacleRepresentation.OBB
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(cube_path).safety_tolerance, 1.0)

        obstacle_strategy.set_default_configuration(
            Cube,
            ObstacleConfiguration(representation=ObstacleRepresentation.OBB, safety_tolerance=-1.0),
            allow_negative_tolerance=True,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(cube_path).representation, ObstacleRepresentation.OBB
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(cube_path).safety_tolerance, -1.0)

        # Cannot set illegal default configurations:
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Cube,
            ObstacleConfiguration(representation=ObstacleRepresentation.CAPSULE, safety_tolerance=0.0),
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Cube,
            ObstacleConfiguration(representation=ObstacleRepresentation.SPHERE, safety_tolerance=1.0),
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Cube,
            ObstacleConfiguration(representation=ObstacleRepresentation.SPHERE, safety_tolerance=-1.0),
            allow_negative_tolerance=False,
        )
        specific_override_cube_path = "/World/NewCube"
        specific_override_cube_core_object = Cube(paths=specific_override_cube_path)

        # we can set a valid override for a specific prim:
        obstacle_strategy.set_default_configuration(
            Cube, ObstacleConfiguration(representation=ObstacleRepresentation.CUBE, safety_tolerance=0.0)
        )
        obstacle_strategy.set_configuration_overrides(
            {
                specific_override_cube_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.OBB, safety_tolerance=1.0
                )
            }
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_cube_path).representation,
            ObstacleRepresentation.OBB,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_cube_path).safety_tolerance, 1.0
        )

        # The override is only applied to the specific prim:
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(cube_path).representation, ObstacleRepresentation.CUBE
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(cube_path).safety_tolerance, 0.0)

        # we can set a valid override for a specific prim:
        obstacle_strategy.set_configuration_overrides(
            {
                specific_override_cube_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.OBB, safety_tolerance=-1.0
                )
            },
            allow_negative_tolerance=True,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_cube_path).representation,
            ObstacleRepresentation.OBB,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_cube_path).safety_tolerance, -1.0
        )

        # Cannot set an illegal override:
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_cube_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.CAPSULE, safety_tolerance=0.0
                )
            },
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_cube_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.SPHERE, safety_tolerance=1.0
                )
            },
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_cube_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.PLANE, safety_tolerance=-1.0
                )
            },
            allow_negative_tolerance=True,
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_cube_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.CUBE, safety_tolerance=-1.0
                )
            },
            allow_negative_tolerance=False,
        )

    async def test_plane_strategies(self):
        """Test obstacle strategy configurations for plane objects.

        Verifies that valid configurations can be set for planes and that invalid configurations
        are properly rejected. Tests both default configurations and specific prim overrides.
        Planes only support PLANE representation.
        """
        # can create an ObstacleStrategy:
        obstacle_strategy = ObstacleStrategy()

        stage = await create_new_stage_async()

        # create a plane in the current scene:
        plane_path = "/World/Plane"
        plane_core_object = Plane(paths=plane_path)

        # We can set valid default configurations. In the specific case of planes, the only valid representation is PLANE.
        obstacle_strategy.set_default_configuration(
            Plane, ObstacleConfiguration(representation=ObstacleRepresentation.PLANE, safety_tolerance=0.0)
        )

        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(plane_path).representation, ObstacleRepresentation.PLANE
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(plane_path).safety_tolerance, 0.0)

        obstacle_strategy.set_default_configuration(
            Plane, ObstacleConfiguration(representation=ObstacleRepresentation.PLANE, safety_tolerance=1.0)
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(plane_path).representation, ObstacleRepresentation.PLANE
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(plane_path).safety_tolerance, 1.0)

        obstacle_strategy.set_default_configuration(
            Plane,
            ObstacleConfiguration(representation=ObstacleRepresentation.PLANE, safety_tolerance=-1.0),
            allow_negative_tolerance=True,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(plane_path).representation, ObstacleRepresentation.PLANE
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(plane_path).safety_tolerance, -1.0)

        # Cannot set illegal default configurations:
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Plane,
            ObstacleConfiguration(representation=ObstacleRepresentation.OBB, safety_tolerance=0.0),
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Plane,
            ObstacleConfiguration(representation=ObstacleRepresentation.SPHERE, safety_tolerance=1.0),
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Plane,
            ObstacleConfiguration(representation=ObstacleRepresentation.PLANE, safety_tolerance=-1.0),
            allow_negative_tolerance=False,
        )
        specific_override_plane_path = "/World/NewPlane"
        specific_override_plane_core_object = Plane(paths=specific_override_plane_path)

        # we can set a valid override for a specific prim:
        obstacle_strategy.set_default_configuration(
            Plane, ObstacleConfiguration(representation=ObstacleRepresentation.PLANE, safety_tolerance=0.0)
        )
        obstacle_strategy.set_configuration_overrides(
            {
                specific_override_plane_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.PLANE, safety_tolerance=1.0
                )
            }
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_plane_path).representation,
            ObstacleRepresentation.PLANE,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_plane_path).safety_tolerance, 1.0
        )

        # The override is only applied to the specific prim:
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(plane_path).representation, ObstacleRepresentation.PLANE
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(plane_path).safety_tolerance, 0.0)

        # we can set a valid override for a specific prim:
        obstacle_strategy.set_configuration_overrides(
            {
                specific_override_plane_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.PLANE, safety_tolerance=-1.0
                )
            },
            allow_negative_tolerance=True,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_plane_path).representation,
            ObstacleRepresentation.PLANE,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_plane_path).safety_tolerance, -1.0
        )

        # Cannot set an illegal override:
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_plane_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.CAPSULE, safety_tolerance=0.0
                )
            },
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_plane_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.OBB, safety_tolerance=1.0
                )
            },
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_plane_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.PLANE, safety_tolerance=-1.0
                )
            },
            allow_negative_tolerance=False,
        )

    async def test_capsule_strategies(self):
        """Test obstacle strategy configurations for capsule objects.

        Verifies that valid configurations can be set for capsules and that invalid configurations
        are properly rejected. Tests both default configurations and specific prim overrides.
        """
        # can create an ObstacleStrategy:
        obstacle_strategy = ObstacleStrategy()

        stage = await create_new_stage_async()

        # create a capsule in the current scene:
        capsule_path = "/World/Capsule"
        capsule_core_object = Capsule(paths=capsule_path)

        # We can set valid default configurations:
        obstacle_strategy.set_default_configuration(
            Capsule, ObstacleConfiguration(representation=ObstacleRepresentation.CAPSULE, safety_tolerance=0.0)
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(capsule_path).representation, ObstacleRepresentation.CAPSULE
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(capsule_path).safety_tolerance, 0.0)

        obstacle_strategy.set_default_configuration(
            Capsule, ObstacleConfiguration(representation=ObstacleRepresentation.OBB, safety_tolerance=1.0)
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(capsule_path).representation, ObstacleRepresentation.OBB
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(capsule_path).safety_tolerance, 1.0)

        obstacle_strategy.set_default_configuration(
            Capsule,
            ObstacleConfiguration(representation=ObstacleRepresentation.OBB, safety_tolerance=-1.0),
            allow_negative_tolerance=True,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(capsule_path).representation, ObstacleRepresentation.OBB
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(capsule_path).safety_tolerance, -1.0)

        # Cannot set illegal default configurations:
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Capsule,
            ObstacleConfiguration(representation=ObstacleRepresentation.SPHERE, safety_tolerance=0.0),
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Capsule,
            ObstacleConfiguration(representation=ObstacleRepresentation.CONE, safety_tolerance=1.0),
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Capsule,
            ObstacleConfiguration(representation=ObstacleRepresentation.CAPSULE, safety_tolerance=-1.0),
            allow_negative_tolerance=False,
        )

        specific_override_capsule_path = "/World/NewCapsule"
        specific_override_capsule_core_object = Capsule(paths=specific_override_capsule_path)

        # we can set a valid override for a specific prim:
        obstacle_strategy.set_default_configuration(
            Capsule, ObstacleConfiguration(representation=ObstacleRepresentation.CAPSULE, safety_tolerance=0.0)
        )
        obstacle_strategy.set_configuration_overrides(
            {
                specific_override_capsule_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.OBB, safety_tolerance=1.0
                )
            }
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_capsule_path).representation,
            ObstacleRepresentation.OBB,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_capsule_path).safety_tolerance, 1.0
        )

        # The override is only applied to the specific prim:
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(capsule_path).representation, ObstacleRepresentation.CAPSULE
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(capsule_path).safety_tolerance, 0.0)

        # we can set a valid override for a specific prim:
        obstacle_strategy.set_configuration_overrides(
            {
                specific_override_capsule_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.OBB, safety_tolerance=-1.0
                )
            },
            allow_negative_tolerance=True,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_capsule_path).representation,
            ObstacleRepresentation.OBB,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_capsule_path).safety_tolerance, -1.0
        )

        # Cannot set an illegal override:
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_capsule_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.SPHERE, safety_tolerance=0.0
                )
            },
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_capsule_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.CONE, safety_tolerance=1.0
                )
            },
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_capsule_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.CAPSULE, safety_tolerance=-1.0
                )
            },
            allow_negative_tolerance=False,
        )

    async def test_cylinder_strategies(self):
        """Test obstacle strategy configurations for cylinder objects.

        Verifies that valid configurations can be set for cylinders and that invalid configurations
        are properly rejected. Tests both default configurations and specific prim overrides.
        """
        # can create an ObstacleStrategy:
        obstacle_strategy = ObstacleStrategy()

        stage = await create_new_stage_async()

        # create a cylinder in the current scene:
        cylinder_path = "/World/Cylinder"
        cylinder_core_object = Cylinder(paths=cylinder_path)

        # We can set valid default configurations:
        obstacle_strategy.set_default_configuration(
            Cylinder, ObstacleConfiguration(representation=ObstacleRepresentation.CYLINDER, safety_tolerance=0.0)
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(cylinder_path).representation, ObstacleRepresentation.CYLINDER
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(cylinder_path).safety_tolerance, 0.0)

        obstacle_strategy.set_default_configuration(
            Cylinder, ObstacleConfiguration(representation=ObstacleRepresentation.OBB, safety_tolerance=1.0)
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(cylinder_path).representation, ObstacleRepresentation.OBB
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(cylinder_path).safety_tolerance, 1.0)

        obstacle_strategy.set_default_configuration(
            Cylinder,
            ObstacleConfiguration(representation=ObstacleRepresentation.OBB, safety_tolerance=-1.0),
            allow_negative_tolerance=True,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(cylinder_path).representation, ObstacleRepresentation.OBB
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(cylinder_path).safety_tolerance, -1.0)

        # Cannot set illegal default configurations:
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Cylinder,
            ObstacleConfiguration(representation=ObstacleRepresentation.SPHERE, safety_tolerance=0.0),
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Cylinder,
            ObstacleConfiguration(representation=ObstacleRepresentation.CONE, safety_tolerance=1.0),
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Cylinder,
            ObstacleConfiguration(representation=ObstacleRepresentation.CYLINDER, safety_tolerance=-1.0),
            allow_negative_tolerance=False,
        )

        specific_override_cylinder_path = "/World/NewCylinder"
        specific_override_cylinder_core_object = Cylinder(paths=specific_override_cylinder_path)

        # we can set a valid override for a specific prim:
        obstacle_strategy.set_default_configuration(
            Cylinder, ObstacleConfiguration(representation=ObstacleRepresentation.CYLINDER, safety_tolerance=0.0)
        )
        obstacle_strategy.set_configuration_overrides(
            {
                specific_override_cylinder_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.OBB, safety_tolerance=1.0
                )
            }
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_cylinder_path).representation,
            ObstacleRepresentation.OBB,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_cylinder_path).safety_tolerance, 1.0
        )

        # The override is only applied to the specific prim:
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(cylinder_path).representation, ObstacleRepresentation.CYLINDER
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(cylinder_path).safety_tolerance, 0.0)

        # we can set a valid override for a specific prim:
        obstacle_strategy.set_configuration_overrides(
            {
                specific_override_cylinder_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.OBB, safety_tolerance=-1.0
                )
            },
            allow_negative_tolerance=True,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_cylinder_path).representation,
            ObstacleRepresentation.OBB,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_cylinder_path).safety_tolerance, -1.0
        )

        # Cannot set an illegal override:
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_cylinder_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.SPHERE, safety_tolerance=0.0
                )
            },
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_cylinder_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.CONE, safety_tolerance=1.0
                )
            },
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_cylinder_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.CYLINDER, safety_tolerance=-1.0
                )
            },
            allow_negative_tolerance=False,
        )

    async def test_mesh_strategies(self):
        """Test obstacle strategy configurations for mesh objects.

        Verifies that valid configurations can be set for meshes and that invalid configurations
        are properly rejected. Tests both default configurations and specific prim overrides.
        Meshes support MESH, TRIANGULATED_MESH, and OBB representations.
        """
        # can create an ObstacleStrategy:
        obstacle_strategy = ObstacleStrategy()

        stage = await create_new_stage_async()

        # create a mesh in the current scene:
        mesh_path = "/World/Mesh"
        mesh_core_object = Mesh(paths=mesh_path)

        # We can set valid default configurations:
        obstacle_strategy.set_default_configuration(
            Mesh, ObstacleConfiguration(representation=ObstacleRepresentation.MESH, safety_tolerance=0.0)
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(mesh_path).representation, ObstacleRepresentation.MESH
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(mesh_path).safety_tolerance, 0.0)

        obstacle_strategy.set_default_configuration(
            Mesh, ObstacleConfiguration(representation=ObstacleRepresentation.TRIANGULATED_MESH, safety_tolerance=1.0)
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(mesh_path).representation,
            ObstacleRepresentation.TRIANGULATED_MESH,
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(mesh_path).safety_tolerance, 1.0)

        obstacle_strategy.set_default_configuration(
            Mesh,
            ObstacleConfiguration(representation=ObstacleRepresentation.OBB, safety_tolerance=-1.0),
            allow_negative_tolerance=True,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(mesh_path).representation, ObstacleRepresentation.OBB
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(mesh_path).safety_tolerance, -1.0)

        # Cannot set illegal default configurations:
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Mesh,
            ObstacleConfiguration(representation=ObstacleRepresentation.SPHERE, safety_tolerance=0.0),
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Mesh,
            ObstacleConfiguration(representation=ObstacleRepresentation.CONE, safety_tolerance=1.0),
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Mesh,
            ObstacleConfiguration(representation=ObstacleRepresentation.MESH, safety_tolerance=-1.0),
            allow_negative_tolerance=False,
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_default_configuration,
            Mesh,
            ObstacleConfiguration(representation=ObstacleRepresentation.TRIANGULATED_MESH, safety_tolerance=-1.0),
            allow_negative_tolerance=False,
        )
        specific_override_mesh_path = "/World/NewMesh"
        specific_override_mesh_core_object = Mesh(paths=specific_override_mesh_path)

        # we can set a valid override for a specific prim:
        obstacle_strategy.set_default_configuration(
            Mesh, ObstacleConfiguration(representation=ObstacleRepresentation.MESH, safety_tolerance=0.0)
        )
        obstacle_strategy.set_configuration_overrides(
            {
                specific_override_mesh_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.OBB, safety_tolerance=1.0
                )
            }
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_mesh_path).representation,
            ObstacleRepresentation.OBB,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_mesh_path).safety_tolerance, 1.0
        )

        # The override is only applied to the specific prim:
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(mesh_path).representation, ObstacleRepresentation.MESH
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(mesh_path).safety_tolerance, 0.0)

        # we can set a valid override for a specific prim:
        obstacle_strategy.set_configuration_overrides(
            {
                specific_override_mesh_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.TRIANGULATED_MESH, safety_tolerance=-1.0
                )
            },
            allow_negative_tolerance=True,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_mesh_path).representation,
            ObstacleRepresentation.TRIANGULATED_MESH,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_mesh_path).safety_tolerance, -1.0
        )

        # Cannot set an illegal override:
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_mesh_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.SPHERE, safety_tolerance=0.0
                )
            },
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_mesh_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.CONE, safety_tolerance=1.0
                )
            },
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_mesh_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.MESH, safety_tolerance=-1.0
                )
            },
            allow_negative_tolerance=False,
        )
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                specific_override_mesh_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.TRIANGULATED_MESH, safety_tolerance=-1.0
                )
            },
            allow_negative_tolerance=False,
        )

    async def test_default_safety_tolerance(self):
        """Test default safety tolerance settings for all obstacle types.

        Verifies that default safety tolerance can be set for all geometric shapes and that
        negative tolerance values are properly handled. Tests that overrides are not affected
        by default tolerance changes.
        """
        obstacle_strategy = ObstacleStrategy()

        stage = await create_new_stage_async()

        sphere_path = "/World/Sphere"
        cone_path = "/World/Cone"
        cube_path = "/World/Cube"
        plane_path = "/World/Plane"
        capsule_path = "/World/Capsule"
        cylinder_path = "/World/Cylinder"
        mesh_path = "/World/Mesh"

        sphere_core_object = Sphere(paths=sphere_path)
        cone_core_object = Cone(paths=cone_path)
        cube_core_object = Cube(paths=cube_path)
        plane_core_object = Plane(paths=plane_path)
        capsule_core_object = Capsule(paths=capsule_path)
        cylinder_core_object = Cylinder(paths=cylinder_path)
        mesh_core_object = Mesh(paths=mesh_path)

        obstacle_strategy.set_default_safety_tolerance(0.5)
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(sphere_path).safety_tolerance, 0.5)
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(cone_path).safety_tolerance, 0.5)
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(cube_path).safety_tolerance, 0.5)
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(plane_path).safety_tolerance, 0.5)
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(capsule_path).safety_tolerance, 0.5)
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(cylinder_path).safety_tolerance, 0.5)
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(mesh_path).safety_tolerance, 0.5)

        # We cannot set a negative safety tolerance if we don't specify allow_negative_tolerance=True:
        self.assertRaises(ValueError, obstacle_strategy.set_default_safety_tolerance, -0.5)

        # We can set a negative safety tolerance if we specify allow_negative_tolerance=True:
        obstacle_strategy.set_default_safety_tolerance(-0.25, allow_negative_tolerance=True)
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(sphere_path).safety_tolerance, -0.25)
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(cone_path).safety_tolerance, -0.25)
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(cube_path).safety_tolerance, -0.25)
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(plane_path).safety_tolerance, -0.25)
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(capsule_path).safety_tolerance, -0.25)
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(cylinder_path).safety_tolerance, -0.25)
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(mesh_path).safety_tolerance, -0.25)

        # If I have an overriden safety tolerance, it should not be affected by the default safety tolerance:
        specific_override_sphere_path = "/World/Sphere"
        specific_override_sphere_core_object = Sphere(paths=specific_override_sphere_path)
        obstacle_strategy.set_configuration_overrides(
            {
                specific_override_sphere_path: ObstacleConfiguration(
                    representation=ObstacleRepresentation.OBB, safety_tolerance=-0.1
                )
            },
            allow_negative_tolerance=True,
        )
        obstacle_strategy.set_default_safety_tolerance(-0.25, allow_negative_tolerance=True)
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_sphere_path).representation,
            ObstacleRepresentation.OBB,
        )
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(specific_override_sphere_path).safety_tolerance, -0.1
        )

    async def test_configuration_overrides_all_or_nothing(self):
        """Tests that configuration overrides are applied atomically (all-or-nothing).

        Verifies that when setting multiple configuration overrides simultaneously,
        if any override is invalid, no changes are applied to any prims.
        """
        obstacle_strategy = ObstacleStrategy()

        stage = await create_new_stage_async()

        sphere_path = "/World/Sphere"
        cube_path = "/World/Cube"
        sphere_core_object = Sphere(paths=sphere_path)
        cube_core_object = Cube(paths=cube_path)

        obstacle_strategy.set_default_configuration(
            Sphere, ObstacleConfiguration(representation=ObstacleRepresentation.SPHERE, safety_tolerance=0.0)
        )

        # Here, we try to set multiple overrides at once, but one of them is illegal. This should raise a ValueError, and no overrides should be applied.
        self.assertRaises(
            ValueError,
            obstacle_strategy.set_configuration_overrides,
            {
                sphere_path: ObstacleConfiguration(representation=ObstacleRepresentation.OBB, safety_tolerance=1.0),
                cube_path: ObstacleConfiguration(representation=ObstacleRepresentation.SPHERE, safety_tolerance=1.0),
            },
        )

        # verify that no overrides were applied:
        self.assertEqual(
            obstacle_strategy.get_obstacle_configuration(sphere_path).representation, ObstacleRepresentation.SPHERE
        )
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(sphere_path).safety_tolerance, 0.0)

    async def test_get_obstacle_configuration_invalid_prim(self):
        """Tests that getting obstacle configuration for an invalid prim raises RuntimeError.

        Verifies that attempting to get obstacle configuration for a non-existent
        or invalid prim path raises an appropriate error.
        """
        obstacle_strategy = ObstacleStrategy()

        stage = await create_new_stage_async()
        with self.assertRaises(RuntimeError):
            obstacle_strategy.get_obstacle_configuration("/World/NotAShape")

    async def test_set_configuration_overrides_invalid_prim(self):
        """Test that setting configuration overrides with invalid prims raise an error."""
        obstacle_strategy = ObstacleStrategy()

        stage = await create_new_stage_async()
        Cube("/World/ValidPrim")
        with self.assertRaises(RuntimeError):
            obstacle_strategy.set_configuration_overrides(
                {
                    "/World/NotAShape": ObstacleConfiguration("mesh", 0.05),
                    "/World/AlsoNotAShape": ObstacleConfiguration("obb", 0.05),
                    "/World/ValidPrim": ObstacleConfiguration("obb", 0.05),
                },
            )

    async def test_invalid_representation_type(self):
        """Test that an invalid obstacle representation type raise a ValueError."""
        ObstacleRepresentation("sphere")
        with self.assertRaises(ValueError):
            ObstacleRepresentation("not_a_type")

    async def test_shape_level_configuration_overrides(self):
        """Test shape-level configuration overrides for obstacle strategy."""
        obstacle_strategy = ObstacleStrategy()

        await create_new_stage_async()

        sphere_path = "/World/Sphere"
        Sphere(paths=sphere_path)

        cube_path = "/World/Cube"
        Cube(paths=cube_path)

        # first, set the shape-level configuration:
        obstacle_strategy.set_default_configuration(Sphere, ObstacleConfiguration("sphere", 0.2))

        # now, set a global default safety tolerance:
        obstacle_strategy.set_default_safety_tolerance(0.1)

        # the shape-level configuration should take precedence over the global default safety tolerance:
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(sphere_path).safety_tolerance, 0.2)
        self.assertEqual(obstacle_strategy.get_obstacle_configuration(cube_path).safety_tolerance, 0.1)
