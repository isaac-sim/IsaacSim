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

"""Basic functionality tests for behavior scripts."""

from __future__ import annotations

import importlib
import os
from unittest.mock import patch

import carb
import isaacsim.replicator.behavior.behaviors as behaviors_module
import omni.kit.app
import omni.kit.commands
import omni.kit.test
import omni.timeline
import omni.usd
from isaacsim.replicator.behavior.behaviors import LightRandomizer, TextureRandomizer
from isaacsim.replicator.behavior.global_variables import EXPOSED_ATTR_NS
from omni.behavior.scripting.core import BehaviorScript
from pxr import Gf, Sdf

SCRIPTS_ATTR = "omni:scripting:scripts"


class TestBehaviorsBasic(omni.kit.test.AsyncTestCase):
    """Test the basic functionality of the behavior scripts."""

    async def setUp(self) -> None:
        """Set up a new stage before each test."""
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Close the stage after each test."""
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()

    async def check_exposed_variables(self, behavior_class: type) -> None:
        """Verify exposed variables are correctly created and removed for a behavior class.

        Args:
            behavior_class: Behavior class whose exposed variables should be checked.
        """
        # Make sure behavior_class is of type BehaviorScript
        self.assertTrue(
            issubclass(behavior_class, BehaviorScript),
            f"Behavior class '{behavior_class.__name__}' is not a subclass of BehaviorScript",
        )

        # Create a fresh stage with a root prim and some child prims
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        root_prim_path = "/World/RootPrim"
        root_prim = stage.DefinePrim(root_prim_path, "Xform")

        # Add scripting API to the prim
        omni.kit.commands.execute("ApplyScriptingAPICommand", paths=[root_prim_path])
        await omni.kit.app.get_app().next_update_async()
        scripts_attr = root_prim.GetAttribute(SCRIPTS_ATTR)
        self.assertTrue(
            scripts_attr and scripts_attr.IsValid(), f"No '{SCRIPTS_ATTR}' attribute found on prim: {root_prim_path}"
        )

        # Get the script path from the behavior class's module
        module = importlib.import_module(behavior_class.__module__)
        script_full_path = module.__file__

        self.assertTrue(os.path.exists(script_full_path), f"Script path does not exist: {script_full_path}")
        self.assertTrue(script_full_path.endswith(".py"), f"Script path is not a .py file: {script_full_path}")

        # Add the behavior script path the list of scripts on the prim
        current_scripts = scripts_attr.Get()
        if current_scripts is None:
            current_scripts = []
        current_scripts.append(Sdf.AssetPath(script_full_path))
        scripts_attr.Set(current_scripts)

        # NOTE, at least 3 updates are needed to ensure the script is loaded and the exposed vars are set
        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()

        # Check if the class has VARIABLES_TO_EXPOSE
        self.assertTrue(
            hasattr(behavior_class, "VARIABLES_TO_EXPOSE"),
            f"Behavior class '{behavior_class.__name__}' does not have a 'VARIABLES_TO_EXPOSE' attribute",
        )

        # Check if the class has a BEHAVIOR_NS attribute
        self.assertTrue(
            hasattr(behavior_class, "BEHAVIOR_NS"),
            f"Behavior class '{behavior_class.__name__}' does not have a 'BEHAVIOR_NS' attribute",
        )
        # Full namespace for the exposed variables
        full_ns = f"{EXPOSED_ATTR_NS}:{behavior_class.BEHAVIOR_NS}"

        # Iterate over the variables which should be exposed with the behavior script initialization
        for exposed_var in behavior_class.VARIABLES_TO_EXPOSE:
            # Check if the exposed variable is set on the prim
            attr_name = exposed_var["attr_name"]
            print(f"\tChecking exposed variable: {attr_name}")
            attr_full_name = f"{full_ns}:{attr_name}"
            attribute = root_prim.GetAttribute(attr_full_name)
            self.assertTrue(
                attribute and attribute.IsValid(), f"Attribute '{attr_full_name}' not found on prim: {root_prim_path}"
            )

            # Check if the exposed variable has the correct type
            attr_type = exposed_var["attr_type"]
            self.assertEqual(
                attribute.GetTypeName(),
                attr_type,
                f"Attribute '{attr_full_name}' has incorrect type: {attribute.GetTypeName()} instead of {attr_type}",
            )

            # Check if the exposed variable has a default value
            # NOTE: only checking if there is a default value, not the actual value due to rounding issues
            self.assertTrue(
                attribute.Get() is not None,
                f"Attribute '{attr_full_name}' does not have a valid value: {attribute.Get()} ",
            )

            # Check if the exposed variable has the correct documentation
            doc = exposed_var.get("doc", "")
            self.assertEqual(
                attribute.GetDocumentation(),
                doc,
                f"Attribute '{attr_full_name}' has incorrect documentation: {attribute.GetDocumentation()} instead of {doc}",
            )

        # Basic test to check if the behavior can run for a few frames
        timeline = omni.timeline.get_timeline_interface()
        print(f"\tStarting timeline for several frames")
        timeline.play()
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
        print(f"\tPausing timeline")
        timeline.pause()
        await omni.kit.app.get_app().next_update_async()
        print(f"\tStopping timeline")
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Remove the behavior script from the prim, this should also remove/invalidate the attributes of the exposed variables
        print(f"\tChecking if exposed variables are removed after clearing the scripts attribute")
        scripts_attr.Set([])

        # NOTE, at least 3 updates are needed to ensure the script is loaded and the exposed vars are set
        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()

        # Check if the exposed variables are removed
        for exposed_var in behavior_class.VARIABLES_TO_EXPOSE:
            attr_name = exposed_var["attr_name"]
            attr_full_name = f"{full_ns}:{attr_name}"
            attribute = root_prim.GetAttribute(attr_full_name)
            self.assertFalse(
                attribute and attribute.IsValid(),
                f"Attribute '{attr_full_name}' with {attribute.Get()} not removed from prim: {root_prim_path}",
            )

    async def test_exposed_variables(self) -> None:
        """Test that all behavior classes correctly expose and remove their variables."""
        # Get the behavior classes to test from the behaviors module __all__ list
        BEHAVIOR_CLASSES = [getattr(behaviors_module, class_name) for class_name in behaviors_module.__all__]
        carb.log_info(f"BEHAVIOR_CLASSES: {BEHAVIOR_CLASSES}")
        for behavior_class in BEHAVIOR_CLASSES:
            print(f"Testing behavior: {behavior_class.__name__}")
            await self.check_exposed_variables(behavior_class)

    async def test_light_randomizer_reset_skips_cached_none_values(self) -> None:
        """Test that light reset skips unset cached attributes and restores remaining prims."""
        stage = omni.usd.get_context().get_stage()

        unset_prim = stage.DefinePrim("/World/UnsetLightInputs", "Xform")
        unset_prim.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float)
        unset_prim.CreateAttribute("inputs:color", Sdf.ValueTypeNames.Color3f)

        authored_prim = stage.DefinePrim("/World/AuthoredLightInputs", "Xform")
        authored_prim.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float)
        authored_prim.CreateAttribute("inputs:color", Sdf.ValueTypeNames.Color3f)
        initial_intensity = 500.0
        initial_color = Gf.Vec3f(0.1, 0.2, 0.3)
        authored_prim.GetAttribute("inputs:intensity").Set(initial_intensity)
        authored_prim.GetAttribute("inputs:color").Set(initial_color)

        randomizer = LightRandomizer.__new__(LightRandomizer)
        randomizer._initial_attributes = {}
        randomizer._cache_initial_attributes(unset_prim)
        randomizer._cache_initial_attributes(authored_prim)
        self.assertIsNone(randomizer._initial_attributes[unset_prim]["inputs:intensity"])
        self.assertIsNone(randomizer._initial_attributes[unset_prim]["inputs:color"])

        unset_prim.GetAttribute("inputs:intensity").Set(1000.0)
        unset_prim.GetAttribute("inputs:color").Set(Gf.Vec3f(0.5, 0.5, 0.5))
        authored_prim.GetAttribute("inputs:intensity").Set(1000.0)
        authored_prim.GetAttribute("inputs:color").Set(Gf.Vec3f(0.5, 0.5, 0.5))

        randomizer._valid_prims = [unset_prim, authored_prim]
        randomizer._update_counter = 1
        randomizer._rng = object()

        randomizer._reset()

        self.assertEqual(authored_prim.GetAttribute("inputs:intensity").Get(), initial_intensity)
        self.assertEqual(authored_prim.GetAttribute("inputs:color").Get(), initial_color)
        self.assertEqual(unset_prim.GetAttribute("inputs:intensity").Get(), 1000.0)
        self.assertEqual(unset_prim.GetAttribute("inputs:color").Get(), Gf.Vec3f(0.5, 0.5, 0.5))
        self.assertEqual(randomizer._valid_prims, [])
        self.assertEqual(randomizer._initial_attributes, {})
        self.assertEqual(randomizer._update_counter, 0)
        self.assertIsNone(randomizer._rng)

    async def test_texture_randomizer_apply_skips_when_no_textures_configured(self) -> None:
        """Test that ``_apply_behavior`` is a no-op when no texture URLs are configured.

        Regression test for the empty-list crash where ``numpy.random.Generator.choice``
        raised on an empty texture list. The guard must log a warning and return early
        without iterating ``_texture_materials`` or touching ``_rng``.
        """
        randomizer = TextureRandomizer.__new__(TextureRandomizer)
        randomizer._texture_urls = []

        # If the guard regresses, the loop below would iterate ``_texture_materials`` and
        # call ``self._rng.choice(...)``; ``_rng = None`` would surface that as AttributeError.
        sentinel_material = object()
        randomizer._texture_materials = [sentinel_material]
        randomizer._rng = None

        # ``prim_path`` is provided by the ``BehaviorScript`` base class at runtime;
        # patch it at the class level so the warning's f-string is well-formed.
        with patch.object(TextureRandomizer, "prim_path", "/World/UnusedRandomizerPrim", create=True):
            randomizer._apply_behavior()

        self.assertEqual(randomizer._texture_urls, [])
        self.assertEqual(randomizer._texture_materials, [sentinel_material])
        self.assertIsNone(randomizer._rng)
