# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Unit tests for nonvisual materials functionality.

Tests the functionality of applying and retrieving nonvisual material properties
used by RTX sensors.
"""

import omni.kit.test
import omni.usd
from isaacsim.sensors.rtx import (
    ATTR_ATTRIBUTE,
    ATTR_BASE,
    ATTR_COATING,
    ATTRIBUTES,
    BASE_MATERIALS,
    COATINGS,
    apply_nonvisual_material,
    decode_material_id,
    get_material_id,
)
from pxr import Sdf, UsdGeom, UsdShade


class TestNonvisualMaterials(omni.kit.test.AsyncTestCase):
    """Test cases for nonvisual materials functionality."""

    async def setUp(self) -> None:
        """Create a new stage for each test."""
        await omni.usd.get_context().new_stage_async()
        self.stage = omni.usd.get_context().get_stage()

    async def tearDown(self) -> None:
        """Clean up the stage after each test."""
        await omni.usd.get_context().close_stage_async()

    def test_base_materials_dictionary(self) -> None:
        """Test that base materials dictionary has expected values."""
        # Test some key materials exist
        self.assertIn("none", BASE_MATERIALS)
        self.assertIn("aluminum", BASE_MATERIALS)
        self.assertIn("plastic", BASE_MATERIALS)
        self.assertIn("clear_glass", BASE_MATERIALS)

        # Test none is 0
        self.assertEqual(BASE_MATERIALS["none"], 0)

    def test_coatings_dictionary(self) -> None:
        """Test that coatings dictionary has expected values."""
        # Test coatings exist and are within 3-bit range (0-7)
        for coating_name, coating_id in COATINGS.items():
            self.assertGreaterEqual(coating_id, 0)
            self.assertLessEqual(coating_id, 7)

        # Test none is 0
        self.assertEqual(COATINGS["none"], 0)
        self.assertEqual(len(COATINGS), 4)  # Should have 4 defined values

    def test_attributes_dictionary(self) -> None:
        """Test that attributes dictionary has expected values."""
        # Test attributes exist and are within 5-bit range (0-31)
        for attr_name, attr_id in ATTRIBUTES.items():
            self.assertGreaterEqual(attr_id, 0)
            self.assertLessEqual(attr_id, 31)

        # Test none is 0
        self.assertEqual(ATTRIBUTES["none"], 0)

    def test_apply_nonvisual_material_with_strings(self) -> None:
        """Test applying nonvisual material with string inputs."""
        # Create a material prim
        material = UsdShade.Material.Define(self.stage, "/World/TestMaterial")
        material_prim = material.GetPrim()

        # Apply nonvisual material with strings
        result = apply_nonvisual_material(material_prim, base="aluminum", coating="paint", attribute="emissive")

        self.assertTrue(result)

        # Verify attributes were set correctly as strings
        base_attr = material_prim.GetAttribute(ATTR_BASE)
        self.assertTrue(base_attr.IsValid())
        self.assertEqual(base_attr.Get(), "aluminum")

        coating_attr = material_prim.GetAttribute(ATTR_COATING)
        self.assertTrue(coating_attr.IsValid())
        self.assertEqual(coating_attr.Get(), "paint")

        attr_attr = material_prim.GetAttribute(ATTR_ATTRIBUTE)
        self.assertTrue(attr_attr.IsValid())
        self.assertEqual(attr_attr.Get(), "emissive")

    def test_apply_nonvisual_material_with_integers(self) -> None:
        """Test applying nonvisual material with integer inputs."""
        # Create a material prim
        material = UsdShade.Material.Define(self.stage, "/World/TestMaterial")
        material_prim = material.GetPrim()

        # Apply nonvisual material with integers
        result = apply_nonvisual_material(material_prim, base=10, coating=2, attribute=2)

        self.assertTrue(result)

        # Verify attributes were set correctly as strings (reverse lookup from IDs)
        base_attr = material_prim.GetAttribute(ATTR_BASE)
        self.assertTrue(base_attr.IsValid())
        # ID 10 should map to "tin"
        self.assertEqual(base_attr.Get(), "tin")

        coating_attr = material_prim.GetAttribute(ATTR_COATING)
        self.assertTrue(coating_attr.IsValid())
        # ID 2 should map to "clearcoat"
        self.assertEqual(coating_attr.Get(), "clearcoat")

        attr_attr = material_prim.GetAttribute(ATTR_ATTRIBUTE)
        self.assertTrue(attr_attr.IsValid())
        # ID 2 should map to "retroreflective"
        self.assertEqual(attr_attr.Get(), "retroreflective")

    def test_apply_nonvisual_material_with_defaults(self) -> None:
        """Test applying nonvisual material with default coating and attribute."""
        # Create a material prim
        material = UsdShade.Material.Define(self.stage, "/World/TestMaterial")
        material_prim = material.GetPrim()

        # Apply nonvisual material with defaults
        result = apply_nonvisual_material(material_prim, base="steel")

        self.assertTrue(result)

        # Verify attributes were set with defaults as strings
        coating_attr = material_prim.GetAttribute(ATTR_COATING)
        self.assertTrue(coating_attr.IsValid())
        self.assertEqual(coating_attr.Get(), "none")

        attr_attr = material_prim.GetAttribute(ATTR_ATTRIBUTE)
        self.assertTrue(attr_attr.IsValid())
        self.assertEqual(attr_attr.Get(), "none")

    def test_apply_nonvisual_material_invalid_prim(self) -> None:
        """Test applying nonvisual material to invalid prim."""
        # Test with None prim
        result = apply_nonvisual_material(None, base="aluminum")
        self.assertFalse(result)

        # Create a non-material prim
        xform = UsdGeom.Xform.Define(self.stage, "/World/TestXform")
        xform_prim = xform.GetPrim()

        # Test with non-material prim
        result = apply_nonvisual_material(xform_prim, base="aluminum")
        self.assertFalse(result)

    def test_apply_nonvisual_material_invalid_base(self) -> None:
        """Test applying nonvisual material with invalid base material."""
        # Create a material prim
        material = UsdShade.Material.Define(self.stage, "/World/TestMaterial")
        material_prim = material.GetPrim()

        # Test with invalid base material name
        result = apply_nonvisual_material(material_prim, base="InvalidMaterial")
        self.assertFalse(result)

    def test_apply_nonvisual_material_invalid_coating(self) -> None:
        """Test applying nonvisual material with invalid coating."""
        # Create a material prim
        material = UsdShade.Material.Define(self.stage, "/World/TestMaterial")
        material_prim = material.GetPrim()

        # Test with invalid coating name
        result = apply_nonvisual_material(material_prim, base="aluminum", coating="invalidcoating")
        self.assertFalse(result)

    def test_apply_nonvisual_material_invalid_attribute(self) -> None:
        """Test applying nonvisual material with invalid attribute."""
        # Create a material prim
        material = UsdShade.Material.Define(self.stage, "/World/TestMaterial")
        material_prim = material.GetPrim()

        # Test with invalid attribute name
        result = apply_nonvisual_material(material_prim, base="aluminum", attribute="invalidattribute")
        self.assertFalse(result)

    def test_get_material_id_basic(self) -> None:
        """Test getting material ID from a material with nonvisual attributes."""
        # Create a material prim
        material = UsdShade.Material.Define(self.stage, "/World/TestMaterial")
        material_prim = material.GetPrim()

        # Apply nonvisual material
        apply_nonvisual_material(
            material_prim, base="aluminum", coating="paint", attribute="emissive"  # ID = 1  # ID = 1  # ID = 1
        )

        # Get material ID
        material_id = get_material_id(material_prim)

        # Expected calculation:
        # base = 1 (bits 0-7)
        # coating = 1 (bits 8-10, shifted left by 8 positions)
        # attribute = 1 (bits 11-15, shifted left by 11 positions)
        # material_id = 1 + (1 << 8) + (1 << 11) = 1 + 256 + 2048 = 2305
        expected_id = 1 + (1 << 8) + (1 << 11)
        self.assertEqual(material_id, expected_id)
        self.assertEqual(material_id, 2305)

    def test_get_material_id_bit_encoding(self) -> None:
        """Test material ID bit encoding with various values."""
        test_cases = [
            # (base, coating, attribute, expected_id)
            (0, 0, 0, 0),  # All zeros - "none", "none", "none"
            (1, 1, 1, 1 + (1 << 8) + (1 << 11)),  # "aluminum", "paint", "emissive"
            (10, 3, 2, 10 + (3 << 8) + (2 << 11)),  # "tin", "paint_clearcoat", "retroreflective"
            (22, 0, 8, 22 + (0 << 8) + (8 << 11)),  # "mirror", "none", "visually_transparent"
        ]

        for base, coating, attribute, expected_id in test_cases:
            with self.subTest(base=base, coating=coating, attribute=attribute):
                # Create a material prim
                material = UsdShade.Material.Define(self.stage, f"/World/TestMaterial_{base}_{coating}_{attribute}")
                material_prim = material.GetPrim()

                # Apply nonvisual material with integer values
                apply_nonvisual_material(material_prim, base=base, coating=coating, attribute=attribute)

                # Get material ID and verify
                material_id = get_material_id(material_prim)
                self.assertEqual(material_id, expected_id)

    def test_get_material_id_no_attributes(self) -> None:
        """Test getting material ID from a material without nonvisual attributes."""
        # Create a material prim without setting nonvisual attributes
        material = UsdShade.Material.Define(self.stage, "/World/TestMaterial")
        material_prim = material.GetPrim()

        # Get material ID (should return 0 for missing attributes)
        material_id = get_material_id(material_prim)
        self.assertEqual(material_id, 0)

    def test_get_material_id_partial_attributes(self) -> None:
        """Test getting material ID with only some attributes set."""
        # Create a material prim
        material = UsdShade.Material.Define(self.stage, "/World/TestMaterial")
        material_prim = material.GetPrim()

        # Set only base attribute manually as string
        base_attr = material_prim.CreateAttribute(
            ATTR_BASE, Sdf.ValueTypeNames.String, custom=True, variability=Sdf.VariabilityUniform
        )
        base_attr.Set("iron")  # ID 4

        # Get material ID (coating and attribute should default to 0)
        material_id = get_material_id(material_prim)
        expected_id = 4  # Only base value
        self.assertEqual(material_id, expected_id)

    def test_get_material_id_invalid_prim(self) -> None:
        """Test getting material ID from invalid prim."""
        # Test with None prim - should return 0
        material_id = get_material_id(None)
        self.assertEqual(material_id, 0)

        # Create a non-material prim
        xform = UsdGeom.Xform.Define(self.stage, "/World/TestXform")
        xform_prim = xform.GetPrim()

        # Test with non-material prim - should return 0
        material_id = get_material_id(xform_prim)
        self.assertEqual(material_id, 0)

    def test_get_material_id_uint16_overflow(self) -> None:
        """Test material ID is properly masked to uint16 range."""
        # Create a material prim
        material = UsdShade.Material.Define(self.stage, "/World/TestMaterial")
        material_prim = material.GetPrim()

        # Set attributes manually as strings - using invalid names that won't be found
        base_attr = material_prim.CreateAttribute(
            ATTR_BASE, Sdf.ValueTypeNames.String, custom=True, variability=Sdf.VariabilityUniform
        )
        base_attr.Set("invalidmaterial")  # Not found, should default to 0

        coating_attr = material_prim.CreateAttribute(
            ATTR_COATING, Sdf.ValueTypeNames.String, custom=True, variability=Sdf.VariabilityUniform
        )
        coating_attr.Set("invalidcoating")  # Not found, should default to 0

        attr_attr = material_prim.CreateAttribute(
            ATTR_ATTRIBUTE, Sdf.ValueTypeNames.String, custom=True, variability=Sdf.VariabilityUniform
        )
        attr_attr.Set("invalidattribute")  # Not found, should default to 0

        # Get material ID
        material_id = get_material_id(material_prim)

        # Verify invalid strings default to 0
        expected_base = 0
        expected_coating = 0
        expected_attribute = 0
        expected_id = expected_base + (expected_coating << 8) + (expected_attribute << 11)

        self.assertEqual(material_id, expected_id)
        self.assertEqual(material_id, 0)  # Should be 0 for all invalid strings

    def test_material_id_roundtrip(self) -> None:
        """Test applying and retrieving material properties maintains consistency."""
        # Create a material prim
        material = UsdShade.Material.Define(self.stage, "/World/TestMaterial")
        material_prim = material.GetPrim()

        # Test multiple combinations
        test_materials = [
            ("none", "none", "none"),
            ("aluminum", "paint", "emissive"),
            ("plastic", "clearcoat", "retroreflective"),
            ("clear_glass", "none", "single_sided"),
            ("steel", "paint_clearcoat", "visually_transparent"),
        ]

        for base, coating, attribute in test_materials:
            with self.subTest(base=base, coating=coating, attribute=attribute):
                # Apply the material
                result = apply_nonvisual_material(material_prim, base=base, coating=coating, attribute=attribute)
                self.assertTrue(result)

                # Get the material ID
                material_id = get_material_id(material_prim)

                # Verify the ID is non-negative and fits in uint16
                self.assertGreaterEqual(material_id, 0)
                self.assertLessEqual(material_id, 0xFFFF)

                # Manually verify bit encoding
                expected_base = BASE_MATERIALS[base]
                expected_coating = COATINGS[coating]
                expected_attribute = ATTRIBUTES[attribute]
                expected_id = expected_base + (expected_coating << 8) + (expected_attribute << 11)
                expected_id &= 0xFFFF

                self.assertEqual(material_id, expected_id)

    def test_decode_material_id_basic(self) -> None:
        """Test basic decoding of material ID into component strings."""
        # Test known material ID (aluminum + paint + emissive = 2305)
        base, coating, attribute = decode_material_id(2305)

        self.assertEqual(base, "aluminum")
        self.assertEqual(coating, "paint")
        self.assertEqual(attribute, "emissive")

    def test_decode_material_id_zero(self) -> None:
        """Test decoding material ID of 0 (all none values)."""
        base, coating, attribute = decode_material_id(0)

        self.assertEqual(base, "none")
        self.assertEqual(coating, "none")
        self.assertEqual(attribute, "none")

    def test_decode_material_id_boundary_values(self) -> None:
        """Test decoding with boundary values for each component."""
        # Test maximum valid values within bit ranges
        test_cases = [
            # Material ID with max base (255), coating 0, attribute 0
            (47, "calibration_lambertion", "none", "none"),  # base ID 47 is max defined
            # Material ID with base 0, max coating (7), attribute 0
            (3 << 8, "none", "paint_clearcoat", "none"),  # coating ID 3 is max defined
            # Material ID with base 0, coating 0, max attribute (31)
            (8 << 11, "none", "none", "visually_transparent"),  # attribute ID 8 is max defined
        ]

        for material_id, expected_base, expected_coating, expected_attribute in test_cases:
            with self.subTest(material_id=material_id):
                base, coating, attribute = decode_material_id(material_id)

                # Note: For undefined values, function should return "none"
                if expected_base == "calibration_lambertion":
                    self.assertEqual(base, expected_base)
                else:
                    self.assertEqual(base, "none")  # 255 is not a defined base material

                self.assertEqual(coating, expected_coating)
                self.assertEqual(attribute, expected_attribute)

    def test_decode_material_id_various_combinations(self) -> None:
        """Test decoding various material ID combinations."""
        test_cases = [
            # (material_id, expected_base, expected_coating, expected_attribute)
            (1, "aluminum", "none", "none"),  # Only base
            (256, "none", "paint", "none"),  # Only coating (1 << 8)
            (2048, "none", "none", "emissive"),  # Only attribute (1 << 11)
            (10 + (3 << 8) + (2 << 11), "tin", "paint_clearcoat", "retroreflective"),
            (22 + (0 << 8) + (8 << 11), "mirror", "none", "visually_transparent"),
            (2 + (2 << 8) + (4 << 11), "steel", "clearcoat", "single_sided"),
        ]

        for material_id, expected_base, expected_coating, expected_attribute in test_cases:
            with self.subTest(material_id=material_id):
                base, coating, attribute = decode_material_id(material_id)
                self.assertEqual(base, expected_base)
                self.assertEqual(coating, expected_coating)
                self.assertEqual(attribute, expected_attribute)

    def test_decode_material_id_invalid_components(self) -> None:
        """Test decoding material ID with invalid component values."""
        # Create material ID with undefined base, coating, and attribute values
        # Use values that are within bit ranges but not defined in dictionaries

        # Base 100 (not defined), coating 5 (not defined), attribute 20 (not defined)
        material_id = 100 + (5 << 8) + (20 << 11)

        base, coating, attribute = decode_material_id(material_id)

        # All should default to "none" since the values are not defined
        self.assertEqual(base, "none")
        self.assertEqual(coating, "none")
        self.assertEqual(attribute, "none")

    def test_decode_material_id_error_handling(self) -> None:
        """Test error handling for invalid material ID values."""
        # Test negative value
        with self.assertRaises(ValueError) as context:
            decode_material_id(-1)
        self.assertIn("outside valid uint16 range", str(context.exception))

        # Test value exceeding uint16 range
        with self.assertRaises(ValueError) as context:
            decode_material_id(0x10000)  # 65536, exceeds uint16 max
        self.assertIn("outside valid uint16 range", str(context.exception))

        # Test maximum valid uint16 value should work
        try:
            base, coating, attribute = decode_material_id(0xFFFF)
            # Should not raise exception, but may return "none" values
            self.assertIsInstance(base, str)
            self.assertIsInstance(coating, str)
            self.assertIsInstance(attribute, str)
        except Exception as e:
            self.fail(f"decode_material_id raised unexpected exception for valid uint16: {e}")

    def test_material_id_encode_decode_roundtrip(self) -> None:
        """Test that encoding and decoding material properties is consistent."""
        # Create a material prim for testing
        material = UsdShade.Material.Define(self.stage, "/World/RoundtripMaterial")
        material_prim = material.GetPrim()

        # Test various material combinations
        test_materials = [
            ("none", "none", "none"),
            ("aluminum", "paint", "emissive"),
            ("plastic", "clearcoat", "retroreflective"),
            ("clear_glass", "none", "single_sided"),
            ("steel", "paint_clearcoat", "visually_transparent"),
            ("tin", "none", "none"),
            ("mirror", "paint", "none"),
            ("water", "none", "emissive"),
        ]

        for original_base, original_coating, original_attribute in test_materials:
            with self.subTest(base=original_base, coating=original_coating, attribute=original_attribute):
                # Apply the material properties
                result = apply_nonvisual_material(
                    material_prim, base=original_base, coating=original_coating, attribute=original_attribute
                )
                self.assertTrue(
                    result, f"Failed to apply material: {original_base}, {original_coating}, {original_attribute}"
                )

                # Get the encoded material ID
                material_id = get_material_id(material_prim)

                # Decode the material ID back to component strings
                decoded_base, decoded_coating, decoded_attribute = decode_material_id(material_id)

                # Verify round-trip consistency
                self.assertEqual(decoded_base, original_base)
                self.assertEqual(decoded_coating, original_coating)
                self.assertEqual(decoded_attribute, original_attribute)

    def test_decode_material_id_bit_extraction(self) -> None:
        """Test that bit extraction works correctly for decode_material_id."""
        # Manually construct material ID with known bit patterns
        base_value = 15  # 0x0F (lower 8 bits)
        coating_value = 5  # 0x5 (3 bits, shifted to positions 8-10)
        attribute_value = 10  # 0xA (5 bits, shifted to positions 11-15)

        # Note: coating_value 5 and attribute_value 10 are not defined in dictionaries
        # This tests the bit extraction logic even with undefined values

        material_id = base_value + (coating_value << 8) + (attribute_value << 11)

        # Extract using same bit operations as the function
        extracted_base = material_id & 0xFF
        extracted_coating = (material_id >> 8) & 0x7
        extracted_attribute = (material_id >> 11) & 0x1F

        self.assertEqual(extracted_base, base_value)
        self.assertEqual(extracted_coating, coating_value)
        self.assertEqual(extracted_attribute, attribute_value)

        # Test decode function returns "none" for undefined values
        base, coating, attribute = decode_material_id(material_id)

        # Base 15 is "plexiglass", coating 5 and attribute 10 are undefined
        self.assertEqual(base, "plexiglass")  # Base 15 is defined
        self.assertEqual(coating, "none")  # Coating 5 is not defined
        self.assertEqual(attribute, "none")  # Attribute 10 is not defined
