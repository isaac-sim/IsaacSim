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

"""Nonvisual materials implementation for RTX sensors.

This module provides functionality for applying and retrieving nonvisual material properties
used by RTX sensors like LiDAR and radar.

Based on documentation from omni.sensors.nv.material extension:
https://docs.isaacsim.omniverse.nvidia.com/4.5.0/sensors/omni_sensors_docs/materials_extension/materials_extension.html#current-materials
"""

import carb
from pxr import Sdf, Usd, UsdShade

# Base material dictionaries organized by category as specified in documentation

# Base Materials - None/Default
NONE_BASE = {
    "none": 0,
}

# Metals base materials
METALS_BASE = {
    "aluminum": 1,
    "steel": 2,
    "oxidized_steel": 3,
    "iron": 4,
    "oxidized_iron": 5,
    "silver": 6,
    "brass": 7,
    "bronze": 8,
    "oxidized_bronze_patina": 9,
    "tin": 10,
}

# Polymers base materials
POLYMERS_BASE = {
    "plastic": 11,
    "fiberglass": 12,
    "carbon_fiber": 13,
    "vinyl": 14,
    "plexiglass": 15,
    "pvc": 16,
    "nylon": 17,
    "polyester": 18,
}

# Glass base materials
GLASS_BASE = {
    "clear_glass": 19,
    "frosted_glass": 20,
    "one_way_mirror": 21,
    "mirror": 22,
    "ceramic_glass": 23,
}

# Other base materials
OTHER_BASE = {
    "asphalt": 24,
    "concrete": 25,
    "leaf_grass": 26,
    "dead_leaf_grass": 27,
    "rubber": 28,
    "wood": 29,
    "bark": 30,
    "cardboard": 31,
    "paper": 32,
    "fabric": 33,
    "skin": 34,
    "fur_hair": 35,
    "leather": 36,
    "marble": 37,
    "brick": 38,
    "stone": 39,
    "gravel": 40,
    "dirt": 41,
    "mud": 42,
    "water": 43,
    "salt_water": 44,
    "snow": 45,
    "ice": 46,
    "calibration_lambertion": 47,
}

# Combined base materials dictionary
BASE_MATERIALS = {
    **NONE_BASE,
    **METALS_BASE,
    **POLYMERS_BASE,
    **GLASS_BASE,
    **OTHER_BASE,
}

# Coating materials (3-bit encoding, values 0-7)
COATINGS = {
    "none": 0,
    "paint": 1,
    "clearcoat": 2,
    "paint_clearcoat": 3,
}

# Material attributes (5-bit encoding, values 0-31)
ATTRIBUTES = {
    "none": 0,
    "emissive": 1,
    "retroreflective": 2,
    "single_sided": 4,
    "visually_transparent": 8,
}


# Attribute name constants
ATTR_PREFIX = carb.settings.get_settings().get("/rtx/materialDb/nonVisualMaterialSemantics/prefix")
ATTR_BASE = f"{ATTR_PREFIX}:base"
ATTR_COATING = f"{ATTR_PREFIX}:coating"
ATTR_ATTRIBUTE = f"{ATTR_PREFIX}:attributes"


def apply_nonvisual_material(
    prim: Usd.Prim, base: str | int, coating: str | int = "none", attribute: str | int = "none"
) -> bool:
    """Apply nonvisual material attributes to a USD material prim.

    This function applies nonvisual material properties (base, coating, attribute) to a USD
    material prim for use by RTX sensors. The material must be a valid material prim.

    Args:
        prim: USD prim that must be a material prim.
        base: Base material type, either string name or integer ID.
        coating: Coating type, either string name or integer ID.
        attribute: Material attribute, either string name or integer ID.

    Returns:
        True if the nonvisual material was successfully applied, False otherwise.
        Error messages are logged via carb.log_error for debugging.

    Example:

    .. code-block:: python

        >>> import omni.usd
        >>> from pxr import UsdShade
        >>> from isaacsim.sensors.rtx.nonvisual_materials import apply_nonvisual_material
        >>>
        >>> stage = omni.usd.get_context().get_stage()
        >>> material = UsdShade.Material.Define(stage, "/World/MyMaterial")
        >>> success = apply_nonvisual_material(
        ...     material.GetPrim(),
        ...     "aluminum",
        ...     "paint",
        ...     "emissive"
        ... )
        >>> success
        True
    """
    if not prim or not prim.IsValid():
        carb.log_error("Provided prim is not a valid USD prim")
        return False

    # Verify the prim is a material
    if not prim.IsA(UsdShade.Material):
        carb.log_error("Provided prim is not a material prim")
        return False

    # Convert inputs to string names for storage
    if isinstance(base, str):
        if base not in BASE_MATERIALS:
            carb.log_error(f"Base material '{base}' not found in base materials dictionary")
            return False
        base_name = base
    else:
        # Reverse lookup: find string name from numeric ID
        base_name = None
        for name, id_val in BASE_MATERIALS.items():
            if id_val == base:
                base_name = name
                break
        if base_name is None:
            carb.log_error(f"Base material ID '{base}' not found in base materials dictionary")
            return False

    if isinstance(coating, str):
        if coating not in COATINGS:
            carb.log_error(f"Coating '{coating}' not found in coatings dictionary")
            return False
        coating_name = coating
    else:
        # Reverse lookup: find string name from numeric ID
        coating_name = None
        for name, id_val in COATINGS.items():
            if id_val == coating:
                coating_name = name
                break
        if coating_name is None:
            carb.log_error(f"Coating ID '{coating}' not found in coatings dictionary")
            return False

    if isinstance(attribute, str):
        if attribute not in ATTRIBUTES:
            carb.log_error(f"Attribute '{attribute}' not found in attributes dictionary")
            return False
        attribute_name = attribute
    else:
        # Reverse lookup: find string name from numeric ID
        attribute_name = None
        for name, id_val in ATTRIBUTES.items():
            if id_val == attribute:
                attribute_name = name
                break
        if attribute_name is None:
            carb.log_error(f"Attribute ID '{attribute}' not found in attributes dictionary")
            return False

    # Apply the nonvisual material attributes as described in the documentation
    try:
        # Set the base material attribute
        base_attr = prim.CreateAttribute(ATTR_BASE, Sdf.ValueTypeNames.String, custom=True)
        base_attr.Set(base_name)

        # Set the coating attribute
        coating_attr = prim.CreateAttribute(ATTR_COATING, Sdf.ValueTypeNames.String, custom=True)
        coating_attr.Set(coating_name)

        # Set the attribute attribute
        attr_attr = prim.CreateAttribute(ATTR_ATTRIBUTE, Sdf.ValueTypeNames.String, custom=True)
        attr_attr.Set(attribute_name)

        return True

    except Exception as e:
        carb.log_error(f"Error applying nonvisual material attributes: {e}")
        return False


def get_material_id(prim: Usd.Prim) -> int:
    """Get material ID from nonvisual material attributes on a USD material prim.

    This function retrieves the nonvisual material attributes from a USD material prim
    and computes the material ID based on the base, coating, and attribute values.
    The material ID is a uint16 with bit encoding as follows:
    - Lower byte (bits 0-7): base material index
    - Upper byte bits 8-10 (lower 3 bits): coatings (values 0-7)
    - Upper byte bits 11-15 (upper 5 bits): attributes (values 0-31)

    Args:
        prim: USD prim that must be a material prim with nonvisual material attributes.

    Returns:
        Computed material ID as an integer. Returns 0 if attributes are not found
        or if prim is invalid.

    Note:
        Error messages are logged via carb.log_error for debugging.

    Example:

    .. code-block:: python

        >>> import omni.usd
        >>> from pxr import UsdShade
        >>> from isaacsim.sensors.rtx.nonvisual_materials import apply_nonvisual_material, get_material_id
        >>>
        >>> stage = omni.usd.get_context().get_stage()
        >>> material = UsdShade.Material.Define(stage, "/World/MyMaterial")
        >>> apply_nonvisual_material(material.GetPrim(), "aluminum", "paint", "emissive")
        True
        >>> material_id = get_material_id(material.GetPrim())
        >>> material_id
        2305
    """
    if not prim or not prim.IsValid():
        carb.log_error("Provided prim is not a valid USD prim")
        return 0

    # Verify the prim is a material
    if not prim.IsA(UsdShade.Material):
        carb.log_error("Provided prim is not a material prim")
        return 0

    # Default values
    base_value = 0
    coating_value = 0
    attribute_value = 0

    # Get base material attribute
    if prim.HasAttribute(ATTR_BASE):
        base_str = prim.GetAttribute(ATTR_BASE).Get()
        if base_str and base_str in BASE_MATERIALS:
            base_value = BASE_MATERIALS[base_str]

    # Get coating attribute
    if prim.HasAttribute(ATTR_COATING):
        coating_str = prim.GetAttribute(ATTR_COATING).Get()
        if coating_str and coating_str in COATINGS:
            coating_value = COATINGS[coating_str]

    # Get attribute attribute
    if prim.HasAttribute(ATTR_ATTRIBUTE):
        attr_str = prim.GetAttribute(ATTR_ATTRIBUTE).Get()
        if attr_str and attr_str in ATTRIBUTES:
            attribute_value = ATTRIBUTES[attr_str]

    # Ensure values fit within their bit ranges
    base_value = base_value & 0xFF  # 8 bits (0-255)
    coating_value = coating_value & 0x7  # 3 bits (0-7)
    attribute_value = attribute_value & 0x1F  # 5 bits (0-31)

    # Compute material ID using bit encoding:
    # Lower byte: base material (bits 0-7)
    # Upper byte: coating (bits 8-10) + attribute (bits 11-15)
    material_id = base_value + (coating_value << 8) + (attribute_value << 11)

    # Ensure result fits in uint16
    material_id = material_id & 0xFFFF

    return material_id


def decode_material_id(material_id: int) -> tuple[str, str, str]:
    """Decode material ID back into base, coating, and attribute string names.

    This function takes a material ID (uint16) and decodes it back into the original
    base material, coating, and attribute string names based on the bit encoding:
    - Lower byte (bits 0-7): base material index
    - Upper byte bits 8-10 (lower 3 bits): coatings (values 0-7)
    - Upper byte bits 11-15 (upper 5 bits): attributes (values 0-31)

    Args:
        material_id: Material ID as computed by get_material_id function.

    Returns:
        Tuple containing (base_name, coating_name, attribute_name) as strings.
        Returns ("none", "none", "none") if any component cannot be decoded.

    Raises:
        ValueError: If material_id is negative or exceeds uint16 range.

    Example:

    .. code-block:: python

        >>> from isaacsim.sensors.rtx.nonvisual_materials import decode_material_id
        >>>
        >>> # Decode a material ID back to string names
        >>> base, coating, attribute = decode_material_id(2305)
        >>> base
        'aluminum'
        >>> coating
        'paint'
        >>> attribute
        'emissive'
    """
    if material_id < 0 or material_id > 0xFFFF:
        raise ValueError(f"Material ID {material_id} is outside valid uint16 range (0-65535)")

    # Extract components using bit operations (reverse of encoding)
    base_value = material_id & 0xFF  # Lower 8 bits
    coating_value = (material_id >> 8) & 0x7  # Bits 8-10
    attribute_value = (material_id >> 11) & 0x1F  # Bits 11-15

    # Reverse lookup to get string names
    base_name = "none"
    for name, id_val in BASE_MATERIALS.items():
        if id_val == base_value:
            base_name = name
            break

    coating_name = "none"
    for name, id_val in COATINGS.items():
        if id_val == coating_value:
            coating_name = name
            break

    attribute_name = "none"
    for name, id_val in ATTRIBUTES.items():
        if id_val == attribute_value:
            attribute_name = name
            break

    return (base_name, coating_name, attribute_name)
