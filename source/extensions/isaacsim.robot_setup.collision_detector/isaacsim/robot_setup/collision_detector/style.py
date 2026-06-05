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

"""UI style definitions for the Robot Self-Collision Detector extension."""

import omni.ui as ui
from omni.ui import color as cl

STYLES = {
    "TreeView": {
        "background_color": 0xFF454545,
        "secondary_color": cl.transparent,
        "border_radius": 0,
        "padding": 0,
        "font_size": 14,
        "font": "NVIDIASans_Rg.ttf",
    },
    "ScrollingFrame": {"background_color": 0xFF23211F},
    "TreeView.Item:hovered": {"color": 0xFFBBBAAA},
    "TreeView.Item:selected": {"color": cl.red},
    "TreeView:selected": {"background_color": 0x66888777},
    "TreeView.Item": {
        "margin": 0,
    },
    "Button": {
        "stack_direction": ui.Direction.LEFT_TO_RIGHT,
    },
    "Button.Image:disabled": {
        "color": 0xFF555555,
    },
    "Button.Label:disabled": {
        "color": 0xFF555555,
    },
    "Rectangle::button_background": {
        "background_color": 0xFF1C1C1C,
    },
    "Image::check_collisions": {
        "image_url": "${isaacsim.robot_setup.collision_detector}/icons/refresh.svg",
        "color": 0xFFAAAAAA,
    },
    "Image::focal": {
        "image_url": "${isaacsim.robot_setup.collision_detector}/icons/focal.svg",
        "color": 0xFF666666,
    },
    "Image::focal:hovered": {
        "color": cl.white,
    },
    "Image::focal:pressed": {
        "color": cl.white,
    },
    "Image::focal:selected": {
        "color": cl.white,
    },
    "Image::sort": {
        "image_url": "${isaacsim.robot_setup.collision_detector}/icons/sort.svg",
        "color": 0xFFAAAAAA,
    },
    "Image::sort:hovered": {
        "color": cl.white,
    },
    "Image::sort_up": {
        "image_url": "${isaacsim.robot_setup.collision_detector}/icons/sort_up.svg",
        "color": 0xFFAAAAAA,
    },
    "Image::sort_up:hovered": {
        "color": cl.white,
    },
    "Image::sort_down": {
        "image_url": "${isaacsim.robot_setup.collision_detector}/icons/sort_down.svg",
        "color": 0xFFAAAAAA,
    },
    "Image::sort_down:hovered": {
        "color": cl.white,
    },
    "Button::select_collision_prim": {
        "background_color": cl.transparent,
        "border_width": 0,
        "font_size": 14,
    },
    "Rectangle::collision_body_icon_0": {
        "background_color": 0xFF007AFF,
        "border_radius": 2,
    },
    "Rectangle::collision_body_icon_1": {
        "background_color": 0xFFB4770D,
        "border_radius": 2,
    },
    "Rectangle::focal_background": {
        "background_color": cl.transparent,
        "border_width": 0.5,
        "border_color": 0xFF454545,
    },
    "Rectangle::focal_parent_background": {
        "background_color": 0xFF2D2D2D,
        "border_width": 0.5,
        "border_color": 0xFF454545,
    },
    "ComboBox::robot_selector": {
        "background_color": 0xFF23211F,
        "secondary_color": 0xFF23211F,
        "color": 0xFFCCCCCC,
        "font_size": 14,
        "border_radius": 4,
    },
    "Label": {
        "color": 0xFF8A8777,
        "font_size": 12,
    },
    "Label:disabled": {
        "color": 0xFF555555,
    },
}
