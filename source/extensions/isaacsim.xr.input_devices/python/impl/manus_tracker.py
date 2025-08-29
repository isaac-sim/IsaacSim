# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import carb
from typing import Dict

try:
    from isaacsim.xr.input_devices._isaac_xr_input_devices import IsaacSimManusTracker
    _manus_tracker_available = True
except ImportError:
    carb.log_warn("IsaacSimManusTracker not available - using mock data for manus")
    _manus_tracker_available = False

class IsaacSimManusGloveTracker:
    def __init__(self):
        self.glove_data = {}
        self.is_connected = False
        
        if _manus_tracker_available:
            try:
                self._manus_tracker = IsaacSimManusTracker()
                success = self._manus_tracker.initialize()
                if success:
                    self.is_connected = True
                    carb.log_info("Manus glove tracker initialized with SDK")
                else:
                    carb.log_warn("Failed to initialize Manus SDK - using mock data")
                    self._manus_tracker = None
            except Exception as e:
                carb.log_warn(f"Failed to initialize Manus tracker: {e} - using mock data")
                self._manus_tracker = None
        else:
            self._manus_tracker = None
            carb.log_info("Manus glove tracker initialized (mock)")
        
    def update(self):
        if self._manus_tracker and self.is_connected:
            try:
                raw_data = self._manus_tracker.get_glove_data()
                for hand in ['left', 'right']:
                    self._populate_glove_data(raw_data, hand)
                
            except Exception as e:
                carb.log_error(f"Failed to update Manus glove data: {e}")
        else:
            # Provide mock data
            self.glove_data = {
                'left_0': {
                    'position': [0.0, 0.0, 0.0],
                    'orientation': [1.0, 0.0, 0.0, 0.0]
                },
                'right_0': {
                    'position': [0.1, 0.0, 0.0],
                    'orientation': [1.0, 0.0, 0.0, 0.0]
                }
            }
    
    def _populate_glove_data(self, raw_data: Dict, hand: str) -> None:
        """
        Convert raw Manus data to Isaac Sim format with individual joint entries.
        Adds to self.glove_data: {joint_name: {position: pos, orientation: ori}}
        """
        if f'{hand}_position' not in raw_data or f'{hand}_orientation' not in raw_data:
            return
        
        position = raw_data[f'{hand}_position']
        orientation = raw_data[f'{hand}_orientation']
        joint_count = len(position) // 3  # 3 values per position
            
        for joint_idx in range(joint_count):
            joint_name = f"{hand}_{joint_idx}"
            self.glove_data[joint_name] = {
                'position': [float(position[i]) for i in range(joint_idx * 3, joint_idx * 3 + 3)],
                'orientation': [float(orientation[i]) for i in range(joint_idx * 4, joint_idx * 4 + 4)]
            }
    
    def get_all_glove_data(self) -> Dict:
        return self.glove_data
    
    def cleanup(self):
        try:
            if self._manus_tracker:
                self._manus_tracker.cleanup()
            self.is_connected = False
        except Exception as e:
            carb.log_error(f"Error during Manus tracker cleanup: {e}")
