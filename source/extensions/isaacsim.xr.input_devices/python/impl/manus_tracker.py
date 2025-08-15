# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import carb
import numpy as np
from typing import Dict, List, Optional

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
                self.glove_data = self._convert_to_isaacsim_format(raw_data)
            except Exception as e:
                carb.log_error(f"Failed to update Manus glove data: {e}")
        else:
            # Provide mock data
            self.glove_data = {
                'left_glove': {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                    'valid': True
                },
                'right_glove': {
                    'position': np.array([0.1, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                    'valid': True
                }
            }
    
    def _convert_to_isaacsim_format(self, raw_data: Dict) -> Dict:
        isaacsim_data = {}
        
        if 'left_position' in raw_data and 'left_orientation' in raw_data:
            isaacsim_data['left_glove'] = {
                'position': np.array(raw_data['left_position']),
                'orientation': np.array(raw_data['left_orientation']),
                'valid': True
            }
        
        if 'right_position' in raw_data and 'right_orientation' in raw_data:
            isaacsim_data['right_glove'] = {
                'position': np.array(raw_data['right_position']),
                'orientation': np.array(raw_data['right_orientation']),
                'valid': True
            }
        
        return isaacsim_data
    
    def get_glove_pose(self, hand: str) -> Optional[Dict]:
        glove_key = f'{hand}_glove'
        if glove_key in self.glove_data:
            return self.glove_data[glove_key]
        return None
    
    def get_all_glove_data(self) -> Dict:
        return self.glove_data.copy()
    
    def cleanup(self):
        try:
            if self._manus_tracker:
                self._manus_tracker.cleanup()
            self.is_connected = False
        except Exception as e:
            carb.log_error(f"Error during Manus tracker cleanup: {e}") 