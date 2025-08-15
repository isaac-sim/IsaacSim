# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import sys
import carb
import numpy as np
from typing import Dict, List, Optional

try:
    # Add libsurvive pysurvive bindings to Python path
    libsurvive_python_path = "/home/yuanchenl/libsurvive/bindings/python"  ###FIXME: hardcoded path
    if libsurvive_python_path not in sys.path:
        sys.path.insert(0, libsurvive_python_path)
    
    import pysurvive
    from pysurvive.pysurvive_generated import survive_simple_close
    PYSURVIVE_AVAILABLE = True
    carb.log_info("pysurvive imported successfully from libsurvive source")
except ImportError as e:
    carb.log_warn(f"pysurvive not available - using mock data for vive: {e}")
    PYSURVIVE_AVAILABLE = False

class IsaacSimViveTracker:
    def __init__(self):
        self.device_data = {}
        self.is_connected = False
        
        if PYSURVIVE_AVAILABLE:
            try:
                self._ctx = pysurvive.SimpleContext([sys.argv[0]])
                if self._ctx is None:
                    raise RuntimeError('Failed to initialize Survive context.')
                self.is_connected = True
                carb.log_info("Vive tracker initialized with pysurvive")
            except Exception as e:
                carb.log_warn(f"Failed to initialize Vive tracker: {e}")
        else:
            self._ctx = None
            self.is_connected = True
            carb.log_info("Vive tracker initialized (mock)")
    
    def update(self):
        if not PYSURVIVE_AVAILABLE:
            self.device_data = {
                'tracker_1_position': [0.0, 0.0, 0.0],
                'tracker_1_orientation': [1.0, 0.0, 0.0, 0.0],
                'tracker_2_position': [0.1, 0.0, 0.0],
                'tracker_2_orientation': [1.0, 0.0, 0.0, 0.0]
            }
            return
        if not self.is_connected:
            return
            
        try:
            output_map: Dict[str, List[float]] = {}
            
            max_iterations = 1000  # Prevent infinite loops
            iteration = 0
            while iteration < max_iterations:
                updated = self._ctx.NextUpdated()
                if not updated:
                    break
                iteration += 1

                pose_obj, _ = updated.Pose()
                pos = pose_obj.Pos  # (x, y, z)
                rot = pose_obj.Rot  # (w, x, y, z)

                device_id = updated.Name().decode('utf-8')
                
                # Capture ALL devices like isaac-deploy does (no filtering)
                output_map[f'{device_id}_position'] = [pos[0], pos[1], pos[2]]
                output_map[f'{device_id}_orientation'] = [rot[0], rot[1], rot[2], rot[3]]
                carb.log_info(f"Detected device {device_id}: pos={pos}, rot={rot}")

            if output_map:
                self.device_data = output_map
                carb.log_info(f"Updated Vive tracker data: {list(output_map.keys())}")
            
        except Exception as e:
            carb.log_error(f"Failed to update Vive tracker data: {e}")
    
    def get_tracker_pose(self, device_id: str) -> Optional[Dict]:
        position_key = f'{device_id}_position'
        orientation_key = f'{device_id}_orientation'
        
        if position_key in self.device_data and orientation_key in self.device_data:
            return {
                'position': np.array(self.device_data[position_key]),
                'orientation': np.array(self.device_data[orientation_key]),
                'valid': True
            }
        return None
    
    def get_all_tracker_data(self) -> Dict:
        return self.device_data.copy()
    
    def cleanup(self):
        try:
            if PYSURVIVE_AVAILABLE and hasattr(self, '_ctx') and self._ctx is not None:
                carb.log_info("Cleaning up Vive tracker context")
                survive_simple_close(self._ctx.ptr)
            self.is_connected = False
        except Exception as e:
            carb.log_error(f"Error during Vive tracker cleanup: {e}") 