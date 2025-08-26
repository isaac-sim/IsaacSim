# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import sys
import carb
from typing import Dict
from pxr import Gf

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
                'tracker_1': {
                    'position': [0.0, 0.0, 0.0],
                    'orientation': [1.0, 0.0, 0.0, 0.0]
                },
                'tracker_2': {
                    'position': [0.1, 0.0, 0.0],
                    'orientation': [1.0, 0.0, 0.0, 0.0]
                }
            }
            return
        if not self.is_connected:
            return
            
        try:
            max_iterations = 10  # Prevent infinite loops
            iteration = 0
            while iteration < max_iterations:
                updated = self._ctx.NextUpdated()
                if not updated:
                    break
                iteration += 1
                
                pose_obj, _ = updated.Pose()
                pos = pose_obj.Pos  # (x, y, z)
                ori = pose_obj.Rot  # (w, x, y, z)
                device_id = updated.Name().decode('utf-8')

                self.device_data[device_id] = {
                    'position': [float(pos[0]), float(-pos[2]), float(pos[1])], # x, z, -y
                    'orientation': [float(ori[0]), float(ori[1]), float(-ori[3]), float(ori[2])]
                }

        except Exception as e:
            carb.log_error(f"Failed to update Vive tracker data: {e}")
    
    def get_all_tracker_data(self) -> Dict:
        return self.device_data
    
    def cleanup(self):
        try:
            if PYSURVIVE_AVAILABLE and hasattr(self, '_ctx') and self._ctx is not None:
                carb.log_info("Cleaning up Vive tracker context")
                survive_simple_close(self._ctx.ptr)
            self.is_connected = False
        except Exception as e:
            carb.log_error(f"Error during Vive tracker cleanup: {e}") 