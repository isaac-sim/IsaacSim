# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import sys
import carb
import os
from typing import Dict
from pxr import Gf

try:
    if 'pysurvive' not in sys.modules:
        ext_root = os.path.abspath(os.path.join(__file__, "../../../../.."))
        vendor_path = os.path.join(ext_root, 'pysurvive')
        if os.path.isdir(vendor_path) and vendor_path not in sys.path:
            sys.path.insert(0, vendor_path)
            carb.log_info(f"Using vendored pysurvive at {vendor_path}")
        else:
            carb.log_warn(f"Failed to add vendored pysurvive: {vendor_path}")

    import pysurvive
    from pysurvive.pysurvive_generated import survive_simple_close
    PYSURVIVE_AVAILABLE = True
    carb.log_info("pysurvive imported successfully")
except ImportError as e:
    carb.log_error(f"pysurvive not available")
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
            raise RuntimeError("pysurvive not available")
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
    
    def get_data(self) -> Dict:
        return self.device_data
    
    def cleanup(self):
        try:
            if PYSURVIVE_AVAILABLE and hasattr(self, '_ctx') and self._ctx is not None:
                carb.log_info("Cleaning up Vive tracker context")
                survive_simple_close(self._ctx.ptr)
            self.is_connected = False
        except Exception as e:
            carb.log_error(f"Error during Vive tracker cleanup: {e}")
