# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import sys
import carb
import numpy as np
from typing import Dict, List, Optional
from pxr import Gf
from omni.kit.xr.core import XRCore

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

# Hardcoded HMD pose matrix from XRCore (logged from head_device.get_virtual_world_pose(""))
hmd_pose = ((0.996499240398407, -0.06362161040306091, 0.054237786680459976, 0),
            (-0.06642970442771912, -0.20866860449314117, 0.9757277965545654, 0), 
            (-0.05075964331626892, -0.9759148359298706, -0.21216443181037903, 0), 
            (0.000521781446877867, 0.19946864247322083, 1.4816499948501587, 1))

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
                
                # Transform Vive coordinates to Isaac Sim coordinate system
                transformed_pos, transformed_ori = self._transform_vive_coordinates(pos, rot)
                
                # Capture ALL devices like isaac-deploy does (no filtering)
                self.device_data[device_id] = {
                    'position': transformed_pos,
                    'orientation': transformed_ori
                }

        except Exception as e:
            carb.log_error(f"Failed to update Vive tracker data: {e}")
    
    
    def _transform_vive_coordinates(self, position: np.ndarray, orientation: np.ndarray) -> tuple:
        """
        Transform Vive tracker coordinates to Isaac Sim coordinate system.
        Attempts to match the C++ OpenXR approach using available XRCore APIs.
        """
        # Create transformation matrix from Vive pose
        vive_quat = Gf.Quatd(orientation[0], Gf.Vec3d(orientation[1], orientation[2], orientation[3]))
        vive_rot = Gf.Matrix3d().SetRotate(vive_quat)
        vive_trans = Gf.Vec3d(position[0], position[1], position[2])
        vive_matrix = Gf.Matrix4d(vive_rot, vive_trans)
        
        # Apply basic transformations for Vive trackers
        # Vive uses Y-up like Isaac Sim, so minimal transformation needed
        scale_transform = Gf.Matrix4d().SetScale(1.0)  # Adjust scale if needed
        offset_transform = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))  # Adjust offset if needed
        
        # Apply the transformations
        transformed_matrix = vive_matrix * scale_transform * offset_transform
        
        # Use HMD pose to align with the same coordinate system as OpenXR hand tracking
        # The HMD pose represents where the stage coordinate system is relative to our devices
        hmd_transform = Gf.Matrix4d(hmd_pose)
        
        # Apply the same transformation that OpenXR.cpp applies internally
        # This aligns our external devices with the stage coordinate system
        transformed_matrix = transformed_matrix * hmd_transform
        
        # Extract transformed position and orientation
        transformed_pos = transformed_matrix.ExtractTranslation()
        transformed_rot = transformed_matrix.ExtractRotation()
        transformed_quat = transformed_rot.GetQuat()
        
        return (
            [transformed_pos[0], transformed_pos[1], transformed_pos[2]],
            [transformed_quat.GetReal(), transformed_quat.GetImaginary()[0], 
             transformed_quat.GetImaginary()[1], transformed_quat.GetImaginary()[2]]
        )
    
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