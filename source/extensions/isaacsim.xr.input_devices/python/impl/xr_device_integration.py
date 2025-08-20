# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import carb
import time
import numpy as np
from typing import Dict
from pxr import Gf
from .manus_tracker import IsaacSimManusGloveTracker
from .vive_tracker import IsaacSimViveTracker

# Left wrist pose from AVP hand tracking
scene_T_wrist_static = Gf.Matrix4d(
    (0.9534056782722473, -0.30112600326538086, -0.018459070473909378, 0), 
    (-0.104372538626194, -0.38662755489349365, 0.9163107872009277, 0), 
    (-0.28306180238723755, -0.8716893196105957, -0.40004217624664307, 0), 
    (-0.1019858792424202, 0.3140803277492523, 1.285851240158081, 1)
)
scene_T_wrist_static = Gf.Matrix4d().SetIdentity()

def get_xr_device_integration():
    """Get the existing XRDeviceIntegration instance from the extension."""
    try:
        import omni.ext
        from .extension import Extension
        ext_instance = Extension.get_instance()
        if ext_instance and hasattr(ext_instance, 'xr_integration'):
            return ext_instance.xr_integration
        else:
            carb.log_warn("Extension not loaded, creating new XRDeviceIntegration instance")
            return XRDeviceIntegration()
    except Exception as e:
        carb.log_warn(f"Failed to get extension instance: {e}, creating new XRDeviceIntegration")
        return XRDeviceIntegration()

class XRDeviceIntegration:
    def __init__(self):
        self.manus_tracker = IsaacSimManusGloveTracker()
        self.vive_tracker = IsaacSimViveTracker()
        self.last_update_time = 0
        self.update_rate = 100.0
        self.device_status = {
            'manus_gloves': {'connected': False, 'last_data_time': 0},
            'vive_trackers': {'connected': False, 'last_data_time': 0}
        }
        self.scene_T_lighthouse_static = None  # Coordinate transformation matrix, will be computed from first Vive wrist position
        
    def register_devices(self):
        try:
            if self.manus_tracker.is_connected:
                carb.log_info("Manus gloves registered successfully")
                self.device_status['manus_gloves']['connected'] = True
            else:
                carb.log_warn("Failed to initialize Manus gloves")
            
            if self.vive_tracker.is_connected:
                carb.log_info("Vive trackers registered successfully")
                self.device_status['vive_trackers']['connected'] = True
            else:
                carb.log_warn("Failed to initialize Vive trackers")
                
        except Exception as e:
            carb.log_error(f"Failed to register XR devices: {e}")
    
    def update_manus(self):
        """Update raw Manus glove data. """
        current_time = time.time()
        self.manus_tracker.update()
        self.device_status['manus_gloves']['last_data_time'] = current_time
    
    def update_vive(self):
        """Update raw Vive tracker data, and initialize coordinate transformation if it is the first data update. """
        current_time = time.time()
        self.vive_tracker.update()
        self.device_status['vive_trackers']['last_data_time'] = current_time
        try:
            # Initialize coordinate transformation from first Vive wrist position
            if self.scene_T_lighthouse_static is None:
                self._initialize_coordinate_transformation()
        except Exception as e:
            carb.log_error(f"Vive tracker update failed: {e}")

    def update_all_devices(self):
        current_time = time.time()
        
        if current_time - self.last_update_time >= 1.0 / self.update_rate:
            try:
                self.update_manus()
                self.update_vive()
                self.last_update_time = current_time
                
            except Exception as e:
                carb.log_error(f"Failed to update XR devices: {e}")

    def pose_to_matrix(self, pose: Dict) -> Gf.Matrix4d:
        """Convert position and orientation to a 4x4 matrix."""
        pos, ori = pose['position'], pose['orientation']
        quat = Gf.Quatd(ori[0], Gf.Vec3d(ori[1], ori[2], ori[3]))
        rot = Gf.Matrix3d().SetRotate(quat)
        trans = Gf.Vec3d(pos[0], pos[1], pos[2])
        return Gf.Matrix4d(rot, trans)
    
    def matrix_to_pose(self, matrix: Gf.Matrix4d) -> tuple:
        """Convert a 4x4 matrix to position and orientation."""
        pos = matrix.ExtractTranslation()
        rot = matrix.ExtractRotation()
        quat = rot.GetQuat()
        return {
            'position': [pos[0], pos[1], pos[2]],
            'orientation': [quat.GetReal(), quat.GetImaginary()[0], quat.GetImaginary()[1], quat.GetImaginary()[2]]
        }
    
    def _initialize_coordinate_transformation(self):
        """
        Initialize the scene to lighthouse coordinate transformation.
        The coordinate transformation is used to transform the wrist pose from lighthouse coordinate system to isaac sim scene coordinate.
        It is computed from a hard-coded AVP wrist pose and the first Vive wrist pose. 
        """
        vive_data = self.vive_tracker.get_all_tracker_data()
        # Look for a wrist tracker in the Vive data
        left_wrist = 'WM0'  # FIXME: hardcoded - assuming that WM0 is the left wrist tracker
        list_trackers = []
        if left_wrist not in vive_data:
            for device_key in vive_data.keys():
                list_trackers.append(device_key)
                if len(device_key) >= 2 and device_key[:2] == 'WM':
                    left_wrist = device_key
                    carb.log_warn(f"Using Vive tracker '{left_wrist}' for coordinate transformation")
                    break
        
        if left_wrist not in vive_data:
            carb.log_warn(f"No Vive wrist tracker found for coordinate transformation. Trackers: {list_trackers}")
            return

        try:
            lighthouse_T_wrist = self.pose_to_matrix(vive_data[left_wrist])
            wrist_T_lighthouse_static = lighthouse_T_wrist.GetInverse()
            self.scene_T_lighthouse_static = wrist_T_lighthouse_static * scene_T_wrist_static
                
        except Exception as e:
            carb.log_error(f"Failed to initialize coordinate transformation: {e}")
    
    def _transform_to_scene_coordinates(self, wrist_data: Dict) -> Dict:
        """Transform vive wrist coordinates to scene coordinates using scene_T_lighthouse_static."""
        if self.scene_T_lighthouse_static is None:
            return wrist_data
            
        try:
            lighthouse_T_wrist = self.pose_to_matrix(wrist_data)
            transformed_matrix = lighthouse_T_wrist * self.scene_T_lighthouse_static
            return self.matrix_to_pose(transformed_matrix)

        except Exception as e:
            carb.log_error(f"Failed to transform coordinates: {e}")
            return wrist_data
    
    def get_all_device_data(self) -> Dict:
        # Get raw data from trackers
        manus_data = self.manus_tracker.get_all_glove_data()
        vive_data = self.vive_tracker.get_all_tracker_data()
        vive_transformed = self._transform_vive_data(vive_data)
        scene_T_wrist = self._get_scene_T_wrist(vive_transformed)
        
        return {
            'manus_gloves': self._transform_manus_data(manus_data, scene_T_wrist),
            'vive_trackers': vive_transformed,
            'device_status': self.device_status
        }
    
    def _transform_vive_data(self, device_data: Dict) -> Dict:
        """Transform all joints in vive data to scene coordinates. """
        if self.scene_T_lighthouse_static is None:
            return device_data
        
        transformed_data = {}
        for joint_name, joint_data in device_data.items():
            transformed_data[joint_name] = self._transform_to_scene_coordinates(joint_data)
        
        return transformed_data

    def _get_scene_T_wrist(self, vive_data: Dict) -> Gf.Matrix4d:
        """Get the wrist position and orientation from vive data."""
        if 'WM0' in vive_data:
            return self.pose_to_matrix(vive_data['WM0']) # FIXME: only works for left hand
        else:
            carb.log_warn("No Vive wrist tracker found")
            return None
    
    def _transform_manus_data(self, manus_data: Dict, scene_T_wrist: Gf.Matrix4d) -> Dict:
        """Transform Manus glove data: wrist * wrist_T_joint."""
        if scene_T_wrist is None:
            return manus_data
        transformed_data = {}
        for joint_name, joint_data in manus_data.items():
            if joint_name.startswith('left_'):
                joint_transformed = self.pose_to_matrix(joint_data) * scene_T_wrist
                transformed_data[joint_name] = self.matrix_to_pose(joint_transformed)
        return transformed_data
    
    def cleanup(self):
        if hasattr(self, 'manus_tracker'):
            self.manus_tracker.cleanup()
        if hasattr(self, 'vive_tracker'):
            self.vive_tracker.cleanup() 