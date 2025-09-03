# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import carb
from .manus_tracker import IsaacSimManusGloveTracker
from .vive_tracker import IsaacSimViveTracker


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
        self.device_status = {
            'manus_gloves': {'connected': False, 'last_data_time': 0},
            'vive_trackers': {'connected': False, 'last_data_time': 0},
            'left_hand_connected': False,
            'right_hand_connected': False
        }
        
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
    
    def cleanup(self):
        if hasattr(self, 'manus_tracker'):
            self.manus_tracker.cleanup()
        if hasattr(self, 'vive_tracker'):
            self.vive_tracker.cleanup()
