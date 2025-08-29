#!/usr/bin/env python3
"""
Basic tests for isaacsim.xr.input_devices
- Verifies modules import
- Exercises individual trackers (mock-safe)
- Exercises XRDeviceIntegration end-to-end structure
"""

import os
import sys

# Add the impl directory to Python path so we can import the modules directly
impl_path = os.path.join(os.path.dirname(__file__), '..', 'impl')
sys.path.insert(0, os.path.abspath(impl_path))

def test_extension_import():
    """Test that core modules can be imported."""
    try:
        from xr_device_integration import XRDeviceIntegration, get_xr_device_integration  # noqa: F401
        from manus_tracker import IsaacSimManusGloveTracker  # noqa: F401
        from vive_tracker import IsaacSimViveTracker  # noqa: F401
        print("Modules imported successfully")
        return True
    except Exception as e:
        print(f"Import failed: {e}")
        return False

def test_individual_trackers():
    """Test individual tracker classes (mock-friendly)."""
    try:
        from manus_tracker import IsaacSimManusGloveTracker
        from vive_tracker import IsaacSimViveTracker

        manus = IsaacSimManusGloveTracker()
        manus.update()
        manus_data = manus.get_all_glove_data()
        print(f"Manus tracker: {len(manus_data)} joint entries")

        vive = IsaacSimViveTracker()
        vive.update()
        vive_data = vive.get_all_tracker_data()
        print(f"Vive tracker: {len(vive_data)} device entries")

        return True
    except Exception as e:
        print(f"Individual tracker test failed: {e}")
        return False

def test_basic_functionality():
    """Test XRDeviceIntegration orchestration and data schema."""
    try:
        from xr_device_integration import XRDeviceIntegration
        integration = XRDeviceIntegration()

        # Register and perform at least one update on both sources
        integration.register_devices()
        integration.update_manus()
        integration.update_vive()

        all_data = integration.get_all_device_data()
        assert isinstance(all_data, dict)
        assert 'manus_gloves' in all_data
        assert 'vive_trackers' in all_data
        assert 'device_status' in all_data

        # Validate structure if entries are present
        for group_key in ('manus_gloves', 'vive_trackers'):
            for _, pose in all_data[group_key].items():
                pos = pose.get('position')
                ori = pose.get('orientation')
                assert isinstance(pos, (list, tuple)) and len(pos) == 3
                assert isinstance(ori, (list, tuple)) and len(ori) == 4

        print("Integration produced structured device data")
        integration.cleanup()
        return True
    except AssertionError as e:
        print(f"Data shape assertion failed: {e}")
        return False
    except Exception as e:
        print(f"Integration test failed: {e}")
        return False

if __name__ == "__main__":
    print("Testing isaacsim.xr.input_devices")
    print("=" * 40)

    tests = [
        ("Module Imports", test_extension_import),
        ("Basic Functionality", test_basic_functionality),
        ("Individual Trackers", test_individual_trackers),
    ]

    passed = 0
    for name, fn in tests:
        print(f"\n{name}:")
        if fn():
            passed += 1

    print(f"\nResults: {passed}/{len(tests)} tests passed")
    if passed == len(tests):
        print("All tests passed")
        print("\nFor hardware testing: connect Manus gloves and Vive trackers, then run the sample.")
    else:
        print("Some tests failed") 