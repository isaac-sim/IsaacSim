#!/usr/bin/env python3
"""
Basic test for IsaacSim XR Input Devices extension
Tests that the extension can be imported and basic functionality works
"""

import sys
import os

# Add the impl directory to Python path so we can import the modules directly
impl_path = os.path.join(os.path.dirname(__file__), '..', 'impl')
sys.path.insert(0, impl_path)

def test_extension_import():
    """Test that the extension modules can be imported"""
    try:
        # Test importing the modules directly
        from xr_device_integration import XRDeviceIntegration
        from manus_tracker import IsaacSimManusGloveTracker
        from vive_tracker import IsaacSimViveTracker
        from extension import Extension
        
        print("✅ All extension modules imported successfully")
        return True
    except ImportError as e:
        print(f"❌ Failed to import extension modules: {e}")
        return False

def test_individual_trackers():
    """Test individual tracker functionality"""
    try:
        from manus_tracker import IsaacSimManusGloveTracker
        from vive_tracker import IsaacSimViveTracker
        
        # Test Manus tracker
        manus_tracker = IsaacSimManusGloveTracker()
        manus_tracker.update()
        manus_data = manus_tracker.get_all_glove_data()
        print(f"✅ Manus tracker: {len(manus_data)} glove(s)")
        
        # Test Vive tracker
        vive_tracker = IsaacSimViveTracker()
        vive_tracker.update()
        vive_data = vive_tracker.get_all_tracker_data()
        print(f"✅ Vive tracker: {len(vive_data)} tracker(s)")
        
        return True
        
    except Exception as e:
        print(f"❌ Individual tracker test failed: {e}")
        return False

if __name__ == "__main__":
    print("🚀 Testing IsaacSim XR Input Devices Extension")
    print("=" * 40)
    
    tests = [
        ("Extension Import", test_extension_import),
        ("Basic Functionality", test_basic_functionality),
        ("Individual Trackers", test_individual_trackers),
    ]
    
    passed = 0
    for test_name, test_func in tests:
        print(f"\n{test_name}:")
        if test_func():
            passed += 1
    
    print(f"\n📊 Results: {passed}/{len(tests)} tests passed")
    
    if passed == len(tests):
        print("🎉 Extension is working correctly!")
        print("\n💡 To test with real hardware:")
        print("   - Connect Manus gloves")
        print("   - Connect Vive trackers")
        print("   - Run IsaacSim and load the extension")
    else:
        print("⚠️  Some tests failed") 