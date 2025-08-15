# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2025-08-08

### Added
- Initial release of IsaacSim XR Input Devices extension
- Support for Manus gloves with ManusSDK (c++ with Python binding)
- Support for Vive trackers with pysurvive
- Integration with IsaacSim XR system
- Synchronized device updates
- Mock data fallback for testing without hardware
- Documentation and build configuration

### Technical Details
- Adapted from isaac-deploy Holoscan implementations
- Rate-limited updates (60Hz default)
- Device status monitoring
- Cleanup and resource management

### Dependencies
- IsaacSim Core API
- IsaacSim Core Utils
- Omniverse Kit XR Core
- ManusSDK (optional - falls back to mock data)
- pysurvive (optional - falls back to mock data)
