# Changelog

## [Unreleased]

### Added
- Add quaternion operations over `Array` batches in `Rotation.hpp`: `normalizeQuaternion()`,
  `conjugateQuaternion()`, `inverseQuaternion()`, `multiplyQuaternions()`, `dotQuaternions()`,
  `rotateByQuaternion()`, `rotateByQuaternionInverse()` and `averageQuaternions()`. Each accepts a
  `rotationFormat` argument, `"xyzw"` (default) or `"wxyz"`.
