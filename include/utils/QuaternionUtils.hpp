#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace eskf {
namespace utils {

// Returns the 3x3 skew-symmetric (cross-product) matrix of a 3-vector.
// skew(v) * u == v.cross(u)
Eigen::Matrix3d skew(const Eigen::Vector3d& v);

// Convert a unit quaternion to its 3x3 rotation matrix.
// Equivalent to q.toRotationMatrix() but explicit.
Eigen::Matrix3d quatToRot(const Eigen::Quaterniond& q);

// Convert a small rotation vector (axis-angle) to a unit quaternion.
// For |dtheta| -> 0, returns identity quaternion.
// Formula: q = [cos(|dtheta|/2), sin(|dtheta|/2) * dtheta / |dtheta|]
// For small angles: q ≈ [1, dtheta/2].normalized()
Eigen::Quaterniond rotVecToQuat(const Eigen::Vector3d& dtheta);

// Convert a unit quaternion to a rotation vector (axis-angle, principal value).
// Inverse of rotVecToQuat. Handles q.w < 0 by negating q first.
// For q near identity, uses Taylor expansion to avoid division by zero.
Eigen::Vector3d quatToRotVec(const Eigen::Quaterniond& q);

// Apply a small rotation error (right-multiply) to a quaternion.
// Result: q_new = (q * rotVecToQuat(dtheta)).normalized()
// Used in the ESKF inject-and-reset step for attitude correction.
Eigen::Quaterniond applyRotVecRight(const Eigen::Quaterniond& q,
                                    const Eigen::Vector3d& dtheta);

} // namespace utils
} // namespace eskf
