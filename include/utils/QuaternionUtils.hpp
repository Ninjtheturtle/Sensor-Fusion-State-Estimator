#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace eskf {
namespace utils {

// skew(v) * u == v.cross(u)
Eigen::Matrix3d skew(const Eigen::Vector3d& v);

// same as q.toRotationMatrix() but explicit
Eigen::Matrix3d quatToRot(const Eigen::Quaterniond& q);

// axis-angle vector -> unit quaternion; handles near-zero via Taylor expansion
Eigen::Quaterniond rotVecToQuat(const Eigen::Vector3d& dtheta);

// inverse of rotVecToQuat; negates q if w < 0 to get principal value
Eigen::Vector3d quatToRotVec(const Eigen::Quaterniond& q);

// right-multiply attitude error: q_new = (q * rotVecToQuat(dtheta)).normalized()
// used in ESKF inject-and-reset
Eigen::Quaterniond applyRotVecRight(const Eigen::Quaterniond& q,
                                    const Eigen::Vector3d& dtheta);

} // namespace utils
} // namespace eskf
