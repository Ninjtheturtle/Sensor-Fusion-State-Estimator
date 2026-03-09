#pragma once

#include <Eigen/Core>
#include <Eigen/LU>

namespace eskf {
namespace utils {

// M = (M + M^T) / 2 — call after propagation/update to prevent eigenvalue drift
template <int N>
inline Eigen::Matrix<double, N, N>
symmetrize(const Eigen::Matrix<double, N, N>& M) {
    return 0.5 * (M + M.transpose());
}

// tries Cholesky first, falls back to full-pivot LU if S isn't positive definite
Eigen::Matrix3d safeInverse3x3(const Eigen::Matrix3d& M);

inline double clamp(double val, double lo, double hi) {
    return val < lo ? lo : (val > hi ? hi : val);
}

// EuRoC uses z-up ENU, so gravity points in -z
inline Eigen::Vector3d gravity() {
    return Eigen::Vector3d(0.0, 0.0, -9.81);
}

} // namespace utils
} // namespace eskf
