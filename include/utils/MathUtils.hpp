#pragma once

#include <Eigen/Core>
#include <Eigen/LU>

namespace eskf {
namespace utils {

// Symmetrize a matrix: M = (M + M^T) / 2.
// Applied to covariance P after each propagation/update step to prevent
// numerical drift from exact symmetry, which would cause negative eigenvalues.
template <int N>
inline Eigen::Matrix<double, N, N>
symmetrize(const Eigen::Matrix<double, N, N>& M) {
    return 0.5 * (M + M.transpose());
}

// Safe inversion of a 3x3 positive-definite matrix using LLT (Cholesky).
// Falls back to full-pivot LU if Cholesky fails (non-positive-definite input).
Eigen::Matrix3d safeInverse3x3(const Eigen::Matrix3d& M);

// Clamp a value to [lo, hi].
inline double clamp(double val, double lo, double hi) {
    return val < lo ? lo : (val > hi ? hi : val);
}

// Gravity vector in world frame (NED-aligned z-down convention used by EuRoC).
// EuRoC ground truth uses z-up ENU convention; gravity is -z.
inline Eigen::Vector3d gravity() {
    return Eigen::Vector3d(0.0, 0.0, -9.81);
}

} // namespace utils
} // namespace eskf
