#pragma once

#include <Eigen/Core>
#include <Eigen/Cholesky>

namespace eskf {
namespace utils {

// chi2 95th percentile thresholds by DOF
namespace ChiSquaredThreshold {
    constexpr double kDof1 = 3.841;   // 1 DOF
    constexpr double kDof2 = 5.991;   // 2 DOF
    constexpr double kDof3 = 7.815;   // 3 DOF (GPS, odom)
    constexpr double kDof6 = 12.592;  // 6 DOF
} // namespace ChiSquaredThreshold

// d² = r^T * S⁻¹ * r — returns true if d² <= threshold (i.e., accept the measurement)
template <int N>
bool mahalanobisGate(
    const Eigen::Matrix<double, N, 1>& residual,
    const Eigen::Matrix<double, N, N>& S,
    double threshold,
    double* d2_out = nullptr)
{
    // S is PD by construction (H*P*H^T + R), so Cholesky is safe here
    Eigen::Matrix<double, N, 1> S_inv_r = S.llt().solve(residual);
    const double d2 = residual.dot(S_inv_r);

    if (d2_out) {
        *d2_out = d2;
    }
    return d2 <= threshold;
}

} // namespace utils
} // namespace eskf
