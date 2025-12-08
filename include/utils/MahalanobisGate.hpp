#pragma once

#include <Eigen/Core>
#include <Eigen/Cholesky>

namespace eskf {
namespace utils {

// Chi-squared thresholds for Mahalanobis distance outlier rejection.
// Values are for the 95th percentile (alpha = 0.05) with k degrees of freedom.
// A measurement is rejected if its squared Mahalanobis distance exceeds the threshold.
namespace ChiSquaredThreshold {
    constexpr double kDof1 = 3.841;   // 1 DOF  (single axis measurement)
    constexpr double kDof2 = 5.991;   // 2 DOF
    constexpr double kDof3 = 7.815;   // 3 DOF  (GPS position, odom velocity)
    constexpr double kDof6 = 12.592;  // 6 DOF  (pose measurement)
} // namespace ChiSquaredThreshold

// Tests whether a measurement residual passes the Mahalanobis distance gate.
//
// The Mahalanobis distance squared is:
//   d² = r^T * S⁻¹ * r
// where r is the innovation (residual) and S is the innovation covariance.
//
// Returns true if the measurement should be accepted (d² <= threshold).
// Returns false to reject the measurement as an outlier.
//
// Template parameter N is the measurement dimension.
template <int N>
bool mahalanobisGate(
    const Eigen::Matrix<double, N, 1>& residual,
    const Eigen::Matrix<double, N, N>& S,
    double threshold,
    double* d2_out = nullptr)
{
    // Use LLT (Cholesky) solve for numerical efficiency.
    // S is positive definite by construction (H*P*H^T + R, both PD).
    Eigen::Matrix<double, N, 1> S_inv_r = S.llt().solve(residual);
    const double d2 = residual.dot(S_inv_r);

    if (d2_out) {
        *d2_out = d2;
    }
    return d2 <= threshold;
}

} // namespace utils
} // namespace eskf
