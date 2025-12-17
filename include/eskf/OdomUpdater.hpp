#pragma once

#include "eskf/State.hpp"
#include "utils/NoiseSim.hpp"

namespace eskf {

// Implements the ESKF measurement update for simulated wheel odometry.
//
// Measurement model: z_odom = v + noise,   noise ~ N(0, R_odom)
// Measurement matrix H (3x15): [0 | I₃ | 0 | 0 | 0]
//
// Uses Mahalanobis gating to reject outliers before updating.
// Uses the Joseph form for numerical stability.
class OdomUpdater {
public:
    explicit OdomUpdater(const ESKFConfig& config);

    // Attempt an odometry measurement update.
    // Returns true if accepted, false if rejected as outlier.
    bool update(NominalState& x, ErrorState& dx, CovMatrix& P,
                const utils::OdomMeasurement& odom);

private:
    ESKFConfig config_;
    Eigen::Matrix3d R_odom_;  // odometry noise covariance (3x3)

    void applyUpdate(NominalState& x, ErrorState& dx, CovMatrix& P,
                     const Eigen::Vector3d& residual);
};

} // namespace eskf
