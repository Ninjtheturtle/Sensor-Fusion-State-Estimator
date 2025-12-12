#pragma once

#include "eskf/State.hpp"
#include "utils/NoiseSim.hpp"

namespace eskf {

// Implements the ESKF measurement update for simulated GPS position.
//
// Measurement model: z_gps = p + noise,   noise ~ N(0, R_gps)
// Measurement matrix H (3x15): [I₃ | 0 | 0 | 0 | 0]
//
// Uses Mahalanobis gating to reject outliers before updating.
// Uses the Joseph form for numerical stability.
class GpsUpdater {
public:
    explicit GpsUpdater(const ESKFConfig& config);

    // Attempt a GPS measurement update.
    // If the measurement passes the Mahalanobis gate, updates x, dx, and P,
    // then calls injectAndReset to fold dx into x and zero dx.
    // Returns true if the measurement was accepted, false if rejected.
    bool update(NominalState& x, ErrorState& dx, CovMatrix& P,
                const utils::GpsMeasurement& gps);

private:
    ESKFConfig config_;
    Eigen::Matrix3d R_gps_;  // GPS measurement noise covariance (3x3)

    // Perform the core Kalman update given a valid residual.
    void applyUpdate(NominalState& x, ErrorState& dx, CovMatrix& P,
                     const Eigen::Vector3d& residual);
};

} // namespace eskf
