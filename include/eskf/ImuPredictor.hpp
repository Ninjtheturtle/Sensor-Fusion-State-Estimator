#pragma once

#include "eskf/State.hpp"
#include "io/CsvLoader.hpp"

namespace eskf {

// Implements the ESKF prediction step driven by IMU measurements.
// On each call, integrates the nominal state forward using RK4 and
// propagates the error-state covariance using the linearized dynamics.
class ImuPredictor {
public:
    explicit ImuPredictor(const ESKFConfig& config);

    // Propagate nominal state x and covariance P forward by dt seconds
    // using the given IMU measurement.
    //
    // imu: raw IMU measurement (gyro + accel in body frame, with bias + noise)
    // dt:  time step [s] (should be ~0.005 s at 200 Hz)
    void predict(NominalState& x, CovMatrix& P,
                 const io::ImuMeasurement& imu, double dt);

private:
    ESKFConfig config_;

    // ── Nominal state integration ─────────────────────────────────────────────
    // RK4 integrator for position, velocity, and quaternion.
    // Biases are held constant during the integration step (piecewise constant).
    void integrateNominalRK4(NominalState& x,
                             const Eigen::Vector3d& a_corrected,
                             const Eigen::Vector3d& w_corrected,
                             double dt);

    // One-step derivative of [p, v, q] given corrected IMU and current state.
    struct StateDerivative {
        Eigen::Vector3d    dp;
        Eigen::Vector3d    dv;
        Eigen::Quaterniond dq;  // quaternion rate (not a unit quaternion)
    };
    StateDerivative computeDerivative(const NominalState& x,
                                      const Eigen::Vector3d& a_c,
                                      const Eigen::Vector3d& w_c) const;

    // ── Error state propagation ───────────────────────────────────────────────
    // Continuous-time error-state dynamics matrix F (15x15).
    Eigen::Matrix<double,15,15> buildF(const Eigen::Matrix3d& R,
                                       const Eigen::Vector3d& a_c,
                                       const Eigen::Vector3d& w_c) const;

    // Noise input matrix G (15x12) mapping 12 noise inputs to 15 error states.
    Eigen::Matrix<double,15,12> buildG(const Eigen::Matrix3d& R) const;

    // Continuous-time process noise covariance Qc (12x12, diagonal).
    Eigen::Matrix<double,12,12> buildQc() const;
};

} // namespace eskf
