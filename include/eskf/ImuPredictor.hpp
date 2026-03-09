#pragma once

#include "eskf/State.hpp"
#include "io/CsvLoader.hpp"

namespace eskf {

// handles the prediction step: RK4 for nominal state + linearized covariance propagation
class ImuPredictor {
public:
    explicit ImuPredictor(const ESKFConfig& config);

    // step x and P forward by dt; imu is raw (bias+noise still in it)
    void predict(NominalState& x, CovMatrix& P,
                 const io::ImuMeasurement& imu, double dt);

private:
    ESKFConfig config_;

    // RK4 for [p, v, q]; biases held constant during the step
    void integrateNominalRK4(NominalState& x,
                             const Eigen::Vector3d& a_corrected,
                             const Eigen::Vector3d& w_corrected,
                             double dt);

    struct StateDerivative {
        Eigen::Vector3d    dp;
        Eigen::Vector3d    dv;
        Eigen::Quaterniond dq;  // not a unit quat — just a rate
    };
    StateDerivative computeDerivative(const NominalState& x,
                                      const Eigen::Vector3d& a_c,
                                      const Eigen::Vector3d& w_c) const;

    // continuous-time error-state dynamics F (15x15)
    Eigen::Matrix<double,15,15> buildF(const Eigen::Matrix3d& R,
                                       const Eigen::Vector3d& a_c,
                                       const Eigen::Vector3d& w_c) const;

    // G maps 12 noise inputs to 15 error states
    Eigen::Matrix<double,15,12> buildG(const Eigen::Matrix3d& R) const;

    // continuous-time process noise Qc (12x12, diagonal)
    Eigen::Matrix<double,12,12> buildQc() const;
};

} // namespace eskf
