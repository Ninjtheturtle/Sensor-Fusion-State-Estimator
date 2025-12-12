#include "eskf/GpsUpdater.hpp"
#include "utils/MathUtils.hpp"
#include "utils/MahalanobisGate.hpp"
#include "utils/QuaternionUtils.hpp"

namespace eskf {

using namespace utils;

GpsUpdater::GpsUpdater(const ESKFConfig& config)
    : config_(config)
{
    // Build 3x3 diagonal GPS measurement noise covariance.
    R_gps_ = Eigen::Matrix3d::Zero();
    R_gps_(0,0) = config_.sigma_gps_xy * config_.sigma_gps_xy;
    R_gps_(1,1) = config_.sigma_gps_xy * config_.sigma_gps_xy;
    R_gps_(2,2) = config_.sigma_gps_z  * config_.sigma_gps_z;
}

bool GpsUpdater::update(NominalState& x, ErrorState& dx, CovMatrix& P,
                        const utils::GpsMeasurement& gps)
{
    // H matrix (3x15): position block only.
    // H = [I₃ | 0₃ | 0₃ | 0₃ | 0₃]
    Eigen::Matrix<double,3,15> H = Eigen::Matrix<double,3,15>::Zero();
    H.block<3,3>(0, 0) = Eigen::Matrix3d::Identity();

    // Innovation: difference between measured and predicted position.
    const Eigen::Vector3d r = gps.position - x.p;

    // Innovation covariance: S = H * P * H^T + R_gps
    const Eigen::Matrix3d S = H * P * H.transpose() + R_gps_;

    // Mahalanobis gate: reject if d² > threshold.
    double d2;
    if (!mahalanobisGate<3>(r, S, config_.chi2_gps_threshold, &d2)) {
        return false;  // measurement rejected
    }

    // Compute Kalman gain: K = P * H^T * S^{-1}  (15x3)
    const Eigen::Matrix<double,15,3> K =
        P * H.transpose() * safeInverse3x3(S);

    // Update error state.
    dx.delta = K * r;

    // Joseph form covariance update (numerically stable, preserves symmetry):
    // P = (I - K*H) * P * (I - K*H)^T + K * R_gps * K^T
    const Eigen::Matrix<double,15,15> IKH =
        Eigen::Matrix<double,15,15>::Identity() - K * H;
    P = IKH * P * IKH.transpose() + K * R_gps_ * K.transpose();
    P = symmetrize<15>(P);

    // Inject error state into nominal state and reset dx.
    applyUpdate(x, dx, P, r);

    return true;
}

void GpsUpdater::applyUpdate(NominalState& x, ErrorState& dx, CovMatrix& /*P*/,
                              const Eigen::Vector3d& /*residual*/)
{
    // Inject error state into nominal state.
    x.p   += dx.dp();
    x.v   += dx.dv();
    x.q    = applyRotVecRight(x.q, dx.dtheta());  // multiplicative
    x.b_g += dx.db_g();
    x.b_a += dx.db_a();
    x.normalizeQuat();

    // Reset error state to zero.
    dx.delta.setZero();
}

} // namespace eskf
