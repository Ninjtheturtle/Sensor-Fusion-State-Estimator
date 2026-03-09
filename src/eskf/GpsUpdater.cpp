#include "eskf/GpsUpdater.hpp"
#include "utils/MathUtils.hpp"
#include "utils/MahalanobisGate.hpp"
#include "utils/QuaternionUtils.hpp"

namespace eskf {

using namespace utils;

GpsUpdater::GpsUpdater(const ESKFConfig& config)
    : config_(config)
{
    R_gps_ = Eigen::Matrix3d::Zero();
    R_gps_(0,0) = config_.sigma_gps_xy * config_.sigma_gps_xy;
    R_gps_(1,1) = config_.sigma_gps_xy * config_.sigma_gps_xy;
    R_gps_(2,2) = config_.sigma_gps_z  * config_.sigma_gps_z;
}

bool GpsUpdater::update(NominalState& x, ErrorState& dx, CovMatrix& P,
                        const utils::GpsMeasurement& gps)
{
    // H = [I₃ | 0 | 0 | 0 | 0] — only observes position
    Eigen::Matrix<double,3,15> H = Eigen::Matrix<double,3,15>::Zero();
    H.block<3,3>(0, 0) = Eigen::Matrix3d::Identity();

    const Eigen::Vector3d r = gps.position - x.p;

    const Eigen::Matrix3d S = H * P * H.transpose() + R_gps_;

    double d2;
    if (!mahalanobisGate<3>(r, S, config_.chi2_gps_threshold, &d2)) {
        return false;
    }

    const Eigen::Matrix<double,15,3> K =
        P * H.transpose() * safeInverse3x3(S);

    dx.delta = K * r;

    // Joseph form — numerically stable, keeps P symmetric
    const Eigen::Matrix<double,15,15> IKH =
        Eigen::Matrix<double,15,15>::Identity() - K * H;
    P = IKH * P * IKH.transpose() + K * R_gps_ * K.transpose();
    P = symmetrize<15>(P);

    applyUpdate(x, dx, P, r);

    return true;
}

void GpsUpdater::applyUpdate(NominalState& x, ErrorState& dx, CovMatrix& /*P*/,
                              const Eigen::Vector3d& /*residual*/)
{
    x.p   += dx.dp();
    x.v   += dx.dv();
    x.q    = applyRotVecRight(x.q, dx.dtheta());  // multiplicative update for attitude
    x.b_g += dx.db_g();
    x.b_a += dx.db_a();
    x.normalizeQuat();

    dx.delta.setZero();
}

} // namespace eskf
