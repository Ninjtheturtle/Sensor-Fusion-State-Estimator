#include "eskf/OdomUpdater.hpp"
#include "utils/MathUtils.hpp"
#include "utils/MahalanobisGate.hpp"
#include "utils/QuaternionUtils.hpp"

namespace eskf {

using namespace utils;

OdomUpdater::OdomUpdater(const ESKFConfig& config)
    : config_(config)
{
    R_odom_ = (config_.sigma_odom * config_.sigma_odom)
              * Eigen::Matrix3d::Identity();
}

bool OdomUpdater::update(NominalState& x, ErrorState& dx, CovMatrix& P,
                         const utils::OdomMeasurement& odom)
{
    // H = [0 | I₃ | 0 | 0 | 0] — only observes velocity
    Eigen::Matrix<double,3,15> H = Eigen::Matrix<double,3,15>::Zero();
    H.block<3,3>(0, 3) = Eigen::Matrix3d::Identity();

    const Eigen::Vector3d r = odom.velocity - x.v;

    const Eigen::Matrix3d S = H * P * H.transpose() + R_odom_;

    double d2;
    if (!mahalanobisGate<3>(r, S, config_.chi2_odom_threshold, &d2)) {
        return false;
    }

    const Eigen::Matrix<double,15,3> K =
        P * H.transpose() * safeInverse3x3(S);

    dx.delta = K * r;

    // Joseph form
    const Eigen::Matrix<double,15,15> IKH =
        Eigen::Matrix<double,15,15>::Identity() - K * H;
    P = IKH * P * IKH.transpose() + K * R_odom_ * K.transpose();
    P = symmetrize<15>(P);

    applyUpdate(x, dx, P, r);

    return true;
}

void OdomUpdater::applyUpdate(NominalState& x, ErrorState& dx, CovMatrix& /*P*/,
                               const Eigen::Vector3d& /*residual*/)
{
    x.p   += dx.dp();
    x.v   += dx.dv();
    x.q    = applyRotVecRight(x.q, dx.dtheta());
    x.b_g += dx.db_g();
    x.b_a += dx.db_a();
    x.normalizeQuat();

    dx.delta.setZero();
}

} // namespace eskf
