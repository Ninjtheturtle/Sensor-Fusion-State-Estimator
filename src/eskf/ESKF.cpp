#include "eskf/ESKF.hpp"
#include "utils/QuaternionUtils.hpp"

namespace eskf {

ESKF::ESKF(const ESKFConfig& config)
    : config_(config)
    , imuPredictor_(config)
    , gpsUpdater_(config)
    , odomUpdater_(config)
{}

void ESKF::initialize(const NominalState& x0, const CovMatrix& P0,
                      double timestamp)
{
    x_            = x0;
    P_            = P0;
    dx_.delta.setZero();
    timestamp_    = timestamp;
    initialized_  = true;

    // Reset counters.
    gps_accepted_  = 0;
    gps_rejected_  = 0;
    odom_accepted_ = 0;
    odom_rejected_ = 0;
}

void ESKF::predictIMU(const io::ImuMeasurement& imu, double dt)
{
    if (!initialized_) return;
    if (dt <= 0.0 || dt > 0.5) return;  // sanity check (0.5s max gap)

    imuPredictor_.predict(x_, P_, imu, dt);
    timestamp_ = imu.timestamp;
}

bool ESKF::updateGPS(const utils::GpsMeasurement& gps)
{
    if (!initialized_) return false;

    const bool accepted = gpsUpdater_.update(x_, dx_, P_, gps);
    if (accepted) {
        ++gps_accepted_;
    } else {
        ++gps_rejected_;
    }
    return accepted;
}

bool ESKF::updateOdometry(const utils::OdomMeasurement& odom)
{
    if (!initialized_) return false;

    const bool accepted = odomUpdater_.update(x_, dx_, P_, odom);
    if (accepted) {
        ++odom_accepted_;
    } else {
        ++odom_rejected_;
    }
    return accepted;
}

void ESKF::injectAndReset()
{
    // Note: inject-and-reset is called internally by GpsUpdater and OdomUpdater
    // after each accepted measurement. This method is provided for external use
    // if needed (e.g., after fusing multiple measurements in one step).
    x_.p   += dx_.dp();
    x_.v   += dx_.dv();
    x_.q    = utils::applyRotVecRight(x_.q, dx_.dtheta());
    x_.b_g += dx_.db_g();
    x_.b_a += dx_.db_a();
    x_.normalizeQuat();

    dx_.delta.setZero();
}

} // namespace eskf
