#pragma once

#include "eskf/State.hpp"
#include "utils/NoiseSim.hpp"

namespace eskf {

// GPS position update: z = p + noise, H = [I₃|0|0|0|0]
// uses Mahalanobis gating + Joseph form covariance update
class GpsUpdater {
public:
    explicit GpsUpdater(const ESKFConfig& config);

    // returns true if accepted
    bool update(NominalState& x, ErrorState& dx, CovMatrix& P,
                const utils::GpsMeasurement& gps);

private:
    ESKFConfig config_;
    Eigen::Matrix3d R_gps_;

    void applyUpdate(NominalState& x, ErrorState& dx, CovMatrix& P,
                     const Eigen::Vector3d& residual);
};

} // namespace eskf
