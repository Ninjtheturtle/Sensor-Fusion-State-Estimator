#pragma once

#include "eskf/State.hpp"
#include "utils/NoiseSim.hpp"

namespace eskf {

// odometry velocity update: z = v + noise, H = [0|I₃|0|0|0]
// same structure as GPS updater but observes velocity
class OdomUpdater {
public:
    explicit OdomUpdater(const ESKFConfig& config);

    // returns true if accepted
    bool update(NominalState& x, ErrorState& dx, CovMatrix& P,
                const utils::OdomMeasurement& odom);

private:
    ESKFConfig config_;
    Eigen::Matrix3d R_odom_;

    void applyUpdate(NominalState& x, ErrorState& dx, CovMatrix& P,
                     const Eigen::Vector3d& residual);
};

} // namespace eskf
