#include "utils/QuaternionUtils.hpp"
#include <cmath>

namespace eskf {
namespace utils {

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<     0.0, -v.z(),  v.y(),
           v.z(),    0.0, -v.x(),
          -v.y(),  v.x(),    0.0;
    return S;
}

Eigen::Matrix3d quatToRot(const Eigen::Quaterniond& q) {
    return q.normalized().toRotationMatrix();
}

Eigen::Quaterniond rotVecToQuat(const Eigen::Vector3d& dtheta) {
    const double angle = dtheta.norm();

    // Threshold below which we use small-angle approximation to avoid
    // division by near-zero sin(angle/2) / angle.
    const double kEps = 1e-10;

    double w, x, y, z;
    if (angle < kEps) {
        // First-order Taylor expansion: q ≈ [1, dtheta/2]
        w = 1.0;
        x = 0.5 * dtheta.x();
        y = 0.5 * dtheta.y();
        z = 0.5 * dtheta.z();
    } else {
        const double half  = 0.5 * angle;
        const double scale = std::sin(half) / angle;
        w = std::cos(half);
        x = scale * dtheta.x();
        y = scale * dtheta.y();
        z = scale * dtheta.z();
    }

    Eigen::Quaterniond q(w, x, y, z);
    return q.normalized();
}

Eigen::Vector3d quatToRotVec(const Eigen::Quaterniond& q_in) {
    // Ensure q.w >= 0 to pick the principal value (angle in [0, pi]).
    Eigen::Quaterniond q = q_in.normalized();
    if (q.w() < 0.0) {
        q.coeffs() = -q.coeffs(); // q and -q represent the same rotation
    }

    const double kEps = 1e-10;
    const double vec_norm = q.vec().norm();

    if (vec_norm < kEps) {
        // Near identity: angle -> 0, use Taylor expansion to avoid 0/0.
        // asin(x) ≈ x for small x, so theta ≈ 2 * vec_norm
        return 2.0 * q.vec();
    }

    const double angle = 2.0 * std::atan2(vec_norm, q.w());
    return (angle / vec_norm) * q.vec();
}

Eigen::Quaterniond applyRotVecRight(const Eigen::Quaterniond& q,
                                     const Eigen::Vector3d& dtheta) {
    // Right-multiply: q_new = q * dq, where dq represents the body-frame error.
    // This is the correct formulation for the ESKF attitude inject-and-reset.
    Eigen::Quaterniond dq = rotVecToQuat(dtheta);
    return (q * dq).normalized();
}

} // namespace utils
} // namespace eskf
