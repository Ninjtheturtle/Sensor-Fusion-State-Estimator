#include "eskf/ImuPredictor.hpp"
#include "utils/QuaternionUtils.hpp"
#include "utils/MathUtils.hpp"

namespace eskf {

using namespace utils;

ImuPredictor::ImuPredictor(const ESKFConfig& config)
    : config_(config)
{}

// ── Public interface ──────────────────────────────────────────────────────────

void ImuPredictor::predict(NominalState& x, CovMatrix& P,
                           const io::ImuMeasurement& imu, double dt)
{
    // 1. Compute bias-corrected IMU readings.
    const Eigen::Vector3d a_c = imu.accel - x.b_a;  // corrected acceleration
    const Eigen::Vector3d w_c = imu.gyro  - x.b_g;  // corrected angular velocity

    // 2. Rotation matrix at current nominal state.
    const Eigen::Matrix3d R = quatToRot(x.q);

    // 3. Integrate nominal state forward using RK4.
    integrateNominalRK4(x, a_c, w_c, dt);

    // 4. Build continuous-time error-state dynamics matrix F (15x15).
    //    Uses R BEFORE the RK4 step (midpoint approximation could be used,
    //    but start-of-step is standard and sufficient at 200 Hz).
    const Eigen::Matrix<double,15,15> F = buildF(R, a_c, w_c);

    // 5. Compute discrete-time state transition matrix.
    //    Second-order approximation: Phi = I + F*dt + (F*dt)^2 / 2
    //    The third-order term is O(dt^3) ≈ 1.25e-7 at 200 Hz, negligible.
    const Eigen::Matrix<double,15,15> Fdt = F * dt;
    const Eigen::Matrix<double,15,15> Phi =
        Eigen::Matrix<double,15,15>::Identity() + Fdt + 0.5 * Fdt * Fdt;

    // 6. Build noise input matrix G and continuous noise covariance Qc.
    const Eigen::Matrix3d R_new = quatToRot(x.q);  // updated R after integration
    const Eigen::Matrix<double,15,12> G  = buildG(R_new);
    const Eigen::Matrix<double,12,12> Qc = buildQc();

    // 7. Discrete process noise: Qd ≈ G * Qc * G^T * dt
    //    First-order Van Loan approximation: Qd = integral_0^dt Phi(s)*G*Qc*G^T*Phi(s)^T ds
    //    For small dt (200 Hz), leading term G*Qc*G^T*dt dominates.
    //    NOTE: do NOT multiply G by dt before squaring — that gives dt^2, which is
    //    200x too small at 5 ms steps and causes the filter to ignore GPS/odom.
    const Eigen::Matrix<double,15,15> Qd = G * Qc * G.transpose() * dt;

    // 8. Propagate covariance.
    P = Phi * P * Phi.transpose() + Qd;

    // 9. Symmetrize to prevent numerical drift.
    P = symmetrize<15>(P);
}

// ── Nominal state RK4 integration ────────────────────────────────────────────

ImuPredictor::StateDerivative
ImuPredictor::computeDerivative(const NominalState& x,
                                const Eigen::Vector3d& a_c,
                                const Eigen::Vector3d& w_c) const
{
    const Eigen::Matrix3d R = quatToRot(x.q);
    StateDerivative d;
    d.dp = x.v;
    d.dv = R * a_c + gravity();  // gravity() = [0,0,-9.81]^T
    // Quaternion kinematic: q_dot = 0.5 * q ⊗ Quaternion(0, w_c)
    // Implemented as: q_dot.coeffs = 0.5 * Omega(w_c) * q.coeffs
    // Using Eigen's convention (w, x, y, z):
    const Eigen::Quaterniond omega_q(0.0, 0.5 * w_c.x(),
                                          0.5 * w_c.y(),
                                          0.5 * w_c.z());
    d.dq.coeffs() = (x.q * omega_q).coeffs();
    return d;
}

void ImuPredictor::integrateNominalRK4(NominalState& x,
                                        const Eigen::Vector3d& a_c,
                                        const Eigen::Vector3d& w_c,
                                        double dt)
{
    // RK4 operates on [p, v, q] treating q as a 4-vector.
    // Biases are held constant during the step.

    auto eval = [&](const NominalState& s) -> StateDerivative {
        return computeDerivative(s, a_c, w_c);
    };

    // k1
    const StateDerivative k1 = eval(x);

    // k2: state at t + dt/2 using k1 derivatives
    NominalState x2;
    x2.p = x.p + 0.5 * dt * k1.dp;
    x2.v = x.v + 0.5 * dt * k1.dv;
    x2.q.coeffs() = x.q.coeffs() + 0.5 * dt * k1.dq.coeffs();
    x2.q.normalize();
    x2.b_g = x.b_g;
    x2.b_a = x.b_a;
    const StateDerivative k2 = eval(x2);

    // k3: state at t + dt/2 using k2 derivatives
    NominalState x3;
    x3.p = x.p + 0.5 * dt * k2.dp;
    x3.v = x.v + 0.5 * dt * k2.dv;
    x3.q.coeffs() = x.q.coeffs() + 0.5 * dt * k2.dq.coeffs();
    x3.q.normalize();
    x3.b_g = x.b_g;
    x3.b_a = x.b_a;
    const StateDerivative k3 = eval(x3);

    // k4: state at t + dt using k3 derivatives
    NominalState x4;
    x4.p = x.p + dt * k3.dp;
    x4.v = x.v + dt * k3.dv;
    x4.q.coeffs() = x.q.coeffs() + dt * k3.dq.coeffs();
    x4.q.normalize();
    x4.b_g = x.b_g;
    x4.b_a = x.b_a;
    const StateDerivative k4 = eval(x4);

    // Combine: x_new = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    x.p += (dt / 6.0) * (k1.dp + 2.0*k2.dp + 2.0*k3.dp + k4.dp);
    x.v += (dt / 6.0) * (k1.dv + 2.0*k2.dv + 2.0*k3.dv + k4.dv);
    x.q.coeffs() += (dt / 6.0) * (k1.dq.coeffs() + 2.0*k2.dq.coeffs()
                                  + 2.0*k3.dq.coeffs() + k4.dq.coeffs());
    x.q.normalize();
}

// ── Error-state dynamics matrices ─────────────────────────────────────────────

Eigen::Matrix<double,15,15>
ImuPredictor::buildF(const Eigen::Matrix3d& R,
                     const Eigen::Vector3d& a_c,
                     const Eigen::Vector3d& w_c) const
{
    Eigen::Matrix<double,15,15> F = Eigen::Matrix<double,15,15>::Zero();

    // Block layout (rows x cols):
    // δp[0:3], δv[3:6], δθ[6:9], δb_g[9:12], δb_a[12:15]

    // δp_dot = δv
    F.block<3,3>(0, 3) = Eigen::Matrix3d::Identity();

    // δv_dot = -R * [a_c]_x * δθ - R * δb_a
    F.block<3,3>(3, 6)  = -R * skew(a_c);
    F.block<3,3>(3, 12) = -R;

    // δθ_dot = -[w_c]_x * δθ - δb_g
    F.block<3,3>(6, 6)  = -skew(w_c);
    F.block<3,3>(6, 9)  = -Eigen::Matrix3d::Identity();

    // δb_g_dot = 0 (random walk, noise-only driven)
    // δb_a_dot = 0 (random walk, noise-only driven)

    return F;
}

Eigen::Matrix<double,15,12>
ImuPredictor::buildG(const Eigen::Matrix3d& R) const
{
    Eigen::Matrix<double,15,12> G = Eigen::Matrix<double,15,12>::Zero();

    // Noise input order: [n_acc(3), n_gyro(3), n_rw_gyro(3), n_rw_acc(3)]
    // δv is driven by accelerometer noise (rotated to world frame)
    G.block<3,3>(3,  0) = R;
    // δθ is driven by gyroscope noise
    G.block<3,3>(6,  3) = Eigen::Matrix3d::Identity();
    // δb_g is driven by gyroscope random walk
    G.block<3,3>(9,  6) = Eigen::Matrix3d::Identity();
    // δb_a is driven by accelerometer random walk
    G.block<3,3>(12, 9) = Eigen::Matrix3d::Identity();

    return G;
}

Eigen::Matrix<double,12,12> ImuPredictor::buildQc() const
{
    Eigen::Matrix<double,12,12> Qc = Eigen::Matrix<double,12,12>::Zero();

    const double san = config_.sigma_acc_n;
    const double sgn = config_.sigma_gyro_n;
    const double sar = config_.sigma_acc_rw;
    const double sgr = config_.sigma_gyro_rw;

    // Diagonal blocks: variances = sigma^2 * I3
    Qc.block<3,3>(0, 0) = (san * san) * Eigen::Matrix3d::Identity();  // accel noise
    Qc.block<3,3>(3, 3) = (sgn * sgn) * Eigen::Matrix3d::Identity();  // gyro noise
    Qc.block<3,3>(6, 6) = (sgr * sgr) * Eigen::Matrix3d::Identity();  // gyro rw
    Qc.block<3,3>(9, 9) = (sar * sar) * Eigen::Matrix3d::Identity();  // accel rw

    return Qc;
}

} // namespace eskf
