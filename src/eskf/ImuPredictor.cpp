#include "eskf/ImuPredictor.hpp"
#include "utils/QuaternionUtils.hpp"
#include "utils/MathUtils.hpp"

namespace eskf {

using namespace utils;

ImuPredictor::ImuPredictor(const ESKFConfig& config)
    : config_(config)
{}

void ImuPredictor::predict(NominalState& x, CovMatrix& P,
                           const io::ImuMeasurement& imu, double dt)
{
    // subtract biases before anything else
    const Eigen::Vector3d a_c = imu.accel - x.b_a;
    const Eigen::Vector3d w_c = imu.gyro  - x.b_g;

    const Eigen::Matrix3d R = quatToRot(x.q);

    integrateNominalRK4(x, a_c, w_c, dt);

    // use R from start of step — good enough at 200 Hz, midpoint barely helps
    const Eigen::Matrix<double,15,15> F = buildF(R, a_c, w_c);

    // second-order approx: Phi = I + F*dt + (F*dt)^2/2
    // third-order is O(dt^3) ≈ 1.25e-7 at 200 Hz, not worth it
    const Eigen::Matrix<double,15,15> Fdt = F * dt;
    const Eigen::Matrix<double,15,15> Phi =
        Eigen::Matrix<double,15,15>::Identity() + Fdt + 0.5 * Fdt * Fdt;

    const Eigen::Matrix3d R_new = quatToRot(x.q);  // updated R after integration
    const Eigen::Matrix<double,15,12> G  = buildG(R_new);
    const Eigen::Matrix<double,12,12> Qc = buildQc();

    // Qd ≈ G * Qc * G^T * dt (first-order Van Loan)
    // do NOT fold dt into G before squaring — that gives dt^2 and kills the gain
    const Eigen::Matrix<double,15,15> Qd = G * Qc * G.transpose() * dt;

    P = Phi * P * Phi.transpose() + Qd;
    P = symmetrize<15>(P);
}

ImuPredictor::StateDerivative
ImuPredictor::computeDerivative(const NominalState& x,
                                const Eigen::Vector3d& a_c,
                                const Eigen::Vector3d& w_c) const
{
    const Eigen::Matrix3d R = quatToRot(x.q);
    StateDerivative d;
    d.dp = x.v;
    d.dv = R * a_c + gravity();  // gravity() = [0,0,-9.81]^T
    // q_dot = 0.5 * q ⊗ [0, w_c]
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
    // biases held constant; q treated as a 4-vector during RK4
    auto eval = [&](const NominalState& s) -> StateDerivative {
        return computeDerivative(s, a_c, w_c);
    };

    const StateDerivative k1 = eval(x);

    NominalState x2;
    x2.p = x.p + 0.5 * dt * k1.dp;
    x2.v = x.v + 0.5 * dt * k1.dv;
    x2.q.coeffs() = x.q.coeffs() + 0.5 * dt * k1.dq.coeffs();
    x2.q.normalize();
    x2.b_g = x.b_g;
    x2.b_a = x.b_a;
    const StateDerivative k2 = eval(x2);

    NominalState x3;
    x3.p = x.p + 0.5 * dt * k2.dp;
    x3.v = x.v + 0.5 * dt * k2.dv;
    x3.q.coeffs() = x.q.coeffs() + 0.5 * dt * k2.dq.coeffs();
    x3.q.normalize();
    x3.b_g = x.b_g;
    x3.b_a = x.b_a;
    const StateDerivative k3 = eval(x3);

    NominalState x4;
    x4.p = x.p + dt * k3.dp;
    x4.v = x.v + dt * k3.dv;
    x4.q.coeffs() = x.q.coeffs() + dt * k3.dq.coeffs();
    x4.q.normalize();
    x4.b_g = x.b_g;
    x4.b_a = x.b_a;
    const StateDerivative k4 = eval(x4);

    x.p += (dt / 6.0) * (k1.dp + 2.0*k2.dp + 2.0*k3.dp + k4.dp);
    x.v += (dt / 6.0) * (k1.dv + 2.0*k2.dv + 2.0*k3.dv + k4.dv);
    x.q.coeffs() += (dt / 6.0) * (k1.dq.coeffs() + 2.0*k2.dq.coeffs()
                                  + 2.0*k3.dq.coeffs() + k4.dq.coeffs());
    x.q.normalize();
}

Eigen::Matrix<double,15,15>
ImuPredictor::buildF(const Eigen::Matrix3d& R,
                     const Eigen::Vector3d& a_c,
                     const Eigen::Vector3d& w_c) const
{
    Eigen::Matrix<double,15,15> F = Eigen::Matrix<double,15,15>::Zero();

    // layout: δp[0:3], δv[3:6], δθ[6:9], δb_g[9:12], δb_a[12:15]

    F.block<3,3>(0, 3) = Eigen::Matrix3d::Identity();  // δp_dot = δv

    F.block<3,3>(3, 6)  = -R * skew(a_c);  // δv_dot from attitude error
    F.block<3,3>(3, 12) = -R;              // δv_dot from accel bias

    F.block<3,3>(6, 6)  = -skew(w_c);                  // δθ_dot from attitude error
    F.block<3,3>(6, 9)  = -Eigen::Matrix3d::Identity(); // δθ_dot from gyro bias

    // biases are random walk — no dynamics terms needed

    return F;
}

Eigen::Matrix<double,15,12>
ImuPredictor::buildG(const Eigen::Matrix3d& R) const
{
    Eigen::Matrix<double,15,12> G = Eigen::Matrix<double,15,12>::Zero();

    // noise order: [n_acc(3), n_gyro(3), n_rw_gyro(3), n_rw_acc(3)]
    G.block<3,3>(3,  0) = R;                            // accel noise  -> δv
    G.block<3,3>(6,  3) = Eigen::Matrix3d::Identity();  // gyro noise   -> δθ
    G.block<3,3>(9,  6) = Eigen::Matrix3d::Identity();  // gyro rw      -> δb_g
    G.block<3,3>(12, 9) = Eigen::Matrix3d::Identity();  // accel rw     -> δb_a

    return G;
}

Eigen::Matrix<double,12,12> ImuPredictor::buildQc() const
{
    Eigen::Matrix<double,12,12> Qc = Eigen::Matrix<double,12,12>::Zero();

    const double san = config_.sigma_acc_n;
    const double sgn = config_.sigma_gyro_n;
    const double sar = config_.sigma_acc_rw;
    const double sgr = config_.sigma_gyro_rw;

    Qc.block<3,3>(0, 0) = (san * san) * Eigen::Matrix3d::Identity();  // accel noise
    Qc.block<3,3>(3, 3) = (sgn * sgn) * Eigen::Matrix3d::Identity();  // gyro noise
    Qc.block<3,3>(6, 6) = (sgr * sgr) * Eigen::Matrix3d::Identity();  // gyro rw
    Qc.block<3,3>(9, 9) = (sar * sar) * Eigen::Matrix3d::Identity();  // accel rw

    return Qc;
}

} // namespace eskf
