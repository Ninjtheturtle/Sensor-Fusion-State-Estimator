#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace eskf {

// ESKF tuning knobs — tweak these if the filter misbehaves

struct ESKFConfig {
    // inflated ~10-20x from ADIS16448 nominal; unmodeled errors (temp drift, misalignment)
    // eat into the budget fast, so keeping gain high matters more than being precise here
    double sigma_acc_n  = 4.0e-2;   // nominal: 2.0e-3 (20x inflation)
    double sigma_gyro_n = 1.6e-3;   // nominal: 1.6e-4 (10x inflation)
    double sigma_acc_rw = 5.0e-3;   // nominal: 3.0e-3 (1.7x)
    double sigma_gyro_rw= 1.0e-3;   // nominal: 1.9e-5 (50x inflation)

    // GPS noise (simulated from GT)
    double sigma_gps_xy = 0.05;   // [m] horizontal
    double sigma_gps_z  = 0.10;   // [m] vertical
    double gps_rate_hz  = 10.0;

    // odom noise (simulated from GT velocity)
    double sigma_odom     = 0.05; // [m/s] all axes
    double odom_rate_hz   = 10.0;

    // chi2 thresholds, 95% confidence, 3 DOF
    double chi2_gps_threshold  = 7.815;
    double chi2_odom_threshold = 7.815;

    // init covariance — needs to be large so GPS/odom have real Kalman gain at startup.
    // with small init P, K_GPS is ~3.8% and the filter basically ignores corrections.
    double init_pos_std  = 0.5;    // [m]    was 0.01 — K_GPS_init ≈ 99%
    double init_vel_std  = 0.3;    // [m/s]  was 0.01 — K_odom_init ≈ 97%
    double init_att_std  = 1e-3;   // [rad]  ~0.057 deg
    double init_bgyr_std = 1e-3;   // [rad/s]
    double init_bacc_std = 1e-2;   // [m/s²]
};

// nominal state — NOT a flat vector; quaternion lives on the manifold
// biases modeled as random walk (held constant during IMU integration)
struct NominalState {
    Eigen::Vector3d    p;    // position [m], world frame
    Eigen::Vector3d    v;    // velocity [m/s], world frame
    Eigen::Quaterniond q;    // orientation (body-to-world), unit quaternion
    Eigen::Vector3d    b_g;  // gyroscope bias [rad/s], body frame
    Eigen::Vector3d    b_a;  // accelerometer bias [m/s²], body frame

    NominalState()
        : p(Eigen::Vector3d::Zero())
        , v(Eigen::Vector3d::Zero())
        , q(Eigen::Quaterniond::Identity())
        , b_g(Eigen::Vector3d::Zero())
        , b_a(Eigen::Vector3d::Zero())
    {}

    // call after inject-and-reset
    void normalizeQuat() { q.normalize(); }
};

// error state: 15-element tangent vector
// layout: [δp(0:3), δv(3:6), δθ(6:9), δb_g(9:12), δb_a(12:15)]
// δθ is a rotation vector; applied multiplicatively as δq ≈ [1, δθ/2]
using ErrorStateVec = Eigen::Matrix<double, 15, 1>;
using CovMatrix     = Eigen::Matrix<double, 15, 15>;

struct ErrorState {
    ErrorStateVec delta = ErrorStateVec::Zero();

    auto dp()     { return delta.segment<3>(0);  }
    auto dv()     { return delta.segment<3>(3);  }
    auto dtheta() { return delta.segment<3>(6);  }
    auto db_g()   { return delta.segment<3>(9);  }
    auto db_a()   { return delta.segment<3>(12); }

    auto dp()     const { return delta.segment<3>(0);  }
    auto dv()     const { return delta.segment<3>(3);  }
    auto dtheta() const { return delta.segment<3>(6);  }
    auto db_g()   const { return delta.segment<3>(9);  }
    auto db_a()   const { return delta.segment<3>(12); }
};

inline CovMatrix initCovariance(const ESKFConfig& cfg) {
    CovMatrix P = CovMatrix::Zero();
    const double pp = cfg.init_pos_std  * cfg.init_pos_std;
    const double vv = cfg.init_vel_std  * cfg.init_vel_std;
    const double tt = cfg.init_att_std  * cfg.init_att_std;
    const double gg = cfg.init_bgyr_std * cfg.init_bgyr_std;
    const double aa = cfg.init_bacc_std * cfg.init_bacc_std;
    for (int i = 0;  i < 3;  ++i) P(i,   i)   = pp;
    for (int i = 3;  i < 6;  ++i) P(i,   i)   = vv;
    for (int i = 6;  i < 9;  ++i) P(i,   i)   = tt;
    for (int i = 9;  i < 12; ++i) P(i,   i)   = gg;
    for (int i = 12; i < 15; ++i) P(i,   i)   = aa;
    return P;
}

} // namespace eskf
