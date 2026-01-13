#include <gtest/gtest.h>
#include "eskf/ImuPredictor.hpp"
#include "eskf/State.hpp"
#include "io/CsvLoader.hpp"
#include <Eigen/Eigenvalues>
#include <cmath>

using namespace eskf;

static io::ImuMeasurement makeImu(double t,
                                   const Eigen::Vector3d& gyro,
                                   const Eigen::Vector3d& accel)
{
    io::ImuMeasurement m;
    m.timestamp = t;
    m.gyro  = gyro;
    m.accel = accel;
    return m;
}

// ── Covariance properties ─────────────────────────────────────────────────────

TEST(ImuPredictor, CovarianceRemainsSymmetric) {
    ESKFConfig cfg;
    ImuPredictor pred(cfg);

    NominalState x;
    CovMatrix P = initCovariance(cfg);

    // Simulate zero motion (accel = gravity, gyro = 0)
    const auto imu = makeImu(0.0,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d(0.0, 0.0, 9.81));

    for (int i = 0; i < 200; ++i) {
        pred.predict(x, P, imu, 0.005);
    }

    const CovMatrix diff = P - P.transpose();
    EXPECT_LT(diff.norm(), 1e-10);
}

TEST(ImuPredictor, CovarianceRemainsPositiveDefinite) {
    ESKFConfig cfg;
    ImuPredictor pred(cfg);

    NominalState x;
    CovMatrix P = initCovariance(cfg);

    const auto imu = makeImu(0.0,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d(0.0, 0.0, 9.81));

    for (int i = 0; i < 200; ++i) {
        pred.predict(x, P, imu, 0.005);
    }

    // All eigenvalues must be positive.
    Eigen::SelfAdjointEigenSolver<CovMatrix> solver(P);
    EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
}

TEST(ImuPredictor, CovarianceGrowsWithTime) {
    // With no measurements, uncertainty must grow (or at worst stay constant).
    ESKFConfig cfg;
    ImuPredictor pred(cfg);

    NominalState x;
    CovMatrix P0 = initCovariance(cfg);
    CovMatrix P  = P0;

    const auto imu = makeImu(0.0,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d(0.0, 0.0, 9.81));

    for (int i = 0; i < 400; ++i) {
        pred.predict(x, P, imu, 0.005);
    }

    // Position variance (block 0:3) should have grown.
    // Use local vars to avoid comma-in-template confusing the EXPECT_GT macro.
    const double trace_P   = P.block<3,3>(0,0).trace();
    const double trace_P0  = P0.block<3,3>(0,0).trace();
    EXPECT_GT(trace_P, trace_P0);
}

// ── Nominal state integration ─────────────────────────────────────────────────

TEST(ImuPredictor, ZeroMotionDoesNotDrift) {
    // If accel = [0,0,g] and gyro = [0,0,0] and biases = 0,
    // the corrected accel = [0,0,g] - [0,0,0] = [0,0,g].
    // v_dot = R * a_c + g = R * [0,0,9.81] + [0,0,-9.81]
    // For identity R: v_dot = [0,0,9.81] + [0,0,-9.81] = [0,0,0] -> no drift.
    ESKFConfig cfg;
    ImuPredictor pred(cfg);

    NominalState x;
    CovMatrix P = initCovariance(cfg);

    const auto imu = makeImu(0.0,
        Eigen::Vector3d::Zero(),          // gyro = 0
        Eigen::Vector3d(0.0, 0.0, 9.81)); // accel = g upward

    const Eigen::Vector3d p0 = x.p;

    for (int i = 0; i < 200; ++i) {
        pred.predict(x, P, imu, 0.005);
    }

    // Position should not drift significantly (only noise-driven covariance growth).
    EXPECT_LT((x.p - p0).norm(), 1e-6);
}

TEST(ImuPredictor, ConstantAccelGivesQuadraticPosition) {
    // Apply constant upward acceleration: a_corrected = [0.5, 0, 0] m/s^2
    // Expected: p_x = 0.5 * 0.5 * t^2, v_x = 0.5 * t
    // With g compensation: bias accel = [0,0,-g], sensor reads [0.5, 0, g]
    ESKFConfig cfg;
    ImuPredictor pred(cfg);

    NominalState x;
    // Set bias so corrected accel = [0.5, 0, 0] + g_compensation in world
    // Simpler: set bias_a = [0,0,-g] so corrected = sensor + [0,0,g] = [0.5, 0, 0]
    // Actually: a_c = a_m - b_a; we want a_c = [0.5, 0, 0]
    // Set b_a = 0, a_m = [0.5, 0, 9.81] (sensor sees gravity + horizontal accel)
    // Then corrected = a_m - b_a = [0.5, 0, 9.81]
    // v_dot = R * [0.5, 0, 9.81] + [0, 0, -9.81] = [0.5, 0, 0] ✓
    CovMatrix P = initCovariance(cfg);

    const auto imu = makeImu(0.0,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d(0.5, 0.0, 9.81));

    const double dt = 0.005;
    const int N = 400;  // 2 seconds
    const double t = N * dt;

    for (int i = 0; i < N; ++i) {
        pred.predict(x, P, imu, dt);
    }

    const double expected_vx = 0.5 * t;
    const double expected_px = 0.5 * 0.5 * t * t;

    EXPECT_NEAR(x.v.x(), expected_vx, 1e-3);
    EXPECT_NEAR(x.p.x(), expected_px, 1e-3);
}

TEST(ImuPredictor, ConstantRotationUpdatesQuaternion) {
    // Apply constant rotation rate about Z: w_z = pi/2 rad/s
    // After 2 seconds, rotation = pi rad (180 deg)
    ESKFConfig cfg;
    ImuPredictor pred(cfg);

    NominalState x;
    CovMatrix P = initCovariance(cfg);

    const double omega_z = M_PI / 2.0;  // rad/s
    const auto imu = makeImu(0.0,
        Eigen::Vector3d(0.0, 0.0, omega_z),
        Eigen::Vector3d(0.0, 0.0, 9.81));

    const double dt = 0.005;
    const int N = 400;  // 2 seconds -> 180 deg rotation

    for (int i = 0; i < N; ++i) {
        pred.predict(x, P, imu, dt);
    }

    // After 180 deg rotation about Z, the quaternion should be near [0, 0, 0, 1]
    // (i.e., w ≈ 0, z ≈ ±1).
    EXPECT_NEAR(std::abs(x.q.z()), 1.0, 0.01);
    EXPECT_NEAR(x.q.norm(), 1.0, 1e-9);
}
