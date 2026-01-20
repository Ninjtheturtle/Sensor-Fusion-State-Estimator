#include <gtest/gtest.h>
#include "eskf/ESKF.hpp"
#include "eskf/State.hpp"
#include "io/CsvLoader.hpp"
#include "utils/NoiseSim.hpp"
#include <Eigen/Eigenvalues>
#include <cmath>

using namespace eskf;

// ── Helper: generate N steps of stationary IMU data ───────────────────────────

static std::vector<io::ImuMeasurement> makeStationaryImu(int N, double dt) {
    std::vector<io::ImuMeasurement> data;
    for (int i = 0; i <= N; ++i) {
        io::ImuMeasurement m;
        m.timestamp = i * dt;
        m.gyro  = Eigen::Vector3d::Zero();
        m.accel = Eigen::Vector3d(0.0, 0.0, 9.81);  // gravity
        data.push_back(m);
    }
    return data;
}

// ── Initialization test ───────────────────────────────────────────────────────

TEST(ESKFIntegration, InitializeSetsState) {
    ESKFConfig cfg;
    ESKF filter(cfg);

    NominalState x0;
    x0.p = Eigen::Vector3d(1.0, 2.0, 3.0);
    x0.v = Eigen::Vector3d(0.1, 0.2, 0.3);
    x0.q = Eigen::Quaterniond(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
    x0.b_g = Eigen::Vector3d(0.001, 0.0, 0.0);
    x0.b_a = Eigen::Vector3d(0.0, 0.0, 0.01);

    CovMatrix P0 = initCovariance(cfg);
    filter.initialize(x0, P0, 0.0);

    EXPECT_NEAR((filter.nominalState().p - x0.p).norm(), 0.0, 1e-15);
    EXPECT_NEAR((filter.nominalState().v - x0.v).norm(), 0.0, 1e-15);
    EXPECT_NEAR(filter.nominalState().q.norm(), 1.0, 1e-12);
}

// ── Stationary filter test ────────────────────────────────────────────────────

TEST(ESKFIntegration, StationaryFilterDoesNotDriftFast) {
    ESKFConfig cfg;
    ESKF filter(cfg);

    NominalState x0;
    CovMatrix    P0 = initCovariance(cfg);
    filter.initialize(x0, P0, 0.0);

    // Run 1 second of stationary IMU.
    const auto imu_data = makeStationaryImu(200, 0.005);
    for (size_t i = 1; i < imu_data.size(); ++i) {
        filter.predictIMU(imu_data[i], 0.005);
    }

    // Position should stay near origin (drift < 1mm for 1 second).
    EXPECT_LT(filter.nominalState().p.norm(), 0.001);
}

// ── GPS convergence test ──────────────────────────────────────────────────────

TEST(ESKFIntegration, GpsUpdateConvergesPosition) {
    ESKFConfig cfg;
    ESKF filter(cfg);

    NominalState x0;
    CovMatrix    P0 = initCovariance(cfg);
    P0 *= 100.0;  // large initial uncertainty
    filter.initialize(x0, P0, 0.0);

    // Apply 10 GPS updates at the true position.
    utils::GpsMeasurement gps;
    gps.timestamp = 0.0;
    gps.position  = Eigen::Vector3d::Zero();  // truth is at origin

    for (int i = 0; i < 10; ++i) {
        filter.updateGPS(gps);
    }

    // After 10 updates, position estimate should converge to origin.
    EXPECT_LT(filter.nominalState().p.norm(), 0.01);
}

// ── Outlier rejection statistics ──────────────────────────────────────────────

TEST(ESKFIntegration, OutlierCountsAreCorrect) {
    ESKFConfig cfg;
    ESKF filter(cfg);

    NominalState x0;
    CovMatrix    P0 = initCovariance(cfg);
    P0 *= 1e-6;  // very tight covariance -> large residuals rejected
    filter.initialize(x0, P0, 0.0);

    // Good measurement (at origin = predicted position).
    utils::GpsMeasurement good_gps;
    good_gps.timestamp = 0.0;
    good_gps.position  = Eigen::Vector3d::Zero();
    filter.updateGPS(good_gps);

    // Bad measurement (100m away with tiny covariance -> rejected).
    utils::GpsMeasurement bad_gps;
    bad_gps.timestamp = 0.0;
    bad_gps.position  = Eigen::Vector3d(100.0, 0.0, 0.0);

    // Re-initialize with tight P for rejection test.
    filter.initialize(x0, P0 * 1e-6, 0.0);
    filter.updateGPS(bad_gps);

    // The bad one should be rejected.
    EXPECT_EQ(filter.gpsRejected(), 1);
}

// ── Covariance is PD after full pipeline step ─────────────────────────────────

TEST(ESKFIntegration, CovarianceRemainsPositiveDefiniteAfterUpdates) {
    ESKFConfig cfg;
    ESKF filter(cfg);

    NominalState x0;
    CovMatrix    P0 = initCovariance(cfg);
    P0 *= 100.0;
    filter.initialize(x0, P0, 0.0);

    const auto imu_data = makeStationaryImu(20, 0.005);

    utils::GpsMeasurement gps;
    gps.position = Eigen::Vector3d(0.01, -0.01, 0.0);

    for (size_t i = 1; i < imu_data.size(); ++i) {
        filter.predictIMU(imu_data[i], 0.005);
        gps.timestamp = imu_data[i].timestamp;
        filter.updateGPS(gps);
    }

    Eigen::SelfAdjointEigenSolver<CovMatrix> solver(filter.covariance());
    EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
}

// ── Error state is zero after update ─────────────────────────────────────────

TEST(ESKFIntegration, TimestampUpdatedAfterPredict) {
    ESKFConfig cfg;
    ESKF filter(cfg);
    filter.initialize(NominalState{}, initCovariance(cfg), 0.0);

    io::ImuMeasurement imu;
    imu.timestamp = 1.234;
    imu.gyro  = Eigen::Vector3d::Zero();
    imu.accel = Eigen::Vector3d(0.0, 0.0, 9.81);
    filter.predictIMU(imu, 0.005);

    EXPECT_NEAR(filter.timestamp(), 1.234, 1e-9);
}
