#include <gtest/gtest.h>
#include "eskf/GpsUpdater.hpp"
#include "eskf/State.hpp"
#include "utils/NoiseSim.hpp"

using namespace eskf;

static utils::GpsMeasurement makeGps(const Eigen::Vector3d& pos) {
    utils::GpsMeasurement m;
    m.timestamp = 0.0;
    m.position  = pos;
    return m;
}

// ── Acceptance / rejection tests ──────────────────────────────────────────────

TEST(GpsUpdater, PerfectMeasurementAccepted) {
    ESKFConfig cfg;
    GpsUpdater updater(cfg);

    NominalState x;  // position at origin
    ErrorState   dx;
    CovMatrix    P = initCovariance(cfg);

    // GPS exactly matches predicted position -> zero residual -> always accepted.
    const auto gps = makeGps(x.p);
    EXPECT_TRUE(updater.update(x, dx, P, gps));
}

TEST(GpsUpdater, LargeOutlierRejected) {
    ESKFConfig cfg;
    GpsUpdater updater(cfg);

    NominalState x;
    ErrorState   dx;
    CovMatrix    P = initCovariance(cfg);
    // Make P tiny so Mahalanobis distance of large residual is huge.
    P *= 1e-6;

    // 100m offset with tiny covariance -> massive d^2 -> rejected.
    const auto gps = makeGps(Eigen::Vector3d(100.0, 0.0, 0.0));
    EXPECT_FALSE(updater.update(x, dx, P, gps));
}

// ── Covariance reduction ──────────────────────────────────────────────────────

TEST(GpsUpdater, CovarianceReducedAfterUpdate) {
    ESKFConfig cfg;
    GpsUpdater updater(cfg);

    NominalState x;
    ErrorState   dx;
    CovMatrix    P = initCovariance(cfg);

    // Inflate initial covariance so update has a meaningful effect.
    P *= 100.0;
    const double trace_before = P.trace();

    const auto gps = makeGps(x.p + Eigen::Vector3d(0.01, 0.01, 0.01));
    updater.update(x, dx, P, gps);

    EXPECT_LT(P.trace(), trace_before);
}

// ── Post-update properties ────────────────────────────────────────────────────

TEST(GpsUpdater, CovarianceSymmetricAfterUpdate) {
    ESKFConfig cfg;
    GpsUpdater updater(cfg);

    NominalState x;
    ErrorState   dx;
    CovMatrix    P = initCovariance(cfg);

    const auto gps = makeGps(x.p + Eigen::Vector3d(0.02, -0.01, 0.05));
    updater.update(x, dx, P, gps);

    EXPECT_LT((P - P.transpose()).norm(), 1e-10);
}

TEST(GpsUpdater, ErrorStateZeroAfterInject) {
    // After update, injectAndReset() should zero dx.delta.
    ESKFConfig cfg;
    GpsUpdater updater(cfg);

    NominalState x;
    ErrorState   dx;
    CovMatrix    P = initCovariance(cfg);
    P *= 100.0;

    const auto gps = makeGps(Eigen::Vector3d(0.1, 0.0, 0.0));
    updater.update(x, dx, P, gps);

    EXPECT_LT(dx.delta.norm(), 1e-15);
}

TEST(GpsUpdater, PositionMovesTowardMeasurement) {
    // After update, estimated position should move toward GPS measurement.
    ESKFConfig cfg;
    GpsUpdater updater(cfg);

    NominalState x;  // position at [0,0,0]
    ErrorState   dx;
    CovMatrix    P = initCovariance(cfg);
    P *= 1000.0;  // large uncertainty -> strong update

    // Use a small offset (0.1m) so Mahalanobis d² ≈ 0.098 << 7.815 threshold.
    // A 1m offset with this P gives d² ≈ 9.76 which is correctly rejected.
    const Eigen::Vector3d gps_pos(0.1, 0.0, 0.0);
    const auto gps = makeGps(gps_pos);
    updater.update(x, dx, P, gps);

    // x.p should be between 0 and gps_pos (Kalman convex combination).
    EXPECT_GT(x.p.x(), 0.0);
    EXPECT_LT(x.p.x(), 0.1);
}
