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

// acceptance / rejection

TEST(GpsUpdater, PerfectMeasurementAccepted) {
    ESKFConfig cfg;
    GpsUpdater updater(cfg);

    NominalState x;
    ErrorState   dx;
    CovMatrix    P = initCovariance(cfg);

    // zero residual — always accepted
    const auto gps = makeGps(x.p);
    EXPECT_TRUE(updater.update(x, dx, P, gps));
}

TEST(GpsUpdater, LargeOutlierRejected) {
    ESKFConfig cfg;
    GpsUpdater updater(cfg);

    NominalState x;
    ErrorState   dx;
    CovMatrix    P = initCovariance(cfg);
    // tiny P means huge Mahalanobis distance
    P *= 1e-6;

    const auto gps = makeGps(Eigen::Vector3d(100.0, 0.0, 0.0));
    EXPECT_FALSE(updater.update(x, dx, P, gps));
}

// covariance reduction

TEST(GpsUpdater, CovarianceReducedAfterUpdate) {
    ESKFConfig cfg;
    GpsUpdater updater(cfg);

    NominalState x;
    ErrorState   dx;
    CovMatrix    P = initCovariance(cfg);

    // inflate so update has a meaningful effect
    P *= 100.0;
    const double trace_before = P.trace();

    const auto gps = makeGps(x.p + Eigen::Vector3d(0.01, 0.01, 0.01));
    updater.update(x, dx, P, gps);

    EXPECT_LT(P.trace(), trace_before);
}

// post-update properties

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
    ESKFConfig cfg;
    GpsUpdater updater(cfg);

    NominalState x;
    ErrorState   dx;
    CovMatrix    P = initCovariance(cfg);
    P *= 1000.0;  // strong update

    // small offset so d² << threshold; 1m would get rejected here
    const Eigen::Vector3d gps_pos(0.1, 0.0, 0.0);
    const auto gps = makeGps(gps_pos);
    updater.update(x, dx, P, gps);

    EXPECT_GT(x.p.x(), 0.0);
    EXPECT_LT(x.p.x(), 0.1);
}
