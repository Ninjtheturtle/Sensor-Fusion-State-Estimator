#include <gtest/gtest.h>
#include "utils/MahalanobisGate.hpp"
#include <Eigen/Core>
#include <cmath>

using namespace eskf::utils;

static constexpr double kTol = 1e-9;

// ── Basic gate tests ──────────────────────────────────────────────────────────

TEST(MahalanobisGate, ZeroResidualAlwaysAccepted) {
    const Eigen::Vector3d r = Eigen::Vector3d::Zero();
    const Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
    double d2;
    EXPECT_TRUE(mahalanobisGate<3>(r, S, ChiSquaredThreshold::kDof3, &d2));
    EXPECT_NEAR(d2, 0.0, kTol);
}

TEST(MahalanobisGate, KnownMahalanobisDistance) {
    // r = [1, 0, 0], S = I => d^2 = 1^2 = 1 < 7.815
    const Eigen::Vector3d r(1.0, 0.0, 0.0);
    const Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
    double d2;
    EXPECT_TRUE(mahalanobisGate<3>(r, S, ChiSquaredThreshold::kDof3, &d2));
    EXPECT_NEAR(d2, 1.0, kTol);
}

TEST(MahalanobisGate, ScaledCovariance) {
    // r = [1, 0, 0], S = 4*I => d^2 = 1/4 = 0.25
    const Eigen::Vector3d r(1.0, 0.0, 0.0);
    const Eigen::Matrix3d S = 4.0 * Eigen::Matrix3d::Identity();
    double d2;
    EXPECT_TRUE(mahalanobisGate<3>(r, S, ChiSquaredThreshold::kDof3, &d2));
    EXPECT_NEAR(d2, 0.25, kTol);
}

TEST(MahalanobisGate, LargeOutlierRejected) {
    // 10-sigma outlier: d^2 = 100 >> 7.815
    const Eigen::Vector3d r(10.0, 0.0, 0.0);
    const Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
    double d2;
    EXPECT_FALSE(mahalanobisGate<3>(r, S, ChiSquaredThreshold::kDof3, &d2));
    EXPECT_NEAR(d2, 100.0, kTol);
}

TEST(MahalanobisGate, ExactlyAtThresholdAccepted) {
    // d^2 == threshold should be accepted (<=)
    const double threshold = ChiSquaredThreshold::kDof3;  // 7.815
    // Set r such that d^2 = threshold: r = [sqrt(7.815), 0, 0], S = I
    const Eigen::Vector3d r(std::sqrt(threshold), 0.0, 0.0);
    const Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
    double d2;
    EXPECT_TRUE(mahalanobisGate<3>(r, S, threshold, &d2));
    EXPECT_NEAR(d2, threshold, 1e-10);
}

TEST(MahalanobisGate, JustAboveThresholdRejected) {
    const double threshold = ChiSquaredThreshold::kDof3;
    const Eigen::Vector3d r(std::sqrt(threshold + 0.01), 0.0, 0.0);
    const Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
    EXPECT_FALSE(mahalanobisGate<3>(r, S, threshold, nullptr));
}

TEST(MahalanobisGate, DiagonalCovariance) {
    // S = diag(1, 4, 9), r = [1, 2, 3]
    // d^2 = 1/1 + 4/4 + 9/9 = 1 + 1 + 1 = 3
    Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
    S(0,0) = 1.0; S(1,1) = 4.0; S(2,2) = 9.0;
    const Eigen::Vector3d r(1.0, 2.0, 3.0);
    double d2;
    mahalanobisGate<3>(r, S, 100.0, &d2);
    EXPECT_NEAR(d2, 3.0, 1e-9);
}

// ── Chi-squared threshold sanity ──────────────────────────────────────────────

TEST(ChiSquaredThreshold, Dof3IsCorrect) {
    // chi^2(3, 0.05) = 7.815 (standard table value)
    EXPECT_NEAR(ChiSquaredThreshold::kDof3, 7.815, 0.001);
}

TEST(ChiSquaredThreshold, Dof1IsCorrect) {
    // chi^2(1, 0.05) = 3.841
    EXPECT_NEAR(ChiSquaredThreshold::kDof1, 3.841, 0.001);
}
