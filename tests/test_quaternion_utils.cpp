#include <gtest/gtest.h>
#include "utils/QuaternionUtils.hpp"
#include <Eigen/Core>
#include <cmath>

using namespace eskf::utils;

static constexpr double kTol = 1e-9;
static constexpr double kLooseTol = 1e-6;

// skew() tests

TEST(SkewMatrix, IsAntisymmetric) {
    const Eigen::Vector3d v(1.0, 2.0, 3.0);
    const Eigen::Matrix3d S = skew(v);
    EXPECT_LT((S + S.transpose()).norm(), kTol);
}

TEST(SkewMatrix, SkewTimesVecIsZero) {
    const Eigen::Vector3d v(1.0, -2.0, 3.0);
    // skew(v) * v == 0 always
    EXPECT_LT((skew(v) * v).norm(), kTol);
}

TEST(SkewMatrix, CrossProductProperty) {
    const Eigen::Vector3d a(1.0, 0.0, 0.0);
    const Eigen::Vector3d b(0.0, 1.0, 0.0);
    // skew(a) * b == a.cross(b)
    EXPECT_LT((skew(a) * b - a.cross(b)).norm(), kTol);
}

// quatToRot() tests

TEST(QuatToRot, IdentityGivesI3) {
    const Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    const Eigen::Matrix3d R = quatToRot(q);
    EXPECT_LT((R - Eigen::Matrix3d::Identity()).norm(), kTol);
}

TEST(QuatToRot, IsOrthogonal) {
    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(0.5, Eigen::Vector3d(1, 1, 0).normalized()));
    const Eigen::Matrix3d R = quatToRot(q);
    EXPECT_LT((R.transpose() * R - Eigen::Matrix3d::Identity()).norm(), kTol);
}

TEST(QuatToRot, Determinant1) {
    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(1.2, Eigen::Vector3d(0, 0, 1).normalized()));
    const Eigen::Matrix3d R = quatToRot(q);
    EXPECT_NEAR(R.determinant(), 1.0, kTol);
}

TEST(QuatToRot, MatchesEigenMethod) {
    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()));
    const Eigen::Matrix3d R1 = quatToRot(q);
    const Eigen::Matrix3d R2 = q.toRotationMatrix();
    EXPECT_LT((R1 - R2).norm(), kTol);
}

// rotVecToQuat() tests

TEST(RotVecToQuat, ZeroVectorGivesIdentity) {
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    const Eigen::Quaterniond q = rotVecToQuat(zero);
    EXPECT_NEAR(q.w(), 1.0, kLooseTol);
    EXPECT_LT(q.vec().norm(), kLooseTol);
}

TEST(RotVecToQuat, OutputIsUnitQuaternion) {
    const Eigen::Vector3d v(0.1, -0.2, 0.3);
    const Eigen::Quaterniond q = rotVecToQuat(v);
    EXPECT_NEAR(q.norm(), 1.0, kTol);
}

TEST(RotVecToQuat, Pi_X_Rotation) {
    // 180 degree rotation about X axis: q = [0, 1, 0, 0]
    const Eigen::Vector3d v(M_PI, 0.0, 0.0);
    const Eigen::Quaterniond q = rotVecToQuat(v);
    EXPECT_NEAR(q.w(), 0.0, kLooseTol);
    EXPECT_NEAR(std::abs(q.x()), 1.0, kLooseTol);
    EXPECT_NEAR(q.y(), 0.0, kLooseTol);
    EXPECT_NEAR(q.z(), 0.0, kLooseTol);
}

TEST(RotVecToQuat, NumericalStabilityNearZero) {
    // exercises the Taylor expansion branch
    const Eigen::Vector3d v(1e-12, 0.0, 0.0);
    const Eigen::Quaterniond q = rotVecToQuat(v);
    EXPECT_TRUE(std::isfinite(q.w()));
    EXPECT_TRUE(std::isfinite(q.x()));
    EXPECT_NEAR(q.norm(), 1.0, kTol);
}

// quatToRotVec() roundtrip

TEST(QuatToRotVec, RoundtripSmallAngle) {
    const Eigen::Vector3d v_in(0.01, -0.02, 0.03);
    const Eigen::Quaterniond q = rotVecToQuat(v_in);
    const Eigen::Vector3d v_out = quatToRotVec(q);
    EXPECT_LT((v_in - v_out).norm(), kLooseTol);
}

TEST(QuatToRotVec, RoundtripLargeAngle) {
    const Eigen::Vector3d v_in(1.0, -0.5, 0.8);
    const Eigen::Quaterniond q = rotVecToQuat(v_in);
    const Eigen::Vector3d v_out = quatToRotVec(q);
    EXPECT_LT((v_in - v_out).norm(), kLooseTol);
}

TEST(QuatToRotVec, NearIdentityStable) {
    const Eigen::Quaterniond q(1.0, 1e-13, 0.0, 0.0);
    const Eigen::Vector3d v = quatToRotVec(q.normalized());
    EXPECT_TRUE(std::isfinite(v.x()));
    EXPECT_TRUE(std::isfinite(v.y()));
    EXPECT_TRUE(std::isfinite(v.z()));
}

// applyRotVecRight() tests

TEST(ApplyRotVecRight, ZeroErrorPreservesQuaternion) {
    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond q2 = applyRotVecRight(q, Eigen::Vector3d::Zero());
    EXPECT_LT((q.coeffs() - q2.coeffs()).norm(), kLooseTol);
}

TEST(ApplyRotVecRight, OutputIsUnitQuaternion) {
    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(1.0, Eigen::Vector3d(1, 1, 1).normalized()));
    const Eigen::Vector3d dtheta(0.01, -0.02, 0.005);
    const Eigen::Quaterniond q2 = applyRotVecRight(q, dtheta);
    EXPECT_NEAR(q2.norm(), 1.0, kTol);
}
