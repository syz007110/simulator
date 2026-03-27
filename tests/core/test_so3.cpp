#include <gtest/gtest.h>
#include "core/math/SO3.hpp"
#include <cmath>

using namespace simrobot::math;
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

// ── hat / vee ──────────────────────────────────────────────────────────────

TEST(SO3, HatProducesSkewSymmetricMatrix) {
    Vector3d omega(1.0, 2.0, 3.0);
    Matrix3d skew = SO3::hat(omega);

    // 反对称性：M + M^T = 0
    EXPECT_NEAR((skew + skew.transpose()).norm(), 0.0, 1e-12);
    // 对角线为零
    EXPECT_NEAR(skew(0,0), 0.0, 1e-12);
    EXPECT_NEAR(skew(1,1), 0.0, 1e-12);
    EXPECT_NEAR(skew(2,2), 0.0, 1e-12);
}

TEST(SO3, HatVeeRoundTrip) {
    // vee(hat(ω)) == ω
    Vector3d omega(1.5, -2.3, 0.7);
    EXPECT_NEAR((SO3::vee(SO3::hat(omega)) - omega).norm(), 0.0, 1e-12);
}

TEST(SO3, HatEquivalentToCrossProduct) {
    // [ω]× v == ω × v
    Vector3d omega(1.0, 0.0, 0.0);
    Vector3d v(0.0, 1.0, 0.0);
    Vector3d cross = omega.cross(v);
    Vector3d hat_v = SO3::hat(omega) * v;
    EXPECT_NEAR((cross - hat_v).norm(), 0.0, 1e-12);
}

// ── exp 映射 ───────────────────────────────────────────────────────────────

TEST(SO3, ExpIdentityForZeroVector) {
    SO3 R = SO3::exp(Vector3d::Zero());
    EXPECT_NEAR((R.matrix() - Matrix3d::Identity()).norm(), 0.0, 1e-12);
}

TEST(SO3, ExpProducesValidRotationMatrix) {
    // 绕 z 轴旋转 45°
    Vector3d omega(0.0, 0.0, M_PI / 4.0);
    SO3 R = SO3::exp(omega);
    EXPECT_TRUE(R.isValid());
}

TEST(SO3, ExpRotationAboutX90Degrees) {
    // 绕 x 轴旋转 90°：y → z，z → -y
    SO3 R = SO3::exp(Vector3d(M_PI / 2.0, 0.0, 0.0));
    Vector3d y(0.0, 1.0, 0.0);
    Vector3d z(0.0, 0.0, 1.0);
    EXPECT_NEAR((R * y - z).norm(), 0.0, 1e-9);
}

TEST(SO3, ExpRotationAboutZ180Degrees) {
    // 绕 z 轴旋转 180°：x → -x，y → -y
    SO3 R = SO3::exp(Vector3d(0.0, 0.0, M_PI));
    Vector3d x(1.0, 0.0, 0.0);
    EXPECT_NEAR((R * x - Vector3d(-1.0, 0.0, 0.0)).norm(), 0.0, 1e-9);
}

// ── log 映射 ───────────────────────────────────────────────────────────────

TEST(SO3, LogOfIdentityIsZero) {
    EXPECT_NEAR(SO3::Identity().log().norm(), 0.0, 1e-12);
}

TEST(SO3, ExpLogRoundTrip) {
    // log(exp(ω)) == ω，对任意小角度成立
    Vector3d omega(0.3, -0.5, 0.8);
    Vector3d recovered = SO3::exp(omega).log();
    EXPECT_NEAR((recovered - omega).norm(), 0.0, 1e-9);
}

TEST(SO3, LogExpRoundTrip) {
    // exp(log(R)) == R
    SO3 R = SO3::exp(Vector3d(0.1, 0.4, -0.6));
    SO3 recovered = SO3::exp(R.log());
    EXPECT_NEAR((recovered.matrix() - R.matrix()).norm(), 0.0, 1e-9);
}

TEST(SO3, LogRecoversThetaAsNorm) {
    // log(R) 的模长 == 旋转角 θ
    double theta = 1.2;
    Vector3d axis(1.0, 0.0, 0.0);
    SO3 R = SO3::exp(theta * axis);
    EXPECT_NEAR(R.log().norm(), theta, 1e-9);
}

// ── 群运算 ─────────────────────────────────────────────────────────────────

TEST(SO3, InverseIsTranspose) {
    SO3 R = SO3::exp(Vector3d(0.3, -0.2, 0.7));
    EXPECT_NEAR((R.matrix() * R.inverse().matrix() - Matrix3d::Identity()).norm(), 0.0, 1e-12);
}

TEST(SO3, CompositionIsAssociative) {
    SO3 R1 = SO3::exp(Vector3d(0.1, 0.2, 0.3));
    SO3 R2 = SO3::exp(Vector3d(-0.4, 0.1, 0.5));
    SO3 R3 = SO3::exp(Vector3d(0.2, -0.3, 0.1));
    SO3 lhs = (R1 * R2) * R3;
    SO3 rhs = R1 * (R2 * R3);
    EXPECT_NEAR((lhs.matrix() - rhs.matrix()).norm(), 0.0, 1e-12);
}

TEST(SO3, CompositionPreservesValidity) {
    SO3 R1 = SO3::exp(Vector3d(1.0, 0.5, -0.3));
    SO3 R2 = SO3::exp(Vector3d(-0.2, 0.8, 0.1));
    EXPECT_TRUE((R1 * R2).isValid());
}

// ── 边界情况 ───────────────────────────────────────────────────────────────

TEST(SO3, ExpLogNearPi) {
    // θ ≈ π 时的数值稳定性
    Vector3d omega(M_PI * 0.9999, 0.0, 0.0);
    SO3 R = SO3::exp(omega);
    EXPECT_TRUE(R.isValid());
    // round-trip 误差在 π 附近允许稍大
    EXPECT_NEAR(R.log().norm(), omega.norm(), 1e-4);
}
