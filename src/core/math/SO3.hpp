#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

namespace simrobot::math {

/**
 * SO3 — 三维旋转群（Special Orthogonal Group）
 *
 * SO(3) = { R ∈ R^{3×3} | R^T R = I, det(R) = +1 }
 *
 * 旋转矩阵 R 满足：
 *   - 列向量两两正交且均为单位向量
 *   - 行列式为 +1（区别于反射变换 det = -1）
 *
 * 对应的李代数 so(3) 是所有 3×3 反对称矩阵的集合：
 *   so(3) = { [ω]× ∈ R^{3×3} | [ω]×^T = -[ω]× }
 *
 * 参考：《Modern Robotics》Ch3，Lynch & Park
 */
class SO3 {
public:
    using Matrix3d = Eigen::Matrix3d;
    using Vector3d = Eigen::Vector3d;

    // ── 构造 ───────────────────────────────────────────────────────────────

    /// 默认构造：单位旋转（无旋转）
    SO3() : R_(Matrix3d::Identity()) {}

    /// 从旋转矩阵构造（不做归一化，调用者负责保证合法性）
    explicit SO3(const Matrix3d& R) : R_(R) {}

    /// 单位旋转
    static SO3 Identity() { return SO3{}; }

    // ── hat / vee 算子 ─────────────────────────────────────────────────────

    /**
     * hat 算子：将旋转向量 ω ∈ R³ 映射为反对称矩阵 [ω]× ∈ so(3)
     *
     *          [  0  -ω₃  ω₂ ]
     * [ω]×  =  [ ω₃   0  -ω₁ ]
     *          [-ω₂  ω₁   0  ]
     *
     * 性质：[ω]× v = ω × v（叉积）
     */
    static Matrix3d hat(const Vector3d& omega) {
        Matrix3d skew;
        skew <<      0.0, -omega(2),  omega(1),
               omega(2),       0.0, -omega(0),
              -omega(1),  omega(0),       0.0;
        return skew;
    }

    /**
     * vee 算子：hat 的逆，从反对称矩阵提取旋转向量
     *
     * 输入矩阵必须是反对称的（仅在 Debug 模式检查）
     */
    static Vector3d vee(const Matrix3d& skew) {
        return Vector3d(skew(2, 1), skew(0, 2), skew(1, 0));
    }

    // ── 指数映射：so(3) → SO(3) ────────────────────────────────────────────

    /**
     * exp 映射（Rodrigues 旋转公式）
     *
     * 给定旋转向量 ω ∈ R³，其中：
     *   - 方向 ω̂ = ω / ‖ω‖  表示旋转轴
     *   - 模长 θ = ‖ω‖       表示旋转角（rad）
     *
     * Rodrigues 公式：
     *   R = I + sin(θ)[ω̂]× + (1 - cos(θ))[ω̂]×²
     *
     * 特殊情况：θ ≈ 0 时退化为单位矩阵
     */
    static SO3 exp(const Vector3d& omega) {
        const double theta = omega.norm();

        if (theta < 1e-10) {
            return SO3::Identity();
        }

        const Vector3d axis = omega / theta;       // 单位旋转轴 ω̂
        const Matrix3d K    = hat(axis);           // [ω̂]×
        const Matrix3d R    = Matrix3d::Identity()
                            + std::sin(theta) * K
                            + (1.0 - std::cos(theta)) * K * K;
        return SO3(R);
    }

    // ── 对数映射：SO(3) → so(3) ────────────────────────────────────────────

    /**
     * log 映射：将旋转矩阵还原为旋转向量 ω ∈ R³
     *
     * 由矩阵迹(trace)恢复旋转角：
     *   cos(θ) = (tr(R) - 1) / 2
     *
     * 旋转轴由反对称部分恢复：
     *   [ω̂]× = (R - R^T) / (2 sin(θ))
     *
     * 特殊情况：
     *   - θ ≈ 0：R ≈ I，返回零向量
     *   - θ ≈ π：公式退化，需用特殊处理（取 R 对角元素最大列）
     *
     * 返回值 ω = θ · ω̂，模长即旋转角（rad），范围 [0, π]
     */
    Vector3d log() const {
        const double cos_theta = (R_.trace() - 1.0) / 2.0;
        // 数值夹紧，防止浮点误差导致 acos 域外
        const double theta = std::acos(std::clamp(cos_theta, -1.0, 1.0));

        // 情况1：θ ≈ 0，近似为零旋转
        if (theta < 1e-10) {
            return Vector3d::Zero();
        }

        // 情况2：θ ≈ π，标准公式的分母 sin(θ) → 0
        if (std::abs(theta - M_PI) < 1e-7) {
            return _logNearPi();
        }

        // 标准情况
        const Matrix3d skew = (theta / (2.0 * std::sin(theta))) * (R_ - R_.transpose());
        return vee(skew);
    }

    // ── 群运算 ─────────────────────────────────────────────────────────────

    /// 旋转复合：R₁ * R₂
    SO3 operator*(const SO3& other) const {
        return SO3(R_ * other.R_);
    }

    /// 旋转作用于向量：R * v
    Vector3d operator*(const Vector3d& v) const {
        return R_ * v;
    }

    /// 逆旋转（等于转置）：R^{-1} = R^T
    SO3 inverse() const {
        return SO3(R_.transpose());
    }

    // ── 访问 ───────────────────────────────────────────────────────────────

    const Matrix3d& matrix() const { return R_; }

    /// 数值有效性检查：R^T R ≈ I 且 det(R) ≈ +1
    bool isValid(double tol = 1e-9) const {
        const double ortho_err = (R_.transpose() * R_ - Matrix3d::Identity()).norm();
        const double det_err   = std::abs(R_.determinant() - 1.0);
        return ortho_err < tol && det_err < tol;
    }

private:
    Matrix3d R_;

    /// θ ≈ π 时的对数映射（分母退化的特殊处理）
    Vector3d _logNearPi() const {
        // 从 R 的对角元素找最大分量，避免除以接近零的数
        const Matrix3d B = (R_ + Matrix3d::Identity()) / 2.0;  // (R+I)/2 = ω̂ω̂^T 当 θ=π
        int   max_idx = 0;
        double max_val = B(0, 0);
        for (int i = 1; i < 3; ++i) {
            if (B(i, i) > max_val) { max_val = B(i, i); max_idx = i; }
        }
        Vector3d axis = B.col(max_idx) / std::sqrt(max_val);
        return M_PI * axis;
    }
};

}  // namespace simrobot::math
