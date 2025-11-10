#include "utils/MathUtils.hpp"
#include <Eigen/Cholesky>

namespace eskf {
namespace utils {

Eigen::Matrix3d safeInverse3x3(const Eigen::Matrix3d& M) {
    // Try Cholesky first (fastest, requires positive-definite input).
    Eigen::LLT<Eigen::Matrix3d> llt(M);
    if (llt.info() == Eigen::Success) {
        return llt.solve(Eigen::Matrix3d::Identity());
    }
    // Fall back to full-pivot LU (handles degenerate cases gracefully).
    return M.fullPivLu().solve(Eigen::Matrix3d::Identity());
}

} // namespace utils
} // namespace eskf
