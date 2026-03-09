#include "utils/MathUtils.hpp"
#include <Eigen/Cholesky>

namespace eskf {
namespace utils {

Eigen::Matrix3d safeInverse3x3(const Eigen::Matrix3d& M) {
    // Cholesky is fastest when M is PD (which it should be for innovation covariance)
    Eigen::LLT<Eigen::Matrix3d> llt(M);
    if (llt.info() == Eigen::Success) {
        return llt.solve(Eigen::Matrix3d::Identity());
    }
    // fallback for degenerate cases — shouldn't happen in practice
    return M.fullPivLu().solve(Eigen::Matrix3d::Identity());
}

} // namespace utils
} // namespace eskf
