#include "utils/NoiseSim.hpp"
#include "io/CsvLoader.hpp"
#include <cmath>
#include <stdexcept>

namespace eskf {
namespace utils {

NoiseSim::NoiseSim(unsigned int seed)
    : rng_(seed)
    , gauss_(0.0, 1.0)
{}

std::vector<GpsMeasurement> NoiseSim::generateGPS(
    const std::vector<io::GroundTruthEntry>& gt,
    double rate_hz,
    double sigma_xy,
    double sigma_z)
{
    if (gt.empty()) {
        return {};
    }

    std::vector<GpsMeasurement> result;

    // Subsample from 200 Hz GT stream by computing skip interval.
    // EuRoC GT is nominally 200 Hz (dt ≈ 0.005 s).
    // Skip interval = round(200 / rate_hz).
    const double gt_rate_hz = 200.0;
    const int skip = static_cast<int>(std::round(gt_rate_hz / rate_hz));
    if (skip < 1) {
        throw std::invalid_argument("GPS rate_hz exceeds GT rate");
    }

    for (size_t i = 0; i < gt.size(); i += static_cast<size_t>(skip)) {
        GpsMeasurement m;
        m.timestamp = gt[i].timestamp;
        m.position.x() = gt[i].p.x() + sigma_xy * gauss_(rng_);
        m.position.y() = gt[i].p.y() + sigma_xy * gauss_(rng_);
        m.position.z() = gt[i].p.z() + sigma_z  * gauss_(rng_);
        result.push_back(m);
    }

    return result;
}

std::vector<OdomMeasurement> NoiseSim::generateOdometry(
    const std::vector<io::GroundTruthEntry>& gt,
    double rate_hz,
    double sigma)
{
    if (gt.empty()) {
        return {};
    }

    std::vector<OdomMeasurement> result;

    const double gt_rate_hz = 200.0;
    const int skip = static_cast<int>(std::round(gt_rate_hz / rate_hz));
    if (skip < 1) {
        throw std::invalid_argument("Odom rate_hz exceeds GT rate");
    }

    for (size_t i = 0; i < gt.size(); i += static_cast<size_t>(skip)) {
        OdomMeasurement m;
        m.timestamp = gt[i].timestamp;
        m.velocity.x() = gt[i].v.x() + sigma * gauss_(rng_);
        m.velocity.y() = gt[i].v.y() + sigma * gauss_(rng_);
        m.velocity.z() = gt[i].v.z() + sigma * gauss_(rng_);
        result.push_back(m);
    }

    return result;
}

} // namespace utils
} // namespace eskf
