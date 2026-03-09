#pragma once

// full include required — std::vector needs complete type for GroundTruthEntry
#include "io/CsvLoader.hpp"
#include <Eigen/Core>
#include <vector>
#include <random>

namespace eskf {
namespace utils {

struct GpsMeasurement {
    double timestamp;          // seconds
    Eigen::Vector3d position;  // [m] world frame, noise added
};

struct OdomMeasurement {
    double timestamp;          // seconds
    Eigen::Vector3d velocity;  // [m/s] world frame, noise added
};

// simulates GPS and odom from EuRoC ground truth by subsampling + adding Gaussian noise
class NoiseSim {
public:
    // seed 42 for reproducible outlier patterns
    explicit NoiseSim(unsigned int seed = 42);

    std::vector<GpsMeasurement> generateGPS(
        const std::vector<eskf::io::GroundTruthEntry>& gt,
        double rate_hz,
        double sigma_xy,
        double sigma_z);

    std::vector<OdomMeasurement> generateOdometry(
        const std::vector<eskf::io::GroundTruthEntry>& gt,
        double rate_hz,
        double sigma);

private:
    std::mt19937 rng_;
    std::normal_distribution<double> gauss_;  // N(0,1), scaled per-call
};

} // namespace utils
} // namespace eskf
