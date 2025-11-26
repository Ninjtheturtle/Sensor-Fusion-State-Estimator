#pragma once

// Full definition required because std::vector<T> needs a complete type.
#include "io/CsvLoader.hpp"
#include <Eigen/Core>
#include <vector>
#include <random>

namespace eskf {
namespace utils {

// Measurement types generated from ground truth.
struct GpsMeasurement {
    double timestamp;          // seconds
    Eigen::Vector3d position;  // [m] in world frame, with added noise
};

struct OdomMeasurement {
    double timestamp;          // seconds
    Eigen::Vector3d velocity;  // [m/s] in world frame, with added noise
};

// Generates simulated GPS and odometry measurements from EuRoC ground truth.
// Both sensors are simulated by subsampling the 200 Hz ground truth stream
// and adding independent Gaussian noise to each sample.
class NoiseSim {
public:
    // seed: RNG seed for reproducibility (42 gives consistent outlier patterns)
    explicit NoiseSim(unsigned int seed = 42);

    // Generate simulated GPS measurements from ground truth positions.
    // rate_hz:   desired GPS output rate (e.g., 10.0)
    // sigma_xy:  horizontal position noise std dev [m] (e.g., 0.05)
    // sigma_z:   vertical position noise std dev [m] (e.g., 0.10)
    std::vector<GpsMeasurement> generateGPS(
        const std::vector<eskf::io::GroundTruthEntry>& gt,
        double rate_hz,
        double sigma_xy,
        double sigma_z);

    // Generate simulated odometry (velocity) measurements from ground truth.
    // rate_hz: desired odometry output rate (e.g., 10.0)
    // sigma:   velocity noise std dev [m/s] (applied to all three axes)
    std::vector<OdomMeasurement> generateOdometry(
        const std::vector<eskf::io::GroundTruthEntry>& gt,
        double rate_hz,
        double sigma);

private:
    std::mt19937 rng_;
    std::normal_distribution<double> gauss_;  // N(0,1), scaled at call site
};

} // namespace utils
} // namespace eskf
