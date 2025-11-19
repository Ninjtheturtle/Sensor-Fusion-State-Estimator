#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <stdexcept>

namespace eskf {
namespace io {

// Raw IMU measurement from EuRoC imu0/data.csv.
// EuRoC column order: timestamp_ns, wx, wy, wz, ax, ay, az
// Note: gyroscope comes BEFORE accelerometer in the EuRoC IMU CSV.
struct ImuMeasurement {
    double timestamp;          // [s], converted from nanoseconds
    Eigen::Vector3d gyro;      // [rad/s], body frame
    Eigen::Vector3d accel;     // [m/s²], body frame
};

// Ground truth pose from EuRoC state_groundtruth_estimate0/data.csv.
// EuRoC column order:
//   timestamp_ns, px, py, pz, qw, qx, qy, qz, vx, vy, vz,
//   bgx, bgy, bgz, bax, bay, baz
struct GroundTruthEntry {
    double timestamp;          // [s], converted from nanoseconds
    Eigen::Vector3d p;         // position [m], world frame
    Eigen::Quaterniond q;      // orientation (w, x, y, z)
    Eigen::Vector3d v;         // velocity [m/s], world frame
    Eigen::Vector3d b_gyro;    // gyroscope bias [rad/s]
    Eigen::Vector3d b_accel;   // accelerometer bias [m/s²]
};

class CsvLoader {
public:
    // Load IMU data from EuRoC imu0/data.csv.
    // Skips lines starting with '#'. Converts timestamps ns -> s.
    // Throws std::runtime_error if the file cannot be opened.
    static std::vector<ImuMeasurement> loadIMU(const std::string& path);

    // Load ground truth from EuRoC state_groundtruth_estimate0/data.csv.
    // Skips lines starting with '#'. Converts timestamps ns -> s.
    // Throws std::runtime_error if the file cannot be opened.
    static std::vector<GroundTruthEntry> loadGroundTruth(const std::string& path);

private:
    // Split a CSV line into tokens. Handles both "," and ", " separators.
    static std::vector<std::string> splitCSV(const std::string& line);

    // Strip trailing '\r' (Windows CRLF line endings in CSV files).
    static std::string stripCR(std::string line);
};

} // namespace io
} // namespace eskf
