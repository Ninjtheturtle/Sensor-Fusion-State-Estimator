#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <stdexcept>

namespace eskf {
namespace io {

// EuRoC imu0/data.csv row
// column order: timestamp_ns, wx, wy, wz, ax, ay, az (gyro FIRST, then accel)
struct ImuMeasurement {
    double timestamp;          // [s], converted from ns
    Eigen::Vector3d gyro;      // [rad/s], body frame
    Eigen::Vector3d accel;     // [m/s²], body frame
};

// EuRoC state_groundtruth_estimate0/data.csv row
// columns: timestamp_ns, px, py, pz, qw, qx, qy, qz, vx, vy, vz, bgx, bgy, bgz, bax, bay, baz
struct GroundTruthEntry {
    double timestamp;          // [s], converted from ns
    Eigen::Vector3d p;         // position [m], world frame
    Eigen::Quaterniond q;      // orientation (w, x, y, z)
    Eigen::Vector3d v;         // velocity [m/s], world frame
    Eigen::Vector3d b_gyro;    // gyro bias [rad/s]
    Eigen::Vector3d b_accel;   // accel bias [m/s²]
};

class CsvLoader {
public:
    // skips '#' header lines, converts ns -> s; throws on bad path
    static std::vector<ImuMeasurement> loadIMU(const std::string& path);

    static std::vector<GroundTruthEntry> loadGroundTruth(const std::string& path);

private:
    static std::vector<std::string> splitCSV(const std::string& line);

    // strips \r from Windows CRLF line endings
    static std::string stripCR(std::string line);
};

} // namespace io
} // namespace eskf
