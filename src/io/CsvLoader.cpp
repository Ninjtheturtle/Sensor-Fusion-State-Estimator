#include "io/CsvLoader.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>

namespace eskf {
namespace io {

// ── Private helpers ───────────────────────────────────────────────────────────

std::string CsvLoader::stripCR(std::string line) {
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }
    return line;
}

std::vector<std::string> CsvLoader::splitCSV(const std::string& line) {
    std::vector<std::string> tokens;
    std::istringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ',')) {
        // Trim leading/trailing whitespace from each token.
        auto start = token.find_first_not_of(" \t\r\n");
        auto end   = token.find_last_not_of(" \t\r\n");
        if (start != std::string::npos) {
            tokens.push_back(token.substr(start, end - start + 1));
        } else {
            tokens.push_back("");
        }
    }
    return tokens;
}

// ── IMU loader ────────────────────────────────────────────────────────────────
// EuRoC imu0/data.csv columns:
//   #timestamp [ns], w_RS_S_x, w_RS_S_y, w_RS_S_z, a_RS_S_x, a_RS_S_y, a_RS_S_z
// Note: GYRO comes first, then ACCEL.

std::vector<ImuMeasurement> CsvLoader::loadIMU(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("CsvLoader: cannot open IMU file: " + path);
    }

    std::vector<ImuMeasurement> data;
    std::string line;

    while (std::getline(file, line)) {
        line = stripCR(line);
        if (line.empty() || line[0] == '#') continue;

        auto tokens = splitCSV(line);
        if (tokens.size() < 7) continue;

        ImuMeasurement m;
        // Timestamp: nanoseconds -> seconds
        m.timestamp = std::stod(tokens[0]) * 1e-9;
        // Columns 1-3: gyroscope [rad/s]
        m.gyro.x()  = std::stod(tokens[1]);
        m.gyro.y()  = std::stod(tokens[2]);
        m.gyro.z()  = std::stod(tokens[3]);
        // Columns 4-6: accelerometer [m/s²]
        m.accel.x() = std::stod(tokens[4]);
        m.accel.y() = std::stod(tokens[5]);
        m.accel.z() = std::stod(tokens[6]);

        data.push_back(m);
    }

    return data;
}

// ── Ground truth loader ───────────────────────────────────────────────────────
// EuRoC state_groundtruth_estimate0/data.csv columns:
//   #timestamp, p_x, p_y, p_z, q_w, q_x, q_y, q_z,
//   v_x, v_y, v_z, b_w_x, b_w_y, b_w_z, b_a_x, b_a_y, b_a_z

std::vector<GroundTruthEntry> CsvLoader::loadGroundTruth(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("CsvLoader: cannot open GT file: " + path);
    }

    std::vector<GroundTruthEntry> data;
    std::string line;

    while (std::getline(file, line)) {
        line = stripCR(line);
        if (line.empty() || line[0] == '#') continue;

        auto tokens = splitCSV(line);
        if (tokens.size() < 17) continue;

        GroundTruthEntry e;
        // Timestamp: nanoseconds -> seconds
        e.timestamp = std::stod(tokens[0]) * 1e-9;

        // Position
        e.p.x() = std::stod(tokens[1]);
        e.p.y() = std::stod(tokens[2]);
        e.p.z() = std::stod(tokens[3]);

        // Quaternion (w, x, y, z) — Eigen Quaterniond constructor is (w, x, y, z)
        double qw = std::stod(tokens[4]);
        double qx = std::stod(tokens[5]);
        double qy = std::stod(tokens[6]);
        double qz = std::stod(tokens[7]);
        e.q = Eigen::Quaterniond(qw, qx, qy, qz).normalized();

        // Velocity
        e.v.x() = std::stod(tokens[8]);
        e.v.y() = std::stod(tokens[9]);
        e.v.z() = std::stod(tokens[10]);

        // Gyroscope bias
        e.b_gyro.x() = std::stod(tokens[11]);
        e.b_gyro.y() = std::stod(tokens[12]);
        e.b_gyro.z() = std::stod(tokens[13]);

        // Accelerometer bias
        e.b_accel.x() = std::stod(tokens[14]);
        e.b_accel.y() = std::stod(tokens[15]);
        e.b_accel.z() = std::stod(tokens[16]);

        data.push_back(e);
    }

    return data;
}

} // namespace io
} // namespace eskf
