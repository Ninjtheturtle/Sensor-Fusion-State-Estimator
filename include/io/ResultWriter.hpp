#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <fstream>
#include <vector>
#include <stdexcept>

namespace eskf {
namespace io {

// One row of logged filter output.
struct TrajectoryEntry {
    double timestamp;
    Eigen::Vector3d p;   // position
    Eigen::Quaterniond q;// orientation
    Eigen::Vector3d v;   // velocity
    // Diagonal of P for position, velocity, and attitude blocks (9 values).
    Eigen::Vector3d P_pos_diag;
    Eigen::Vector3d P_vel_diag;
    Eigen::Vector3d P_att_diag;
};

// Writes filter trajectory output to a CSV file.
// Also retains entries in memory for in-process RMSE computation.
class ResultWriter {
public:
    // Opens (or creates) the output file. Writes CSV header.
    // Throws std::runtime_error if the file cannot be opened.
    explicit ResultWriter(const std::string& path);
    ~ResultWriter();

    // Append one timestamped state to the file.
    void write(const TrajectoryEntry& entry);

    // Returns all logged entries (for RMSE computation in main.cpp).
    const std::vector<TrajectoryEntry>& entries() const;

private:
    std::ofstream file_;
    std::vector<TrajectoryEntry> log_;
};

} // namespace io
} // namespace eskf
