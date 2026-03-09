#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <fstream>
#include <vector>
#include <stdexcept>

namespace eskf {
namespace io {

struct TrajectoryEntry {
    double timestamp;
    Eigen::Vector3d p;   // position
    Eigen::Quaterniond q;// orientation
    Eigen::Vector3d v;   // velocity
    // diagonal of P for pos/vel/att blocks (logged for uncertainty visualization)
    Eigen::Vector3d P_pos_diag;
    Eigen::Vector3d P_vel_diag;
    Eigen::Vector3d P_att_diag;
};

// writes filter output to CSV; also keeps entries in memory for RMSE at the end
class ResultWriter {
public:
    // opens/creates the file and writes the header; throws if path is bad
    explicit ResultWriter(const std::string& path);
    ~ResultWriter();

    void write(const TrajectoryEntry& entry);

    // used by main to compute RMSE after the loop
    const std::vector<TrajectoryEntry>& entries() const;

private:
    std::ofstream file_;
    std::vector<TrajectoryEntry> log_;
};

} // namespace io
} // namespace eskf
