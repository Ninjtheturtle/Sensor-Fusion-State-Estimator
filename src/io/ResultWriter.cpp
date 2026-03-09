#include "io/ResultWriter.hpp"
#include <iomanip>
#include <stdexcept>

namespace eskf {
namespace io {

ResultWriter::ResultWriter(const std::string& path) {
    file_.open(path, std::ios::out | std::ios::trunc);
    if (!file_.is_open()) {
        throw std::runtime_error("ResultWriter: cannot open output file: " + path);
    }
    file_ << "timestamp_s,"
          << "est_px,est_py,est_pz,"
          << "est_qw,est_qx,est_qy,est_qz,"
          << "est_vx,est_vy,est_vz,"
          << "P_pos_xx,P_pos_yy,P_pos_zz,"
          << "P_vel_xx,P_vel_yy,P_vel_zz,"
          << "P_att_xx,P_att_yy,P_att_zz"
          << "\n";
}

ResultWriter::~ResultWriter() {
    if (file_.is_open()) {
        file_.close();
    }
}

void ResultWriter::write(const TrajectoryEntry& e) {
    file_ << std::fixed << std::setprecision(9) << e.timestamp << ","
          << std::setprecision(6)
          << e.p.x() << "," << e.p.y() << "," << e.p.z() << ","
          << e.q.w() << "," << e.q.x() << "," << e.q.y() << "," << e.q.z() << ","
          << e.v.x() << "," << e.v.y() << "," << e.v.z() << ","
          << e.P_pos_diag.x() << "," << e.P_pos_diag.y() << "," << e.P_pos_diag.z() << ","
          << e.P_vel_diag.x() << "," << e.P_vel_diag.y() << "," << e.P_vel_diag.z() << ","
          << e.P_att_diag.x() << "," << e.P_att_diag.y() << "," << e.P_att_diag.z()
          << "\n";
    log_.push_back(e);
}

const std::vector<TrajectoryEntry>& ResultWriter::entries() const {
    return log_;
}

} // namespace io
} // namespace eskf
