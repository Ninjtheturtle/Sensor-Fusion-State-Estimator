#include "eskf/ESKF.hpp"
#include "eskf/State.hpp"
#include "io/CsvLoader.hpp"
#include "io/ResultWriter.hpp"
#include "utils/NoiseSim.hpp"

#include <iostream>
#include <string>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <iomanip>

struct Args {
    std::string imu_path;
    std::string gt_path;
    std::string out_path = "output/trajectory.csv";
    bool help = false;
};

static void printUsage(const char* argv0) {
    std::cout << "Usage: " << argv0 << "\n"
              << "  --imu  <path>   EuRoC imu0/data.csv\n"
              << "  --gt   <path>   EuRoC state_groundtruth_estimate0/data.csv\n"
              << "  --out  <path>   Output trajectory CSV (default: output/trajectory.csv)\n"
              << "  --help          Show this message\n";
}

static Args parseArgs(int argc, char* argv[]) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        std::string a(argv[i]);
        if (a == "--help" || a == "-h") {
            args.help = true;
        } else if (a == "--imu" && i + 1 < argc) {
            args.imu_path = argv[++i];
        } else if (a == "--gt" && i + 1 < argc) {
            args.gt_path = argv[++i];
        } else if (a == "--out" && i + 1 < argc) {
            args.out_path = argv[++i];
        }
    }
    return args;
}

// linear interp of GT at time t; returns false if out of range
static bool interpolateGT(const std::vector<eskf::io::GroundTruthEntry>& gt,
                           double t,
                           Eigen::Vector3d& p_out)
{
    if (gt.empty()) return false;
    if (t < gt.front().timestamp || t > gt.back().timestamp) return false;

    auto it = std::lower_bound(gt.begin(), gt.end(), t,
        [](const eskf::io::GroundTruthEntry& e, double ts) {
            return e.timestamp < ts;
        });

    if (it == gt.end()) { p_out = gt.back().p; return true; }
    if (it == gt.begin()) { p_out = gt.front().p; return true; }

    const auto& hi = *it;
    const auto& lo = *(it - 1);
    const double alpha = (t - lo.timestamp) / (hi.timestamp - lo.timestamp);
    p_out = lo.p + alpha * (hi.p - lo.p);
    return true;
}

static void computeAndPrintRMSE(
    const std::vector<eskf::io::TrajectoryEntry>& log,
    const std::vector<eskf::io::GroundTruthEntry>& gt)
{
    if (log.empty() || gt.empty()) {
        std::cerr << "[RMSE] No data to evaluate.\n";
        return;
    }

    double sum_sq_x = 0.0, sum_sq_y = 0.0, sum_sq_z = 0.0, sum_sq_3d = 0.0;
    int count = 0;

    for (const auto& entry : log) {
        Eigen::Vector3d gt_p;
        if (!interpolateGT(gt, entry.timestamp, gt_p)) continue;

        const Eigen::Vector3d err = entry.p - gt_p;
        sum_sq_x  += err.x() * err.x();
        sum_sq_y  += err.y() * err.y();
        sum_sq_z  += err.z() * err.z();
        sum_sq_3d += err.squaredNorm();
        ++count;
    }

    if (count == 0) {
        std::cerr << "[RMSE] No overlapping timestamps found.\n";
        return;
    }

    const double rmse_x   = std::sqrt(sum_sq_x  / count);
    const double rmse_y   = std::sqrt(sum_sq_y  / count);
    const double rmse_z   = std::sqrt(sum_sq_z  / count);
    const double rmse_3d  = std::sqrt(sum_sq_3d / count);

    std::cout << "\n========================================\n"
              << "  Position RMSE (over " << count << " samples)\n"
              << "========================================\n"
              << std::fixed << std::setprecision(3)
              << "  X:   " << rmse_x  * 100.0 << " cm\n"
              << "  Y:   " << rmse_y  * 100.0 << " cm\n"
              << "  Z:   " << rmse_z  * 100.0 << " cm\n"
              << "  3D:  " << rmse_3d * 100.0 << " cm\n"
              << "========================================\n";

    if (rmse_3d < 0.05) {
        std::cout << "  [PASS] RMSE < 5 cm target achieved!\n";
    } else {
        std::cout << "  [NOTE] RMSE >= 5 cm. Consider tuning noise parameters.\n";
    }
    std::cout << "========================================\n\n";
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    const Args args = parseArgs(argc, argv);

    if (args.help || args.imu_path.empty() || args.gt_path.empty()) {
        printUsage(argv[0]);
        return args.help ? 0 : 1;
    }

    try {
        std::cout << "[INFO] Loading IMU data from:          " << args.imu_path << "\n";
        const auto imu_data = eskf::io::CsvLoader::loadIMU(args.imu_path);
        std::cout << "[INFO]   Loaded " << imu_data.size() << " IMU samples\n";

        std::cout << "[INFO] Loading ground truth from:      " << args.gt_path << "\n";
        const auto gt_data = eskf::io::CsvLoader::loadGroundTruth(args.gt_path);
        std::cout << "[INFO]   Loaded " << gt_data.size() << " GT samples\n";

        if (imu_data.size() < 2 || gt_data.empty()) {
            throw std::runtime_error("Insufficient data loaded.");
        }

        // simulate GPS and odom from GT
        eskf::ESKFConfig config;  // uses default parameters for EuRoC MH_01_easy
        eskf::utils::NoiseSim noise_sim(42);

        std::cout << "[INFO] Generating simulated GPS (10 Hz, 5cm XY / 10cm Z noise)...\n";
        const auto gps_data = noise_sim.generateGPS(
            gt_data, config.gps_rate_hz, config.sigma_gps_xy, config.sigma_gps_z);
        std::cout << "[INFO]   Generated " << gps_data.size() << " GPS samples\n";

        std::cout << "[INFO] Generating simulated odometry (10 Hz, 5cm/s noise)...\n";
        const auto odom_data = noise_sim.generateOdometry(
            gt_data, config.odom_rate_hz, config.sigma_odom);
        std::cout << "[INFO]   Generated " << odom_data.size() << " odom samples\n";

        eskf::NominalState x0;
        x0.p   = gt_data[0].p;
        x0.v   = gt_data[0].v;
        x0.q   = gt_data[0].q;
        x0.b_g = gt_data[0].b_gyro;
        x0.b_a = gt_data[0].b_accel;

        const eskf::CovMatrix P0 = eskf::initCovariance(config);

        // IMU starts ~1s before GT — if we initialize at imu_data[0] the position drifts ~1m
        // before the first GPS arrives, killing the Mahalanobis gate. skip to first GT timestamp.
        size_t imu_start = 0;
        while (imu_start < imu_data.size() &&
               imu_data[imu_start].timestamp < gt_data[0].timestamp) {
            ++imu_start;
        }
        if (imu_start >= imu_data.size()) {
            throw std::runtime_error(
                "No IMU samples found at or after the first GT timestamp.");
        }
        std::cout << "[INFO] Skipping " << imu_start << " pre-GT IMU samples ("
                  << std::fixed << std::setprecision(3)
                  << (imu_data[imu_start].timestamp - imu_data[0].timestamp)
                  << " s)\n";

        eskf::ESKF filter(config);
        filter.initialize(x0, P0, imu_data[imu_start].timestamp);

        std::cout << "[INFO] Writing output to:              " << args.out_path << "\n\n";
        eskf::io::ResultWriter writer(args.out_path);

        // skip GPS/odom before filter start
        size_t gps_idx  = 0;
        size_t odom_idx = 0;
        while (gps_idx  < gps_data.size()  &&
               gps_data[gps_idx].timestamp  < imu_data[imu_start].timestamp) ++gps_idx;
        while (odom_idx < odom_data.size() &&
               odom_data[odom_idx].timestamp < imu_data[imu_start].timestamp) ++odom_idx;

        size_t log_every = 1;  // log every IMU step (200 Hz)

        std::cout << "[INFO] Running ESKF fusion loop...\n";
        for (size_t i = imu_start + 1; i < imu_data.size(); ++i) {
            const double dt = imu_data[i].timestamp - imu_data[i-1].timestamp;

            // bad dt — skip
            if (dt <= 0.0 || dt > 0.1) continue;

            filter.predictIMU(imu_data[i], dt);

            while (gps_idx < gps_data.size() &&
                   gps_data[gps_idx].timestamp <= imu_data[i].timestamp)
            {
                filter.updateGPS(gps_data[gps_idx]);
                ++gps_idx;
            }

            while (odom_idx < odom_data.size() &&
                   odom_data[odom_idx].timestamp <= imu_data[i].timestamp)
            {
                filter.updateOdometry(odom_data[odom_idx]);
                ++odom_idx;
            }

            if (i % log_every == 0) {
                const auto& x = filter.nominalState();
                const auto& P = filter.covariance();

                eskf::io::TrajectoryEntry entry;
                entry.timestamp   = filter.timestamp();
                entry.p           = x.p;
                entry.q           = x.q;
                entry.v           = x.v;
                entry.P_pos_diag  = P.diagonal().segment<3>(0);
                entry.P_vel_diag  = P.diagonal().segment<3>(3);
                entry.P_att_diag  = P.diagonal().segment<3>(6);
                writer.write(entry);
            }

            if (i % 2000 == 0) {
                const double t_elapsed = imu_data[i].timestamp - imu_data[0].timestamp;
                std::cout << "  t = " << std::fixed << std::setprecision(1)
                          << t_elapsed << " s  |  "
                          << "GPS accepted/rejected: "
                          << filter.gpsAccepted() << "/" << filter.gpsRejected()
                          << "  |  Odom accepted/rejected: "
                          << filter.odomAccepted() << "/" << filter.odomRejected()
                          << "\n";
            }
        }

        std::cout << "\n[INFO] Fusion complete.\n"
                  << "  IMU samples processed:   " << imu_data.size() - 1 << "\n"
                  << "  GPS accepted/rejected:   "
                  << filter.gpsAccepted() << " / " << filter.gpsRejected() << "\n"
                  << "  Odom accepted/rejected:  "
                  << filter.odomAccepted() << " / " << filter.odomRejected() << "\n"
                  << "  Output written to:       " << args.out_path << "\n";

        computeAndPrintRMSE(writer.entries(), gt_data);

        std::cout << "[INFO] To visualize:\n"
                  << "  python scripts/visualize_trajectory.py\n"
                  << "  python scripts/plot_errors.py\n\n";

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << "\n";
        return 1;
    }

    return 0;
}
