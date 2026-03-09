#pragma once

#include "eskf/State.hpp"
#include "eskf/ImuPredictor.hpp"
#include "eskf/GpsUpdater.hpp"
#include "eskf/OdomUpdater.hpp"
#include "io/CsvLoader.hpp"
#include "utils/NoiseSim.hpp"

namespace eskf {

// top-level ESKF orchestrator — owns nominal state, error state, and covariance
// call initialize() once, then predictIMU() + updateGPS()/updateOdometry() in a loop
class ESKF {
public:
    explicit ESKF(const ESKFConfig& config = ESKFConfig{});

    // call once with the first GT entry
    void initialize(const NominalState& x0, const CovMatrix& P0,
                    double timestamp = 0.0);

    void predictIMU(const io::ImuMeasurement& imu, double dt);

    // returns true if accepted (passed Mahalanobis gate)
    bool updateGPS(const utils::GpsMeasurement& gps);

    bool updateOdometry(const utils::OdomMeasurement& odom);

    const NominalState& nominalState() const { return x_; }
    const CovMatrix&    covariance()   const { return P_; }
    double              timestamp()    const { return timestamp_; }

    int gpsAccepted()  const { return gps_accepted_;  }
    int gpsRejected()  const { return gps_rejected_;  }
    int odomAccepted() const { return odom_accepted_; }
    int odomRejected() const { return odom_rejected_; }

private:
    ESKFConfig    config_;
    NominalState  x_;
    ErrorState    dx_;
    CovMatrix     P_;
    double        timestamp_ = 0.0;
    bool          initialized_ = false;

    ImuPredictor  imuPredictor_;
    GpsUpdater    gpsUpdater_;
    OdomUpdater   odomUpdater_;

    int gps_accepted_  = 0;
    int gps_rejected_  = 0;
    int odom_accepted_ = 0;
    int odom_rejected_ = 0;

    // fold dx_ into x_, then zero dx_
    void injectAndReset();
};

} // namespace eskf
