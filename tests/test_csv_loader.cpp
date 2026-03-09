#include <gtest/gtest.h>
#include "io/CsvLoader.hpp"
#include <fstream>
#include <string>
#include <cstdio>

using namespace eskf::io;

// helper: write a temp CSV file

static std::string writeTempFile(const std::string& content) {
    const std::string path = "test_temp_csv.csv";
    std::ofstream f(path, std::ios::trunc);
    f << content;
    f.close();
    return path;
}

// IMU loader tests

TEST(CsvLoader, LoadImuSkipsHeaderLines) {
    const std::string csv =
        "#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z\n"
        "1000000000,0.1,0.2,0.3,0.0,0.0,9.81\n"
        "1005000000,0.0,0.0,0.0,0.0,0.0,9.81\n";
    const std::string path = writeTempFile(csv);

    auto data = CsvLoader::loadIMU(path);
    EXPECT_EQ(data.size(), 2u);

    std::remove(path.c_str());
}

TEST(CsvLoader, LoadImuTimestampConversion) {
    const std::string csv =
        "#header\n"
        "1500000000000,0.0,0.0,0.0,0.0,0.0,9.81\n";
    const std::string path = writeTempFile(csv);

    auto data = CsvLoader::loadIMU(path);
    ASSERT_EQ(data.size(), 1u);
    EXPECT_NEAR(data[0].timestamp, 1500.0, 1e-6);

    std::remove(path.c_str());
}

TEST(CsvLoader, LoadImuColumnOrder) {
    // EuRoC column order: gyro first, then accel
    const std::string csv =
        "#ts,wx,wy,wz,ax,ay,az\n"
        "1000000000,1.0,2.0,3.0,4.0,5.0,6.0\n";
    const std::string path = writeTempFile(csv);

    auto data = CsvLoader::loadIMU(path);
    ASSERT_EQ(data.size(), 1u);
    EXPECT_NEAR(data[0].gyro.x(),  1.0, 1e-9);
    EXPECT_NEAR(data[0].gyro.y(),  2.0, 1e-9);
    EXPECT_NEAR(data[0].gyro.z(),  3.0, 1e-9);
    EXPECT_NEAR(data[0].accel.x(), 4.0, 1e-9);
    EXPECT_NEAR(data[0].accel.y(), 5.0, 1e-9);
    EXPECT_NEAR(data[0].accel.z(), 6.0, 1e-9);

    std::remove(path.c_str());
}

TEST(CsvLoader, LoadImuHandlesCRLF) {
    // Windows CRLF should parse cleanly
    const std::string csv =
        "#header\r\n"
        "1000000000,0.0,0.0,0.0,0.0,0.0,9.81\r\n";
    const std::string path = writeTempFile(csv);

    auto data = CsvLoader::loadIMU(path);
    EXPECT_EQ(data.size(), 1u);
    EXPECT_TRUE(std::isfinite(data[0].accel.z()));

    std::remove(path.c_str());
}

TEST(CsvLoader, LoadImuEmptyFileReturnsEmpty) {
    const std::string path = writeTempFile("");
    auto data = CsvLoader::loadIMU(path);
    EXPECT_TRUE(data.empty());
    std::remove(path.c_str());
}

TEST(CsvLoader, LoadImuMissingFileThrows) {
    EXPECT_THROW(CsvLoader::loadIMU("/nonexistent/path/imu.csv"),
                 std::runtime_error);
}

// ground truth loader tests

TEST(CsvLoader, LoadGroundTruthColumnOrder) {
    const std::string csv =
        "#ts,px,py,pz,qw,qx,qy,qz,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz\n"
        "1000000000,1.0,2.0,3.0,1.0,0.0,0.0,0.0,4.0,5.0,6.0,0.1,0.2,0.3,0.4,0.5,0.6\n";
    const std::string path = writeTempFile(csv);

    auto data = CsvLoader::loadGroundTruth(path);
    ASSERT_EQ(data.size(), 1u);

    const auto& e = data[0];
    EXPECT_NEAR(e.timestamp, 1.0,  1e-6);
    EXPECT_NEAR(e.p.x(),     1.0,  1e-9);
    EXPECT_NEAR(e.p.y(),     2.0,  1e-9);
    EXPECT_NEAR(e.p.z(),     3.0,  1e-9);
    EXPECT_NEAR(e.q.w(),     1.0,  1e-6);
    EXPECT_NEAR(e.v.x(),     4.0,  1e-9);
    EXPECT_NEAR(e.b_gyro.x(),  0.1, 1e-9);
    EXPECT_NEAR(e.b_accel.x(), 0.4, 1e-9);

    std::remove(path.c_str());
}

TEST(CsvLoader, LoadGroundTruthQuaternionNormalized) {
    // non-unit quat in file should be normalized on load
    const std::string csv =
        "#ts,px,py,pz,qw,qx,qy,qz,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz\n"
        "1000000000,0,0,0,2.0,0.0,0.0,0.0,0,0,0,0,0,0,0,0,0\n";
    const std::string path = writeTempFile(csv);

    auto data = CsvLoader::loadGroundTruth(path);
    ASSERT_EQ(data.size(), 1u);
    EXPECT_NEAR(data[0].q.norm(), 1.0, 1e-9);

    std::remove(path.c_str());
}
