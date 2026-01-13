add_executable(run_tests
    test_quaternion_utils.cpp
    test_imu_predictor.cpp
    test_gps_updater.cpp
    test_mahalanobis.cpp
    test_csv_loader.cpp
    test_integration.cpp
)

target_link_libraries(run_tests PRIVATE
    eskf_lib
    GTest::gtest
    GTest::gtest_main
)

target_include_directories(run_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

include(GoogleTest)
gtest_discover_tests(run_tests
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
