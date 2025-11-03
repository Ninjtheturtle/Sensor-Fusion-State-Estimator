cmake_minimum_required(VERSION 3.21)
project(SensorFusionESKF VERSION 1.0.0 LANGUAGES CXX)

# ── C++17 ──────────────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Windows / MinGW-w64 flags ──────────────────────────────────────────────────
if(WIN32)
    add_compile_definitions(NOMINMAX)           # prevent min/max macros from windows.h
    add_compile_definitions(WIN32_LEAN_AND_MEAN)
    add_compile_definitions(_USE_MATH_DEFINES)  # M_PI on MSVC (MinGW defines it anyway)
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find dependencies (provided by MSYS2 UCRT64 via pacman) ───────────────────
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(GTest REQUIRED)

# ── Static library: all filter + IO + util code (no main) ─────────────────────
add_library(eskf_lib STATIC
    src/utils/QuaternionUtils.cpp
    src/utils/MathUtils.cpp
    src/utils/MahalanobisGate.cpp
    src/utils/NoiseSim.cpp
    src/io/CsvLoader.cpp
    src/io/ResultWriter.cpp
    src/eskf/ImuPredictor.cpp
    src/eskf/GpsUpdater.cpp
    src/eskf/OdomUpdater.cpp
    src/eskf/ESKF.cpp
)

target_include_directories(eskf_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(eskf_lib PUBLIC
    Eigen3::Eigen
)

# ── Main executable ────────────────────────────────────────────────────────────
add_executable(sensor_fusion src/main.cpp)
target_link_libraries(sensor_fusion PRIVATE eskf_lib)

# ── Tests ──────────────────────────────────────────────────────────────────────
enable_testing()
add_subdirectory(tests)
