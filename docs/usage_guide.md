# Usage Guide

## Prerequisites

### 1. Install MSYS2

Download and install MSYS2 from https://www.msys2.org

- Install to the default path: `C:\msys64`
- After installation, open **MSYS2 UCRT64** from the Start Menu
- Run the initial system update:
  ```bash
  pacman -Syu
  ```
  Close and reopen the terminal if prompted, then run:
  ```bash
  pacman -Su
  ```

### 2. Install C++ Build Tools

In the MSYS2 UCRT64 terminal:

```bash
pacman -S mingw-w64-ucrt-x86_64-gcc
pacman -S mingw-w64-ucrt-x86_64-cmake
pacman -S mingw-w64-ucrt-x86_64-ninja
pacman -S mingw-w64-ucrt-x86_64-eigen3
pacman -S mingw-w64-ucrt-x86_64-gtest
```

### 3. Add to Windows PATH

Open **System Properties → Environment Variables → System Variables → PATH** and add:
```
C:\msys64\ucrt64\bin
```

Or run as Administrator in PowerShell:
```powershell
[System.Environment]::SetEnvironmentVariable("PATH",
    "C:\msys64\ucrt64\bin;" + $env:PATH, "Machine")
```

### 4. Install VS Code Extensions

In VS Code, install:
- **C/C++ Extension Pack** (`ms-vscode.cpptools-extension-pack`)
- **CMake Tools** (`ms-vscode.cmake-tools`)

### 5. Install Python Packages

```bash
pip install numpy matplotlib scipy pandas
```

---

## Building the Project

### From VS Code

1. Open the project folder in VS Code
2. When prompted, select the CMake kit: **MSYS2 UCRT64 Debug**
   - If not prompted, press `Ctrl+Shift+P` → "CMake: Select a Kit"
3. Press **Ctrl+Shift+B** to build

### From Terminal

```bash
# Configure
cmake --preset ucrt64-debug

# Build
cmake --build build/debug

# Build Release (for performance)
cmake --preset ucrt64-release
cmake --build build/release
```

---

## Running Tests

```bash
ctest --preset test-debug --verbose
```

Or from VS Code: `Ctrl+Shift+P` → "CMake: Run Tests"

Expected output: all tests GREEN.

---

## Downloading the EuRoC Dataset

1. Open in browser: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
2. Scroll to **Machine Hall** sequences
3. Download **MH_01_easy** — click the "ASL format" download link (~2.9 GB ZIP)
4. Extract the ZIP. Inside you will find `mav0/`
5. Copy the two required CSV files:
   ```
   mav0/imu0/data.csv
     → data/MH_01_easy/imu0/data.csv

   mav0/state_groundtruth_estimate0/data.csv
     → data/MH_01_easy/state_groundtruth_estimate0/data.csv
   ```

The camera images (`mav0/cam0/`, `mav0/cam1/`) are not needed for this project.

---

## Running the Filter

```bash
./build/debug/sensor_fusion.exe \
  --imu  data/MH_01_easy/imu0/data.csv \
  --gt   data/MH_01_easy/state_groundtruth_estimate0/data.csv \
  --out  output/trajectory.csv
```

Expected runtime: ~5-10 seconds for the 80-second MH_01_easy sequence.

Expected output:
```
[INFO] Loading IMU data...  Loaded 16522 IMU samples
[INFO] Loading ground truth... Loaded 3682 GT samples
[INFO] Generating simulated GPS...  Generated 184 GPS samples
[INFO] Generating simulated odometry... Generated 184 odom samples
[INFO] Running ESKF fusion loop...
  t = 10.0 s | GPS accepted/rejected: 100/0 | Odom accepted/rejected: 100/0
  ...

========================================
  Position RMSE (over 16520 samples)
========================================
  X:   X.XXX cm
  Y:   X.XXX cm
  Z:   X.XXX cm
  3D:  X.XXX cm
========================================
  [PASS] RMSE < 5 cm target achieved!
========================================
```

---

## Debugging with VS Code

1. Build in Debug mode (Ctrl+Shift+B)
2. Press **F5** to launch the debugger
3. Set breakpoints anywhere in the source code
4. The debugger uses GDB from MSYS2 (configured in `.vscode/launch.json`)

---

## Visualizing Results

After running the filter:

```bash
# 3D trajectory plot
python scripts/visualize_trajectory.py

# Position error plot + RMSE table
python scripts/plot_errors.py
```

Plots are displayed interactively. `plot_errors.py` also saves `output/position_errors.png`.

---

## Tuning Parameters

Default parameters are in `include/eskf/State.hpp` (`ESKFConfig` struct) and documented in `data/config/ekf_params.yaml`.

### If RMSE > 5 cm:

1. **Increase GPS noise trust** (decrease `sigma_gps_xy`, `sigma_gps_z`) — more aggressive corrections
2. **Increase IMU noise parameters** (`sigma_acc_n`, `sigma_gyro_n`) — trust IMU less, trust GPS more
3. **Adjust outlier threshold** — decrease `chi2_gps_threshold` to accept more updates
4. **Check initialization** — ensure bias values from first GT entry are accurate

### Testing other EuRoC sequences:

Replace the `--imu` and `--gt` paths with any other sequence:
- `MH_02_easy` — similar difficulty
- `MH_03_medium` — faster motion
- `V1_01_easy` — Vicon room, tighter space

---

## Project Build Commands Summary

| Task | Command |
|------|---------|
| Configure (debug) | `cmake --preset ucrt64-debug` |
| Build (debug) | `cmake --build build/debug` |
| Build (release) | `cmake --preset ucrt64-release && cmake --build build/release` |
| Run tests | `ctest --preset test-debug --verbose` |
| Run filter | `./build/debug/sensor_fusion.exe --imu ... --gt ... --out ...` |
| Visualize | `python scripts/visualize_trajectory.py` |
| RMSE plot | `python scripts/plot_errors.py` |
