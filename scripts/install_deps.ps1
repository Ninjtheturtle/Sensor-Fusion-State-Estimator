# install_deps.ps1
# Installs all dependencies required for the Sensor Fusion ESKF project.
# Run this script from a PowerShell terminal (not the MSYS2 terminal).
# Requires MSYS2 to already be installed at C:\msys64.
#
# Usage:
#   Right-click -> "Run with PowerShell", OR
#   In PowerShell: .\scripts\install_deps.ps1

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

Write-Host "=== Sensor Fusion ESKF - Dependency Installer ===" -ForegroundColor Cyan
Write-Host ""

# ── Step 1: Check MSYS2 ────────────────────────────────────────────────────────
$msys2Path = "C:\msys64"
if (-not (Test-Path "$msys2Path\ucrt64\bin\gcc.exe")) {
    Write-Host "[ERROR] MSYS2 not found at $msys2Path or GCC not installed." -ForegroundColor Red
    Write-Host ""
    Write-Host "Please install MSYS2 first:" -ForegroundColor Yellow
    Write-Host "  1. Download from: https://www.msys2.org"
    Write-Host "  2. Install to C:\msys64 (default)"
    Write-Host "  3. Open 'MSYS2 UCRT64' from Start Menu"
    Write-Host "  4. Run: pacman -Syu"
    Write-Host "  5. Close and reopen MSYS2 UCRT64, then run: pacman -Su"
    Write-Host "  6. Then re-run this script"
    exit 1
}
Write-Host "[OK] MSYS2 found at $msys2Path" -ForegroundColor Green

# ── Step 2: Install C++ packages via pacman ────────────────────────────────────
Write-Host ""
Write-Host "Installing MSYS2 packages (Eigen3, GTest, CMake, Ninja, GCC)..." -ForegroundColor Cyan
$pacman = "$msys2Path\usr\bin\pacman.exe"

$packages = @(
    "mingw-w64-ucrt-x86_64-gcc",
    "mingw-w64-ucrt-x86_64-cmake",
    "mingw-w64-ucrt-x86_64-ninja",
    "mingw-w64-ucrt-x86_64-eigen3",
    "mingw-w64-ucrt-x86_64-gtest"
)

foreach ($pkg in $packages) {
    Write-Host "  Installing $pkg..." -ForegroundColor Gray
    & $pacman -S --noconfirm $pkg 2>&1 | Out-Null
}
Write-Host "[OK] C++ packages installed" -ForegroundColor Green

# ── Step 3: Add MSYS2 UCRT64 to System PATH ───────────────────────────────────
Write-Host ""
Write-Host "Checking Windows PATH for MSYS2 UCRT64..." -ForegroundColor Cyan
$ucrt64Bin = "C:\msys64\ucrt64\bin"
$currentPath = [System.Environment]::GetEnvironmentVariable("PATH", "Machine")

if ($currentPath -notlike "*$ucrt64Bin*") {
    Write-Host "  Adding $ucrt64Bin to system PATH..." -ForegroundColor Gray
    try {
        [System.Environment]::SetEnvironmentVariable(
            "PATH",
            "$ucrt64Bin;$currentPath",
            "Machine"
        )
        Write-Host "[OK] PATH updated (restart VS Code for changes to take effect)" -ForegroundColor Green
    } catch {
        Write-Host "[WARN] Could not update system PATH (run as Administrator)." -ForegroundColor Yellow
        Write-Host "       Manually add to PATH: $ucrt64Bin" -ForegroundColor Yellow
    }
} else {
    Write-Host "[OK] $ucrt64Bin already in PATH" -ForegroundColor Green
}

# ── Step 4: Install Python packages ───────────────────────────────────────────
Write-Host ""
Write-Host "Installing Python packages (numpy, matplotlib, scipy, pandas)..." -ForegroundColor Cyan
$pythonExe = Get-Command python -ErrorAction SilentlyContinue
if ($null -eq $pythonExe) {
    Write-Host "[WARN] Python not found. Please install Python 3.x from https://python.org" -ForegroundColor Yellow
    Write-Host "       Then run: pip install numpy matplotlib scipy pandas" -ForegroundColor Yellow
} else {
    & python -m pip install --upgrade pip numpy matplotlib scipy pandas 2>&1 | Out-Null
    Write-Host "[OK] Python packages installed" -ForegroundColor Green
}

# ── Step 5: Verify installations ──────────────────────────────────────────────
Write-Host ""
Write-Host "=== Verification ===" -ForegroundColor Cyan

$checks = @(
    @{ Name = "GCC";    Path = "C:\msys64\ucrt64\bin\g++.exe" },
    @{ Name = "CMake";  Path = "C:\msys64\ucrt64\bin\cmake.exe" },
    @{ Name = "Ninja";  Path = "C:\msys64\ucrt64\bin\ninja.exe" },
    @{ Name = "GDB";    Path = "C:\msys64\ucrt64\bin\gdb.exe" },
    @{ Name = "Eigen3"; Path = "C:\msys64\ucrt64\include\eigen3\Eigen\Core" },
    @{ Name = "GTest";  Path = "C:\msys64\ucrt64\include\gtest\gtest.h" }
)

$allOk = $true
foreach ($check in $checks) {
    if (Test-Path $check.Path) {
        Write-Host "  [OK] $($check.Name)" -ForegroundColor Green
    } else {
        Write-Host "  [MISSING] $($check.Name) at $($check.Path)" -ForegroundColor Red
        $allOk = $false
    }
}

Write-Host ""
if ($allOk) {
    Write-Host "All dependencies installed successfully!" -ForegroundColor Green
    Write-Host ""
    Write-Host "Next steps:" -ForegroundColor Cyan
    Write-Host "  1. Restart VS Code (or open a new terminal)"
    Write-Host "  2. Open this folder in VS Code"
    Write-Host "  3. Install VS Code extensions:"
    Write-Host "       ms-vscode.cpptools-extension-pack"
    Write-Host "       ms-vscode.cmake-tools"
    Write-Host "  4. Press Ctrl+Shift+B to build"
    Write-Host "  5. Download EuRoC MH_01_easy dataset (see docs/usage_guide.md)"
} else {
    Write-Host "Some dependencies are missing. Check the errors above." -ForegroundColor Red
}
