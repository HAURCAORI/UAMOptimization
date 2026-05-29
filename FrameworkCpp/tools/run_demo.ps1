# run_demo.ps1 — one-click demo that runs all four modes and opens the output folder.
#
# Usage (from any directory):
#   powershell -ExecutionPolicy Bypass -File tools\run_demo.ps1
#
# What it does:
#   1. kills any leftover FrameworkCpp.exe so a re-build won't hit a file lock
#   2. runs eval / mission / calibrate / soo+mission against the example data
#   3. opens output_demo\ in Explorer so you can browse the SVG / CSV / JSON files
#
# What you get in output_demo\:
#   1_baseline\acs\acs_fig*.svg     ACS plots for the baseline (open any .svg in a browser)
#   1_baseline\acs\acs_vertices.csv 4-D vertex cloud of the attainable control set
#   2_mission\mission_result.json   mission energy totals
#   2_mission\mission_segments.csv  per-segment power / energy / distance (open in Excel)
#   2_mission\acs\*.svg             ACS plots under the city-taxi mission
#   3_calibration\calibration_result.json   fitted parameters + residual stats
#   3_calibration\calibration_residuals.csv per-point measured vs. predicted power
#   4_soo_mission\soo_run.json      SOO best design, all metrics, decision vector
#   4_soo_mission\soo_parameters.csv baseline vs. best parameters (physical units)
#   4_soo_mission\comparison.csv     baseline vs. SOO-best metric comparison

$ErrorActionPreference = 'Stop'
$script_dir  = Split-Path -Parent $MyInvocation.MyCommand.Path
$root        = Split-Path -Parent $script_dir
$exe         = Join-Path $root 'x64\Release\FrameworkCpp.exe'
$out_root    = Join-Path $root 'output_demo'
$mission     = Join-Path $root 'data\example_mission_profile.json'
$flight_csv  = Join-Path $root 'data\example_flight_data.csv'

if (-not (Test-Path $exe)) {
    Write-Host "FrameworkCpp.exe missing at: $exe" -ForegroundColor Red
    Write-Host "Build the solution first (Visual Studio: Release|x64 → Build Solution)." -ForegroundColor Red
    exit 1
}

Write-Host "Killing any leftover FrameworkCpp processes..." -ForegroundColor Cyan
Stop-Process -Name FrameworkCpp -Force -ErrorAction SilentlyContinue
Start-Sleep -Milliseconds 500

if (Test-Path $out_root) {
    Write-Host "Wiping previous output_demo\ ..." -ForegroundColor Cyan
    Remove-Item -Recurse -Force $out_root
}
New-Item -ItemType Directory -Force $out_root | Out-Null

function Run-Step($label, $args_list) {
    Write-Host ""
    Write-Host "===== $label =====" -ForegroundColor Yellow
    & $exe @args_list
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Step '$label' exited with code $LASTEXITCODE" -ForegroundColor Red
    }
}

Run-Step "1/4  baseline eval (legacy hover budget)" @(
    'eval', '--output-dir', "$out_root\1_baseline", '--plot-acs')

Run-Step "2/4  mission evaluation (city taxi profile)" @(
    'mission', $mission, '--output-dir', "$out_root\2_mission", '--plot-acs')

Run-Step "3/4  flight-data calibration" @(
    'calibrate', $flight_csv, '--output-dir', "$out_root\3_calibration")

Run-Step "4/4  SOO optimization with mission profile (pop=16, gen=12 demo budget)" @(
    'soo', '--mission', $mission, '--soo-pop', '16', '--soo-gen', '12',
    '--output-dir', "$out_root\4_soo_mission", '--plot-acs')

Write-Host ""
Write-Host "All artifacts written to: $out_root" -ForegroundColor Green
Write-Host "Opening output_demo in Explorer..." -ForegroundColor Green
Invoke-Item $out_root
