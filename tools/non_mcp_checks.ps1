param(
    [string]$CppFile = "src/main.cpp",
    [string]$BuildDir = "build",
    [switch]$Configure,
    [switch]$Build
)

$ErrorActionPreference = "Stop"

function Test-Cmd([string]$Name) {
    return $null -ne (Get-Command $Name -ErrorAction SilentlyContinue)
}

Write-Host "== Non-MCP Checks =="

if (-not (Test-Path $BuildDir)) {
    New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
}

Write-Host ""
Write-Host "[1/3] CMake configure (optional)"
if ($Configure) {
    cmake -S . -B $BuildDir | Out-Host
} else {
    Write-Host "Skip configure (use -Configure to enable)."
}

Write-Host ""
Write-Host "[2/3] C++ diagnostics (clangd --check)"
if (Test-Cmd "clangd") {
    if (-not (Test-Path $CppFile)) {
        Write-Host "Skip clangd: file not found -> $CppFile"
    } else {
        $clangdBuildDir = $BuildDir
        $compileDb = Join-Path $clangdBuildDir "compile_commands.json"

        # Visual Studio generators often omit compile_commands.json.
        # If Ninja is available, generate a dedicated clangd build dir.
        if (-not (Test-Path $compileDb)) {
            if (Test-Cmd "cmake" -and Test-Cmd "ninja") {
                $clangdBuildDir = Join-Path $BuildDir "clangd"
                if (-not (Test-Path $clangdBuildDir)) {
                    New-Item -ItemType Directory -Force -Path $clangdBuildDir | Out-Null
                }
                cmake -S . -B $clangdBuildDir -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON | Out-Host
                $compileDb = Join-Path $clangdBuildDir "compile_commands.json"
            }
        }

        if (Test-Path $compileDb) {
            clangd --compile-commands-dir=$clangdBuildDir --check=$CppFile | Out-Host
        } else {
            Write-Host "Skip clangd: compile_commands.json not found (checked '$BuildDir' and '$BuildDir/clangd')."
        }
    }
} else {
    Write-Host "Skip clangd: clangd not found in PATH."
}

Write-Host ""
Write-Host "[3/3] Build quick check"
if ($Build) {
    cmake --build $BuildDir --config Debug | Out-Host
} else {
    Write-Host "Skip build (use -Build to enable)."
}

Write-Host ""
Write-Host "Done."
