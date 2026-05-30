# Compiles all FrameworkCpp visualization shaders to SPIR-V using glslc.
# Run from the FrameworkCpp/visualization/shaders directory, or adjust $ShaderDir below.
# Requires the Vulkan SDK to be installed (glslc in PATH or set $GlslcPath).

param(
    [string]$GlslcPath = "glslc"   # set to full path if glslc is not in PATH
)

$ShaderDir  = $PSScriptRoot
$OutputDir  = Join-Path $ShaderDir "compiled"

if (-not (Test-Path $OutputDir)) {
    New-Item -ItemType Directory -Path $OutputDir | Out-Null
}

$shaders = @(
    @{ src = "primitive.vert"; out = "primitive.vert.spv" },
    @{ src = "primitive.frag"; out = "primitive.frag.spv" },
    @{ src = "skybox.vert";    out = "skybox.vert.spv"    },
    @{ src = "skybox.frag";    out = "skybox.frag.spv"    },
    @{ src = "shadow.vert";    out = "shadow.vert.spv"    },
    @{ src = "shadow.frag";    out = "shadow.frag.spv"    }
)

$errors = 0
foreach ($s in $shaders) {
    $src = Join-Path $ShaderDir $s.src
    $dst = Join-Path $OutputDir $s.out
    Write-Host "Compiling $($s.src) ..."
    & $GlslcPath $src -o $dst
    if ($LASTEXITCODE -ne 0) {
        Write-Error "FAILED: $($s.src)"
        $errors++
    } else {
        Write-Host "  -> $($s.out)"
    }
}

if ($errors -eq 0) {
    Write-Host "`nAll shaders compiled successfully."
} else {
    Write-Error "`n$errors shader(s) failed to compile."
    exit 1
}
