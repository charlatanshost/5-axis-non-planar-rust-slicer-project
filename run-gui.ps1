<#
.SYNOPSIS
    Build and launch the MultiAxis Slicer GUI.

.DESCRIPTION
    Wraps `cargo run --bin gui` so the crate's location inside files/multiaxis_slicer
    does not have to be remembered. Cargo handles incremental rebuilds, so re-running
    this after a source change only recompiles what changed.

.PARAMETER Fast
    Build the debug profile instead of release. Compiles far quicker -- the release
    profile uses lto = true and codegen-units = 1, so a cold release build takes
    several minutes -- but the GUI itself runs noticeably slower. Use this while
    iterating on code, not for actual slicing work.

.PARAMETER Rebuild
    Discard this crate's build artifacts first and compile from scratch. Worth
    reaching for if cargo appears to be running stale code: its fingerprinting has
    been observed to miss edits in this tree.

.EXAMPLE
    .\run-gui.ps1
    Release build, then launch.

.EXAMPLE
    .\run-gui.ps1 -Fast
    Quick debug build, then launch.
#>
param(
    [switch]$Fast,
    [switch]$Rebuild
)

$ErrorActionPreference = 'Stop'

$CrateDir = Join-Path $PSScriptRoot 'files/multiaxis_slicer'
$Manifest = Join-Path $CrateDir 'Cargo.toml'

if (-not (Test-Path -LiteralPath $Manifest)) {
    Write-Host "Could not find Cargo.toml at:" -ForegroundColor Red
    Write-Host "  $Manifest"
    Write-Host "Run this script from the repository root."
    exit 1
}

if (-not (Get-Command cargo -ErrorAction SilentlyContinue)) {
    Write-Host "cargo was not found on PATH." -ForegroundColor Red
    Write-Host "Install Rust from https://rustup.rs, then open a new terminal."
    exit 1
}

# --release is the default; -Fast opts down to the debug profile.
$cargoArgs = @('run', '--bin', 'gui')
if (-not $Fast) { $cargoArgs += '--release' }

Push-Location $CrateDir
try {
    if ($Rebuild) {
        Write-Host "Clearing build artifacts for multiaxis_slicer..." -ForegroundColor Cyan
        cargo clean -p multiaxis_slicer
    }

    $profileName = if ($Fast) { 'debug' } else { 'release' }
    Write-Host "Building and launching the GUI ($profileName)..." -ForegroundColor Cyan
    Write-Host "The first build after a clean checkout takes a while. Later runs are incremental."
    Write-Host ""

    cargo @cargoArgs
    $code = $LASTEXITCODE

    if ($code -ne 0) {
        Write-Host ""
        Write-Host "cargo exited with code $code." -ForegroundColor Red
        if (-not $Fast) {
            Write-Host "If this is a compile error, '.\run-gui.ps1 -Fast' iterates quicker."
        }
    }
    exit $code
}
finally {
    Pop-Location
}
