# Start the headless Gazebo server on one of the sim worlds (Windows).
# Windows only runs the server (no gz GUI, gazebosim/gz-sim#168); watch the run
# through avoid.py's chase camera, live (--show) or in output\run.mp4.
#
# Usage:  .\run-gz.ps1            # runs until Ctrl+C; start-gz.ps1 runs it hidden
param([string]$World = 'stereo_avoid.sdf')
$ErrorActionPreference = 'Stop'
. $PSScriptRoot\activate-ros2gz.ps1

# The world and its textures are generated, not committed.
if (-not (Test-Path (Join-Path $PSScriptRoot 'worlds\textures\ground.png'))) {
    python (Join-Path $PSScriptRoot 'make_textures.py')
}
if (-not (Test-Path (Join-Path $PSScriptRoot "worlds\$World"))) {
    $maker = if ($World -like 'warehouse*') { 'make_warehouse.py' } else { 'make_world.py' }
    python (Join-Path $PSScriptRoot $maker)
}

$bin = "$env:CONDA_PREFIX\Library\bin"
$worldPath = Join-Path $PSScriptRoot "worlds\$World"
# gz only loads gz-sim8-gz.dll with cwd = Library\bin; world must be absolute.
Push-Location $bin
try {
    & (Join-Path $bin 'gz.bat') sim -s -r -v 3 $worldPath
} finally {
    Pop-Location
}
