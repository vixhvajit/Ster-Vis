# Activate the conda env that holds Gazebo Harmonic (RoboStack), for Windows.
# Usage:  . .\activate-ros2gz.ps1
# Set STER_VIS_CONDA to your conda root and STER_VIS_GZ_ENV to the env name if
# they differ from the defaults below.

$condaRoot = if ($env:STER_VIS_CONDA) { $env:STER_VIS_CONDA } else { 'C:\ProgramData\miniforge3' }
$envName = if ($env:STER_VIS_GZ_ENV) { $env:STER_VIS_GZ_ENV } else { 'ros2gz' }
$condaHook = Join-Path $condaRoot 'shell\condabin\conda-hook.ps1'
if (-not (Test-Path $condaHook)) { throw "conda hook not found at $condaHook - set STER_VIS_CONDA" }
. $condaHook
# RoboStack's ros-workspace hook calls a local_setup.ps1 that it does not
# ship; the error is harmless, so do not let it abort activation.
$prev = $ErrorActionPreference; $ErrorActionPreference = 'Continue'
conda activate $envName 2>$null
$ErrorActionPreference = $prev

# PowerShell activation skips the env's .bat hooks, and one of them tells
# gz-rendering where OGRE's shaders live. Without it cameras render nothing.
$env:OGRE_RESOURCE_PATH = "$env:CONDA_PREFIX\Library\bin"
$env:OGRE2_RESOURCE_PATH = "$env:CONDA_PREFIX\Library\bin\OGRE-Next"
$env:SDF_PATH = "$env:CONDA_PREFIX\Library\share\sdformat13\1.10"

$env:PYTHONUNBUFFERED = '1'
$env:GZ_IP = '127.0.0.1'
$env:GZ_VERSION = 'harmonic'
$env:GZ_SIM_RESOURCE_PATH = (Join-Path $PSScriptRoot 'worlds') + [IO.Path]::PathSeparator + $env:GZ_SIM_RESOURCE_PATH
