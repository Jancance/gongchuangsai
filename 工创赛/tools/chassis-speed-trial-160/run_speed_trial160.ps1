param([string]$Gcc)

$ErrorActionPreference = 'Stop'
if (!$Gcc) {
    $compiler = Get-Command gcc -CommandType Application -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if ($compiler) {
        $Gcc = $compiler.Source
    } else {
        $Gcc = 'D:/dev/Dev-Cpp/MinGW64/bin/gcc.exe'
    }
}

$projectData = (Get-Item -LiteralPath $PSScriptRoot).Parent.Parent.FullName
$projectRoot = Split-Path -Parent $projectData
$firmware = Join-Path $projectRoot '3.stm32f407工程'
$chassis = Join-Path $firmware 'CHASSIS/USER'
$output = Join-Path $projectData 'tmp/chassis-speed-trial-160'
$null = New-Item -ItemType Directory -Path $output -Force
$exe = Join-Path $output 'speed_trial160.exe'
$csv = Join-Path $output 'speed_trial160.csv'

# The host executable uses the geometry and encoding only, without mission modes.
& $Gcc -std=c99 -Wall -Wextra -DAPP_MOTOR_OFFLINE_ONLY=1 `
    -DAPP_ZONE2_UNVERIFIED_TRIAL_ENABLE=0 -DAPP_SPEED_TRIAL160=0 `
    "-I$chassis" (Join-Path $PSScriptRoot 'speed_trial160.c') `
    (Join-Path $chassis 'chassis_kinematics.c') -lm -o $exe
if ($LASTEXITCODE -ne 0) { throw "GCC failed with exit code $LASTEXITCODE." }

$rows = & $exe
if ($LASTEXITCODE -ne 0) { throw "Trajectory generator failed with exit code $LASTEXITCODE." }
$rows | Set-Content -LiteralPath $csv -Encoding Ascii
Write-Output $csv
