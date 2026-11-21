<#
Wait for the first recorded Bare wide16 v3 scene, then render its 16 robots
from saved MuJoCo joint states. The collection continues through later scenes.
#>
param(
    [string]$ResultsName = 'grid_16_wide_v3',
    [double]$CameraDistance = 1.7
)

$ErrorActionPreference = 'Stop'
if ($ResultsName -notmatch '^[A-Za-z0-9_-]+$') { throw 'Invalid ResultsName.' }
if (-not [double]::IsFinite($CameraDistance) -or $CameraDistance -le 0) {
    throw 'CameraDistance must be positive and finite.'
}
$repository = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$dataset = Join-Path $repository (Join-Path 'data' $ResultsName)
$container = "a0509-grid-$ResultsName"
$video = Join-Path $repository (Join-Path 'results' 'wide16_v3_bare_recorded_mujoco.mp4')
$python = Join-Path $repository '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $python)) { throw "Missing Python environment: $python" }

while ($true) {
    $ready = $true
    foreach ($number in 1..16) {
        $trial = Join-Path $dataset "simulation\T$number\bare_p0\trial_001"
        $metadataPath = Join-Path $trial 'metadata.json'
        $csvPath = Join-Path $trial 'dynamics.csv'
        if (-not (Test-Path -LiteralPath $metadataPath) -or -not (Test-Path -LiteralPath $csvPath)) {
            $ready = $false
            break
        }
        try {
            $metadata = Get-Content -LiteralPath $metadataPath -Raw | ConvertFrom-Json
            if ($metadata.condition_id -ne 'bare_p0' -or $metadata.trajectory_id -ne "T$number" -or
                $metadata.trajectory_config.profile -ne 'wide16_v3' -or
                $metadata.quality.rows_written -ne 45201 -or
                $metadata.report.max_tracking_error_rad -gt 0.05 -or
                $metadata.report.torque_limit_samples -ne 0 -or
                $metadata.report.cross_robot_contacts -ne 0) {
                $ready = $false
                break
            }
        } catch {
            $ready = $false
            break
        }
    }
    if ($ready) { break }
    $state = & docker inspect $container --format '{{.State.Status}}' 2>$null
    if ($LASTEXITCODE -ne 0 -or $state -ne 'running') {
        throw "Collection stopped before the first Bare scene completed. Check: docker logs $container"
    }
    Write-Host 'Waiting for recorded Bare T1-T16 scene...'
    Start-Sleep -Seconds 30
}

& $python -m scripts.render_bare_16_mujoco_replay --campaign $dataset --output $video --camera-distance $CameraDistance
if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath $video)) {
    throw 'Recorded MuJoCo video render failed.'
}
Write-Host "Recorded video: $video"
