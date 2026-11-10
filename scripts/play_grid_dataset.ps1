<#
One headless MuJoCo world with up to 16 A0509 lanes. Simulation only.
The 2x2 pilot is required for each campaign before full 4x4 collection.
#>
param(
    [switch]$Pilot,
    [switch]$Play,
    [switch]$Status,
    [ValidateSet('legacy','trajectory16')][string]$Campaign = 'legacy',
    [ValidateSet('all','T1','T2','T3','T4','T5','T6','T7','T8','T9','T10','T11','T12','T13','T14','T15','T16')][string]$Trajectory = 'all',
    [ValidateRange(1,2)][int]$MaxBatches = 2,
    [ValidateRange(1,21)][int]$MaxRuns = 21,
    [string]$ResultsName = 'grid_4x4_7conditions',
    [string]$BaseContainer = 'a0509-dynamics',
    [switch]$UseCadFaceCenterFixture,
    [double[]]$MdpgMountXyz,
    [double[]]$MdpgMountRpy
)

$ErrorActionPreference = 'Stop'
if ($Campaign -eq 'trajectory16' -and -not $PSBoundParameters.ContainsKey('ResultsName')) {
    $ResultsName = 'grid_16_trajectories'
}
if ($Campaign -eq 'trajectory16' -and $Trajectory -ne 'all') {
    throw 'The trajectory16 campaign assigns T1-T16 to all 16 lanes; use -Trajectory all.'
}
if ($ResultsName -notmatch '^[A-Za-z0-9_-]+$') { throw 'ResultsName must use letters, digits, _ or -.' }
if ($BaseContainer -notmatch '^[A-Za-z0-9_.-]+$') { throw 'BaseContainer has unsupported characters.' }
if (([int][bool]$Pilot + [int][bool]$Play + [int][bool]$Status) -gt 1) { throw 'Choose one of -Pilot, -Play, -Status.' }
$repository = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$root = Join-Path $repository (Join-Path 'results' $ResultsName)
$name = "a0509-grid-$ResultsName"
if ($Status) {
    $report = Join-Path $root 'pilot_report.json'
    if (Test-Path -LiteralPath $report) {
        $pilotData = Get-Content -LiteralPath $report -Raw | ConvertFrom-Json
        Write-Host "Pilot pass: $($pilotData.pass); max error: $($pilotData.max_tracking_error_rad) rad; elapsed: $($pilotData.wall_seconds) s"
    } else { Write-Host 'Pilot report: absent' }
    $finished = 0
    $sceneGroups = @{}
    foreach ($file in @(Get-ChildItem -LiteralPath (Join-Path $root 'simulation') -Recurse -Filter metadata.json -File -ErrorAction SilentlyContinue)) {
        try {
            $item = Get-Content -LiteralPath $file.FullName -Raw | ConvertFrom-Json
            $bagMetadata = Join-Path $root (Join-Path $item.rosbag_relative_path 'metadata.yaml')
            if ($item.report.max_tracking_error_rad -le 0.05 -and $item.report.torque_limit_samples -eq 0 -and
                $item.report.cross_robot_contacts -eq 0 -and
                (Test-Path -LiteralPath $bagMetadata) -and
                (Test-Path -LiteralPath (Join-Path $file.DirectoryName 'dynamics.csv'))) {
                $finished++
                $key = "$($item.condition_id)|$($item.trial_id)"
                if (-not $sceneGroups.ContainsKey($key)) { $sceneGroups[$key] = @{} }
                $sceneGroups[$key][$item.trajectory_id] = $true
            }
        } catch { }
    }
    $total = if ($Campaign -eq 'trajectory16') { 336 } else { 294 }
    Write-Host "Full trials complete: $finished/$total"
    if ($Campaign -eq 'trajectory16') {
        $scenesComplete = @($sceneGroups.Values | Where-Object { $_.Count -eq 16 }).Count
        Write-Host "Scene executions complete: $scenesComplete/21"
    }
    & docker ps -a --filter "name=^/$name`$" --format 'table {{.Names}}\t{{.Status}}'
    if ($LASTEXITCODE -ne 0) { throw 'Docker status failed.' }
    return
}
if (-not $Pilot -and -not $Play) {
    Write-Host 'Run -Pilot first, then -Play. Use -Status to inspect results.'
    return
}
if ($UseCadFaceCenterFixture) {
    if ($PSBoundParameters.ContainsKey('MdpgMountXyz') -or $PSBoundParameters.ContainsKey('MdpgMountRpy')) {
        throw 'Choose the CAD fixture or explicit mDPG mount arrays.'
    }
    $MdpgMountXyz = [double[]]@(0, 0, 0)
    $MdpgMountRpy = [double[]]@(0, 0, 0)
    Write-Warning 'Simulation fixture only: mDPG red CAD face center at link_6 origin. No measured adapter calibration.'
}
foreach ($values in @($MdpgMountXyz, $MdpgMountRpy)) {
    if ($null -eq $values -or $values.Count -ne 3) { throw 'Supply three mDPG mount XYZ and RPY values, or -UseCadFaceCenterFixture.' }
    foreach ($value in $values) {
        if ([double]::IsNaN($value) -or [double]::IsInfinity($value)) { throw 'Mount values must be finite.' }
    }
}
$spec = Join-Path $repository 'src/a0509_dynamics_experiment/config/payload_conditions.json'
$specHash = (Get-FileHash -LiteralPath $spec -Algorithm SHA256).Hash.ToLowerInvariant()
$declaration = [ordered]@{
    source = $(if ($UseCadFaceCenterFixture) { 'cad_red_face_center_simulation_fixture' } else { 'user_supplied_transform' })
    xyz_m = @($MdpgMountXyz)
    rpy_rad = @($MdpgMountRpy)
    verified_for_real_robot = $false
    payload_spec_sha256 = $specHash
}
New-Item -ItemType Directory -Path $root -Force | Out-Null
$declPath = Join-Path $root 'mount_declaration.json'
if (Test-Path -LiteralPath $declPath) {
    $old = Get-Content -LiteralPath $declPath -Raw | ConvertFrom-Json
    if ($old.source -ne $declaration.source -or $old.payload_spec_sha256 -ne $specHash -or
        (@($old.xyz_m) -join ',') -ne (@($MdpgMountXyz) -join ',') -or
        (@($old.rpy_rad) -join ',') -ne (@($MdpgMountRpy) -join ',')) {
        throw 'Existing dataset has different tool mount or payload specification. Use another ResultsName.'
    }
} else {
    [System.IO.File]::WriteAllText($declPath, ($declaration | ConvertTo-Json -Depth 4),
                                   [System.Text.UTF8Encoding]::new($false))
}
if ($Play) {
    $pilotReport = Join-Path $root 'pilot_report.json'
    if (-not (Test-Path -LiteralPath $pilotReport) -or
        -not (Get-Content -LiteralPath $pilotReport -Raw | ConvertFrom-Json).pass) {
        throw '2x2 pilot has not passed. Run -Pilot and inspect pilot_report.json first.'
    }
}
$mutex = [System.Threading.Mutex]::new($false, 'Local\A0509_sim_collection')
try { $locked = $mutex.WaitOne(0) }
catch [System.Threading.AbandonedMutexException] { $locked = $true }
if (-not $locked) { $mutex.Dispose(); throw 'Another A0509 collector is preparing.' }
try {
    $active = @(& docker ps --filter 'label=a0509.collect.run' --format '{{.Names}}')
    if ($LASTEXITCODE -ne 0) { throw 'Docker daemon unavailable.' }
    if ($active.Count) { throw "Another A0509 collection is running: $($active -join ', ')" }
    $inspectText = & docker inspect $BaseContainer
    $base = @(($inspectText -join "`n" | ConvertFrom-Json))[0]
    if ($LASTEXITCODE -ne 0 -or $null -eq $base) { throw "Missing base container $BaseContainer" }
    $srcHost = Join-Path $repository 'src'
    $resultsHost = Join-Path $repository 'results'
    $srcMount = @($base.Mounts | Where-Object { $_.Destination -eq '/ros2_ws/src' })
    $resultsMount = @($base.Mounts | Where-Object { $_.Destination -eq '/ros2_ws/results' })
    $installMount = @($base.Mounts | Where-Object { $_.Destination -eq '/ros2_ws/install' })
    $buildMount = @($base.Mounts | Where-Object { $_.Destination -eq '/ros2_ws/build' })
    if ($srcMount.Count -ne 1 -or $resultsMount.Count -ne 1 -or $installMount.Count -ne 1 -or $buildMount.Count -ne 1 -or
        $srcMount[0].Source -ne $srcHost -or $resultsMount[0].Source -ne $resultsHost -or
        $installMount[0].Type -ne 'volume' -or $buildMount[0].Type -ne 'volume') { throw 'Base Docker mounts differ from this repository.' }
    if (-not $base.State.Running) {
        & docker start $BaseContainer | Out-Null
        if ($LASTEXITCODE -ne 0) { throw 'Could not start base container.' }
    }
    $baseProcesses = & docker top $BaseContainer
    if ($LASTEXITCODE -ne 0) { throw 'Could not inspect base container processes.' }
    if (($baseProcesses -join "`n") -match 'run_experiment --simulation') {
        throw 'Original sequential ROS simulation collector is active in the base container.'
    }
    $build = 'source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --symlink-install --packages-select mdpg_1s1_description a0509_dynamics_experiment --event-handlers console_cohesion+'
    & docker exec $BaseContainer bash -lc $build
    if ($LASTEXITCODE -ne 0) { throw 'ROS2 package build failed.' }
    $mode = if ($Pilot) { '--pilot' } else { '--play' }
    $command = "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec python3 -u -m a0509_dynamics_experiment.grid_simulation --results /ros2_ws/results/$ResultsName --mount-xyz $($MdpgMountXyz -join ' ') --mount-rpy $($MdpgMountRpy -join ' ') --payload-spec /ros2_ws/src/a0509_dynamics_experiment/config/payload_conditions.json $mode --campaign $Campaign --trajectory $Trajectory --max-batches $MaxBatches --max-runs $MaxRuns"
    $oldContainer = @(& docker ps -a --filter "name=^/$name`$" --format '{{.Names}}')
    if ($oldContainer.Count) {
        & docker rm $name | Out-Null
        if ($LASTEXITCODE -ne 0) { throw 'Could not remove prior stopped grid container.' }
    }
    $dockerArgs = @('run', '-d', '--name', $name, '--network', 'none',
              '--label', "a0509.collect.run=$ResultsName",
              '--mount', "type=bind,source=$srcHost,destination=/ros2_ws/src,readonly",
              '--mount', "type=bind,source=$resultsHost,destination=/ros2_ws/results",
              '--mount', "type=volume,source=$($installMount[0].Name),destination=/ros2_ws/install,readonly",
              '--mount', "type=volume,source=$($buildMount[0].Name),destination=/ros2_ws/build,readonly",
              '--entrypoint', 'bash', $base.Config.Image, '-lc', $command)
    & docker @dockerArgs
    if ($LASTEXITCODE -ne 0) { throw 'Could not start grid collector.' }
    Write-Host "Started $mode in $name. Output: $root"
    Write-Host "Monitor: docker logs -f $name"
    Write-Host "Status:  .\scripts\play_grid_dataset.ps1 -Campaign $Campaign -Status -ResultsName $ResultsName"
} finally {
    $mutex.ReleaseMutex()
    $mutex.Dispose()
}
