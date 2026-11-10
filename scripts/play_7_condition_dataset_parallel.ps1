<#
Run the 7 x 14 x 3 MuJoCo campaign across isolated simulated A0509 workers.
Each (condition, repeat) shard belongs to one container and one ROS domain.
No real robot or Doosan emulator is started.
#>
param(
    [switch]$Play,
    [switch]$Status,
    [ValidateRange(1, 8)][int]$RobotCount = 2,
    [string]$ResultsName = 'recollection_7conditions_grasp_v2',
    [ValidateRange(0, 232)][int]$BaseSimDomainId = 174,
    [string]$BaseContainer = 'a0509-dynamics',
    [switch]$UseCadFaceCenterFixture,
    [double[]]$MdpgMountXyz,
    [double[]]$MdpgMountRpy,
    [switch]$RecordVideo
)

$ErrorActionPreference = 'Stop'
if ($ResultsName -notmatch '^[A-Za-z0-9_-]+$') { throw 'ResultsName must use letters, digits, _ or -.' }
if ($BaseContainer -notmatch '^[A-Za-z0-9_.-]+$') { throw 'BaseContainer has unsupported characters.' }
if ($BaseSimDomainId + $RobotCount - 1 -gt 232) { throw 'One or more ROS domain IDs exceed 232.' }
$repository = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$resultsRoot = Join-Path $repository (Join-Path 'results' $ResultsName)
$runsRoot = Join-Path $resultsRoot 'parallel_runs'

if ($Status) {
    $runs = @(Get-ChildItem -LiteralPath $runsRoot -Directory -ErrorAction SilentlyContinue | Sort-Object Name -Descending)
    if ($runs.Count -eq 0) { Write-Host "No parallel run under $resultsRoot"; return }
    $run = $runs[0]
    $manifest = Get-Content -LiteralPath (Join-Path $run.FullName 'manifest.json') -Raw | ConvertFrom-Json
    $finished = @(Get-ChildItem -LiteralPath $resultsRoot -Recurse -File -Filter metadata.json -ErrorAction SilentlyContinue |
        Where-Object { $_.FullName -notmatch '[\\/]parallel_runs[\\/]' } |
        Where-Object { Test-Path -LiteralPath (Join-Path $_.DirectoryName 'rosbag/metadata.yaml') })
    Write-Host "Campaign: $ResultsName | complete metadata+bag: $($finished.Count)/294 | run: $($run.Name)"
    foreach ($index in 0..($manifest.robot_count - 1)) {
        $statusPath = Join-Path $run.FullName ('worker_{0:00}.status.json' -f ($index + 1))
        $container = $manifest.containers[$index]
        if (Test-Path -LiteralPath $statusPath) {
            $state = Get-Content -LiteralPath $statusPath -Raw | ConvertFrom-Json
            Write-Host ("{0}: {1} {2}/{3} | {4} | {5}" -f $container, $state.state,
                        $state.completed, $state.assigned, $state.current_job, $state.error)
        } else {
            Write-Host "$container`: no status file yet"
        }
    }
    & docker ps -a --filter "label=a0509.collect.run=$($run.Name)" --format 'table {{.Names}}\t{{.Status}}'
    return
}

$conditions = @(
    [pscustomobject]@{ Id='bare_p0';   Gripper='none';     PayloadKg=0.0 },
    [pscustomobject]@{ Id='ag145_p0';  Gripper='ag145';    PayloadKg=0.0 },
    [pscustomobject]@{ Id='ag145_p05'; Gripper='ag145';    PayloadKg=0.5 },
    [pscustomobject]@{ Id='ag145_p10'; Gripper='ag145';    PayloadKg=1.0 },
    [pscustomobject]@{ Id='mdpg_p0';   Gripper='mdpg_1s1'; PayloadKg=0.0 },
    [pscustomobject]@{ Id='mdpg_p05';  Gripper='mdpg_1s1'; PayloadKg=0.5 },
    [pscustomobject]@{ Id='mdpg_p10';  Gripper='mdpg_1s1'; PayloadKg=1.0 }
)
$jobs = @(
    for ($conditionIndex = 0; $conditionIndex -lt $conditions.Count; $conditionIndex++) {
        $condition = $conditions[$conditionIndex]
        foreach ($repeat in 1..3) {
            $worker = (($conditionIndex * 3) + $repeat - 1) % $RobotCount
            foreach ($trajectory in 1..14) {
                [ordered]@{
                    condition = $condition.Id
                    gripper = $condition.Gripper
                    payload_kg = $condition.PayloadKg
                    trajectory = "T$trajectory"
                    repeat = $repeat
                    worker = $worker
                }
            }
        }
    }
)
if ($jobs.Count -ne 294) { throw "Expected 294 jobs, found $($jobs.Count)." }
Write-Host "Simulation-only plan: 294 trials, $RobotCount independent A0509 workers."
foreach ($index in 0..($RobotCount - 1)) {
    Write-Host ("  worker {0}: domain {1}, {2} assigned trials" -f ($index + 1),
                ($BaseSimDomainId + $index), @($jobs | Where-Object { $_.worker -eq $index }).Count)
}
if (-not $Play) {
    Write-Host 'Plan only. Add -Play to start detached Docker workers; use -Status to monitor.'
    return
}

if ($UseCadFaceCenterFixture) {
    if ($PSBoundParameters.ContainsKey('MdpgMountXyz') -or $PSBoundParameters.ContainsKey('MdpgMountRpy')) {
        throw 'Choose either -UseCadFaceCenterFixture or explicit mDPG mount arrays.'
    }
    $MdpgMountXyz = [double[]]@(0, 0, 0)
    $MdpgMountRpy = [double[]]@(0, 0, 0)
    Write-Warning 'Unmeasured simulation fixture: red CAD face center at link_6 origin. Not a real adapter calibration.'
}
foreach ($values in @($MdpgMountXyz, $MdpgMountRpy)) {
    if ($null -eq $values -or $values.Count -ne 3) {
        throw 'MdpgMountXyz and MdpgMountRpy must each contain three declared values.'
    }
    foreach ($value in $values) {
        if ([double]::IsNaN($value) -or [double]::IsInfinity($value)) {
            throw 'mDPG mount values must be finite.'
        }
    }
}
if ($RobotCount -gt 4) {
    Write-Warning 'This Docker engine has 16 GB RAM. More than four software-rendered MuJoCo workers may slow down or stop.'
}
$specHost = Join-Path $repository 'src/a0509_dynamics_experiment/config/payload_conditions.json'
if (-not (Test-Path -LiteralPath $specHost)) { throw "Missing payload spec: $specHost" }
$specSha256 = (Get-FileHash -LiteralPath $specHost -Algorithm SHA256).Hash.ToLowerInvariant()
$mountSource = if ($UseCadFaceCenterFixture) { 'cad_red_face_center_simulation_fixture' } else { 'user_supplied_transform' }
$declaration = [ordered]@{
    source = $mountSource
    xyz_m = @($MdpgMountXyz)
    rpy_rad = @($MdpgMountRpy)
    verified_for_real_robot = $false
    note = 'Simulation-only declared mount; CAD has an internal 180-degree X rotation. Real adapter, TCP and pneumatic IO remain unverified.'
    payload_spec_sha256 = $specSha256
}
$existingDeclaration = Join-Path $resultsRoot 'mount_declaration.json'
if (Test-Path -LiteralPath $existingDeclaration) {
    $old = Get-Content -LiteralPath $existingDeclaration -Raw | ConvertFrom-Json
    if ($old.source -ne $mountSource -or $old.payload_spec_sha256 -ne $specSha256 -or
        @($old.xyz_m).Count -ne 3 -or @($old.rpy_rad).Count -ne 3) {
        throw 'Existing dataset mount declaration or payload specification differs. Use a new ResultsName.'
    }
    for ($i = 0; $i -lt 3; $i++) {
        if ([math]::Abs([double]$old.xyz_m[$i] - $MdpgMountXyz[$i]) -gt 1e-12 -or
            [math]::Abs([double]$old.rpy_rad[$i] - $MdpgMountRpy[$i]) -gt 1e-12) {
            throw 'Existing dataset mDPG mount differs. Use a new ResultsName.'
        }
    }
}

$runningWorkers = @(& docker ps --filter "label=a0509.collect.results=$ResultsName" --format '{{.Names}}')
if ($LASTEXITCODE -ne 0) { throw 'Docker daemon is unavailable.' }
if ($runningWorkers.Count -gt 0) { throw "Parallel workers already running: $($runningWorkers -join ', ')" }
$baseInspectText = & docker inspect $BaseContainer
if ($LASTEXITCODE -ne 0) { throw "Missing base Docker container: $BaseContainer" }
$base = @(($baseInspectText -join "`n" | ConvertFrom-Json))[0]
$srcHost = Join-Path $repository 'src'
$resultsHost = Join-Path $repository 'results'
$srcMount = @($base.Mounts | Where-Object { $_.Destination -eq '/ros2_ws/src' })
$resultsMount = @($base.Mounts | Where-Object { $_.Destination -eq '/ros2_ws/results' })
$installMount = @($base.Mounts | Where-Object { $_.Destination -eq '/ros2_ws/install' })
$buildMount = @($base.Mounts | Where-Object { $_.Destination -eq '/ros2_ws/build' })
if ($srcMount.Count -ne 1 -or $resultsMount.Count -ne 1 -or $installMount.Count -ne 1 -or $buildMount.Count -ne 1 -or
    $srcMount[0].Source -ne $srcHost -or $resultsMount[0].Source -ne $resultsHost -or
    $installMount[0].Type -ne 'volume' -or $buildMount[0].Type -ne 'volume') {
    throw 'Base container mounts do not match this repository or lack shared ROS build/install volumes.'
}
if ($base.State.Running) {
    $processes = & docker top $BaseContainer
    if ($LASTEXITCODE -ne 0) { throw 'Could not inspect base container processes.' }
    if (($processes -join "`n") -match 'run_experiment --simulation') {
        throw 'The original sequential collection is running. Stop it before starting parallel workers.'
    }
} else {
    & docker start $BaseContainer | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "Could not start base container $BaseContainer" }
}

$build = 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon build --symlink-install --packages-select mdpg_1s1_description sdml_a0509_description a0509_mujoco_sim sdml_a0509_bringup a0509_dynamics_experiment --event-handlers console_cohesion+'
& docker exec $BaseContainer bash -lc $build
if ($LASTEXITCODE -ne 0) { throw 'ROS2 package build failed; no parallel worker was started.' }
$specInside = '/ros2_ws/src/a0509_dynamics_experiment/config/payload_conditions.json'
$check = "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 -m a0509_dynamics_experiment.payload_model --check-all $specInside"
& docker exec $BaseContainer bash -lc $check
if ($LASTEXITCODE -ne 0) { throw 'Payload validation failed; no parallel worker was started.' }

New-Item -ItemType Directory -Force -Path $runsRoot | Out-Null
if (-not (Test-Path -LiteralPath $existingDeclaration)) {
    $utf8 = [System.Text.UTF8Encoding]::new($false)
    [System.IO.File]::WriteAllText($existingDeclaration, ($declaration | ConvertTo-Json -Depth 5), $utf8)
}
$runId = (Get-Date -Format 'yyyyMMdd_HHmmss') + '_' + [guid]::NewGuid().ToString('N').Substring(0, 6)
$runRoot = Join-Path $runsRoot $runId
New-Item -ItemType Directory -Path $runRoot | Out-Null
$containers = @(foreach ($index in 0..($RobotCount - 1)) { 'a0509-col-{0}-w{1:00}' -f $runId, ($index + 1) })
$manifest = [ordered]@{
    schema_version = 1
    results_name = $ResultsName
    run_id = $runId
    robot_count = $RobotCount
    base_domain_id = $BaseSimDomainId
    payload_spec_sha256 = $specSha256
    mount = $declaration
    record_video = [bool]$RecordVideo
    containers = $containers
    jobs = $jobs
}
$manifestPath = Join-Path $runRoot 'manifest.json'
$utf8 = [System.Text.UTF8Encoding]::new($false)
[System.IO.File]::WriteAllText($manifestPath, ($manifest | ConvertTo-Json -Depth 10), $utf8)

$srcMountArg = "type=bind,source=$srcHost,destination=/ros2_ws/src,readonly"
$resultsMountArg = "type=bind,source=$resultsHost,destination=/ros2_ws/results"
$installMountArg = "type=volume,source=$($installMount[0].Name),destination=/ros2_ws/install,readonly"
$buildMountArg = "type=volume,source=$($buildMount[0].Name),destination=/ros2_ws/build,readonly"
$insideManifest = "/ros2_ws/results/$ResultsName/parallel_runs/$runId/manifest.json"
foreach ($index in 0..($RobotCount - 1)) {
    $domain = $BaseSimDomainId + $index
    $workerCommand = "Xvfb :99 -screen 0 1280x720x24 >/tmp/xvfb.log 2>&1 & sleep 1 && source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec python3 -u /ros2_ws/src/a0509_dynamics_experiment/scripts/parallel_collection_worker.py --manifest $insideManifest --index $index"
    $dockerArgs = @('run', '-d', '--name', $containers[$index],
                    '--label', "a0509.collect.results=$ResultsName",
                    '--label', "a0509.collect.run=$runId",
                    '--network', 'bridge', '--shm-size', '512m',
                    '-e', "ROS_DOMAIN_ID=$domain", '-e', 'DISPLAY=:99',
                    '-e', 'LIBGL_ALWAYS_SOFTWARE=1', '-e', 'ROS_LOCALHOST_ONLY=1',
                    '--mount', $srcMountArg, '--mount', $resultsMountArg,
                    '--mount', $installMountArg, '--mount', $buildMountArg,
                    '--entrypoint', 'bash',
                    $base.Image, '-lc', $workerCommand)
    $id = & docker @dockerArgs
    if ($LASTEXITCODE -ne 0) { throw "Worker $($index + 1) failed to start. Inspect $runRoot" }
    Write-Host "Started $($containers[$index]) (domain $domain, id $($id.Substring(0, 12)))"
}
Write-Host "Detached collection started. Progress: .\scripts\play_7_condition_dataset_parallel.ps1 -Status -ResultsName $ResultsName"
Write-Host "Worker logs: $runRoot"
