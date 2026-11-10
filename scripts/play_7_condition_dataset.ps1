<#
Plan or play 7 tool/payload conditions x T1..T14 x 3 independent trials.
No real-robot mode is used. The CAD-face mount fixture is a declared simulation assumption.
#>
param(
    [switch]$Play,
    [switch]$RecordVideo,
    [switch]$UseCadFaceCenterFixture,
    [double[]]$MdpgMountXyz,
    [double[]]$MdpgMountRpy,
    [string]$Container = 'a0509-dynamics',
    [string]$ResultsName = 'recollection_7conditions',
    [int]$SimDomainId = 174
)

$ErrorActionPreference = 'Stop'
if ($ResultsName -notmatch '^[A-Za-z0-9_-]+$') { throw 'ResultsName must use letters, digits, _ or -.' }
if ($Container -notmatch '^[A-Za-z0-9_.-]+$') { throw 'Container name has unsupported characters.' }
if ($SimDomainId -lt 0 -or $SimDomainId -gt 232) { throw 'SimDomainId must be 0..232.' }

$repository = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$resultsRoot = Join-Path $repository (Join-Path 'results' $ResultsName)
New-Item -ItemType Directory -Force -Path $resultsRoot | Out-Null

$conditions = @(
    [pscustomobject]@{ Id='bare_p0';   Gripper='none';     PayloadKg=0.0 }
    [pscustomobject]@{ Id='ag145_p0';  Gripper='ag145';    PayloadKg=0.0 }
    [pscustomobject]@{ Id='ag145_p05'; Gripper='ag145';    PayloadKg=0.5 }
    [pscustomobject]@{ Id='ag145_p10'; Gripper='ag145';    PayloadKg=1.0 }
    [pscustomobject]@{ Id='mdpg_p0';   Gripper='mdpg_1s1'; PayloadKg=0.0 }
    [pscustomobject]@{ Id='mdpg_p05';  Gripper='mdpg_1s1'; PayloadKg=0.5 }
    [pscustomobject]@{ Id='mdpg_p10';  Gripper='mdpg_1s1'; PayloadKg=1.0 }
)
$jobs = @(
    foreach ($condition in $conditions) {
        foreach ($trajectory in 1..14) {
            foreach ($repeat in 1..3) {
                [pscustomobject]@{
                    TrialId    = '{0}_T{1}_r{2:000}' -f $condition.Id, $trajectory, $repeat
                    Condition  = $condition.Id
                    Gripper    = $condition.Gripper
                    PayloadKg  = $condition.PayloadKg
                    Trajectory = "T$trajectory"
                    Repeat     = $repeat
                }
            }
        }
    }
)
if ($jobs.Count -ne 294) { throw "Expected 294 jobs, found $($jobs.Count)." }
$planPath = Join-Path $resultsRoot 'collection_plan.csv'
$jobs | Export-Csv -LiteralPath $planPath -NoTypeInformation -Encoding UTF8
Write-Host "Planned $($jobs.Count) simulation trials: $planPath"
if (-not $Play) {
    Write-Host 'Dry run only. Use -UseCadFaceCenterFixture for the unmeasured video/CAD alignment or supply a measured mDPG mount with -Play.'
    return
}

if ($UseCadFaceCenterFixture) {
    if ($PSBoundParameters.ContainsKey('MdpgMountXyz') -or $PSBoundParameters.ContainsKey('MdpgMountRpy')) {
        throw 'Choose either -UseCadFaceCenterFixture or explicit mDPG mount arrays.'
    }
    # CAD red face is centered at the raw origin; its internal 180-degree X
    # rotation is already applied in the URDF and MuJoCo tool model.
    $MdpgMountXyz = [double[]]@(0, 0, 0)
    $MdpgMountRpy = [double[]]@(0, 0, 0)
    Write-Warning 'Using an unmeasured simulation fixture: red CAD face center at link_6 origin. Not a real adapter calibration.'
}

# Check every declared model input before the first trial, so missing values
# cannot leave a partially collected 294-trial campaign.
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
$specHost = Join-Path $repository 'src/a0509_dynamics_experiment/config/payload_conditions.json'
if (-not (Test-Path -LiteralPath $specHost)) { throw "Missing payload spec: $specHost" }
$specSha256 = (Get-FileHash -LiteralPath $specHost -Algorithm SHA256).Hash.ToLowerInvariant()
$running = & docker inspect --format '{{.State.Running}}' $Container 2>$null
if ($LASTEXITCODE -ne 0 -or $running.Trim() -ne 'true') { throw "Docker container $Container is not running." }

& docker exec $Container bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon build --symlink-install --packages-select mdpg_1s1_description sdml_a0509_description a0509_mujoco_sim sdml_a0509_bringup a0509_dynamics_experiment --event-handlers console_cohesion+'
if ($LASTEXITCODE -ne 0) { throw 'ROS2 package build failed.' }
$specInside = '/ros2_ws/src/a0509_dynamics_experiment/config/payload_conditions.json'
$check = "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 -m a0509_dynamics_experiment.payload_model --check-all $specInside"
& docker exec $Container bash -lc $check
if ($LASTEXITCODE -ne 0) { throw 'Payload specs are incomplete or invalid; no trial was started.' }

$invariant = [System.Globalization.CultureInfo]::InvariantCulture
$mountXyzText = @($MdpgMountXyz | ForEach-Object { $_.ToString('R', $invariant) })
$mountRpyText = @($MdpgMountRpy | ForEach-Object { $_.ToString('R', $invariant) })
$mountDeclaration = [ordered]@{
    source = $(if ($UseCadFaceCenterFixture) { 'cad_red_face_center_simulation_fixture' } else { 'user_supplied_transform' })
    xyz_m = @($MdpgMountXyz)
    rpy_rad = @($MdpgMountRpy)
    verified_for_real_robot = $false
    note = 'The model rotates the raw CAD 180 degrees about X internally. This declaration applies only to MuJoCo dataset collection; adapter, TCP and pneumatic IO remain unverified.'
    payload_spec_sha256 = $specSha256
}
$mountDeclaration | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $resultsRoot 'mount_declaration.json') -Encoding UTF8
$number = 0
foreach ($job in $jobs) {
    $number++
    $trialRoot = Join-Path $resultsRoot (Join-Path $job.Condition ("repeat_{0:000}" -f $job.Repeat))
    $trialPath = Join-Path $trialRoot (Join-Path 'simulation' $job.Trajectory)
    $finished = @(Get-ChildItem -LiteralPath $trialPath -Directory -Filter 'trial_*' -ErrorAction SilentlyContinue | Where-Object {
        $metadataPath = Join-Path $_.FullName 'metadata.json'
        $csvPath = Join-Path $_.FullName 'dynamics.csv'
        $bagPath = Join-Path $_.FullName 'rosbag/metadata.yaml'
        if (-not ((Test-Path $metadataPath) -and (Test-Path $csvPath) -and (Test-Path $bagPath))) { return $false }
        try {
            $metadata = Get-Content -LiteralPath $metadataPath -Raw | ConvertFrom-Json
            if ($metadata.gripper_type -ne $job.Gripper -or
                [double]$metadata.payload_mass_kg -ne $job.PayloadKg -or
                $metadata.trajectory_id -ne $job.Trajectory) { return $false }
            if ($job.PayloadKg -gt 0 -and $metadata.payload_model.spec_sha256 -ne $specSha256) {
                return $false
            }
            if ($job.Gripper -eq 'mdpg_1s1') {
                $savedXyz = @($metadata.gripper_model.mount_xyz_m)
                $savedRpy = @($metadata.gripper_model.mount_rpy_rad)
                if ($savedXyz.Count -ne 3 -or $savedRpy.Count -ne 3) { return $false }
                for ($i = 0; $i -lt 3; $i++) {
                    if ([math]::Abs([double]$savedXyz[$i] - $MdpgMountXyz[$i]) -gt 1e-12 -or
                        [math]::Abs([double]$savedRpy[$i] - $MdpgMountRpy[$i]) -gt 1e-12) {
                        return $false
                    }
                }
            }
            return ($metadata.execution_success -eq $true -and
                    $metadata.sample_count -gt 0 -and
                    $metadata.quality.rows_written -eq $metadata.sample_count)
        } catch { return $false }
    })
    if ($finished.Count -gt 0) {
        Write-Host "[$number/294] Already complete: $($job.TrialId)"
        continue
    }
    $insideRoot = "/ros2_ws/results/$ResultsName/$($job.Condition)/repeat_$('{0:000}' -f $job.Repeat)"
    $massText = $job.PayloadKg.ToString('0.0', $invariant)
    $arguments = @('--simulation', '--trajectory', $job.Trajectory,
                   '--gripper-type', $job.Gripper, '--payload-mass', $massText,
                   '--results-dir', $insideRoot, '--sim-domain-id', "$SimDomainId",
                   '--sample-hz', '100')
    if ($job.Gripper -eq 'mdpg_1s1') {
        $arguments += @('--mdpg-dynamics-model', 'box_approx', '--mdpg-mount-xyz')
        $arguments += $mountXyzText
        $arguments += '--mdpg-mount-rpy'
        $arguments += $mountRpyText
    }
    if ($job.PayloadKg -gt 0) { $arguments += @('--payload-spec-file', $specInside) }
    if ($RecordVideo) { $arguments += '--record-video' }
    $command = 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && cd /ros2_ws && ros2 run a0509_dynamics_experiment run_experiment ' + [string]::Join(' ', $arguments)
    Write-Host "[$number/294] Running $($job.TrialId)"
    & docker exec $Container bash -lc $command
    if ($LASTEXITCODE -ne 0) { throw "Trial failed: $($job.TrialId). Correct the cause, then rerun this script to resume." }
}
Write-Host "Completed all $($jobs.Count) planned simulation trials under $resultsRoot"
