<# Independent MuJoCo simulation. No ROS2 or Docker required. #>
[CmdletBinding()]
param(
    [ValidateSet('setup','plan','analyze','offline','pilot','collect','report','status')]
    [string]$Mode='plan',
    [string]$DataRoot='F:\연구\grad',
    [string]$SourceRoot='H:\a0509-data\training_datasets_2026-11-22\data\friction_diverse_v1',
    [string]$LegacySpeedRoot='H:\a0509-data\wide16_speed_sim_v1',
    [string]$ResultsName='friction_sim_v4',
    [string]$Config,
    [ValidateSet('none','ag145','mdpg_1s1')][string]$Tool,
    [string[]]$Conditions,[string[]]$Trajectories,
    [ValidateRange(1,10000)][int]$MaxJobs
)
$ErrorActionPreference='Stop'
$repository=(Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$venv=Join-Path $repository '.venv-friction-sim'
$python=Join-Path $venv 'Scripts/python.exe'
$entry=Join-Path $PSScriptRoot 'collect_friction_simulation.py'
$package=Join-Path $repository 'src/a0509_friction_sim/a0509_friction_sim'
if($ResultsName -notmatch '^[A-Za-z0-9_-]+$'){throw 'Invalid ResultsName.'}
if($Mode -eq 'setup'){
    if(-not (Test-Path -LiteralPath $python)){
        $existingPython=Join-Path $repository '.venv-friction/Scripts/python.exe'
        if(Test-Path -LiteralPath $existingPython){ & $existingPython -m venv $venv }
        elseif(Get-Command py -ErrorAction SilentlyContinue){ & py -3.12 -m venv $venv }
        else { & python -m venv $venv }
        if($LASTEXITCODE -ne 0){throw 'Could not create .venv-friction-sim. Install Python 3.10-3.12.'}
    }
    $requirements=Join-Path $package 'requirements.txt'
    $cache=Join-Path $repository 'output/friction-package-cache'
    if(Get-Command uv -ErrorAction SilentlyContinue){
        & uv pip install --quiet --python $python --cache-dir $cache -r $requirements
    } else { & $python -m pip install --only-binary=:all: --cache-dir $cache -r $requirements }
    if($LASTEXITCODE -ne 0){throw 'Simulation environment installation failed.'}
    & $python -m pip check
    if($LASTEXITCODE -ne 0){throw 'Simulation dependency check failed.'}
    & $python $entry --help | Out-Null
    if($LASTEXITCODE -ne 0){throw 'Simulation imports failed.'}
    Write-Host "Simulation environment ready: $python"
    return
}
if(-not (Test-Path -LiteralPath $python)){throw 'Run -Mode setup first.'}
if([string]::IsNullOrWhiteSpace($Config)){$Config=Join-Path $package 'config.json'}
$SourceRoot=(Resolve-Path -LiteralPath $SourceRoot).Path
$Config=(Resolve-Path -LiteralPath $Config).Path
if(-not [IO.Path]::IsPathRooted($DataRoot)){$DataRoot=Join-Path $repository $DataRoot}
$output=[IO.Path]::GetFullPath((Join-Path $DataRoot $ResultsName))
if(-not (Test-Path -LiteralPath ([IO.Path]::GetPathRoot($output)))){throw "Output drive is unavailable: $DataRoot"}
$arguments=@('-u',$entry,'--mode',$Mode,'--config',$Config,'--source',$SourceRoot,
             '--legacy-speed-root',$LegacySpeedRoot,'--output',$output)
if($Tool){$arguments+=@('--tool',$Tool)}
if($Conditions){$arguments+='--conditions';$arguments+=$Conditions}
if($Trajectories){$arguments+='--trajectories';$arguments+=$Trajectories}
if($PSBoundParameters.ContainsKey('MaxJobs')){$arguments+=@('--max-jobs',[string]$MaxJobs)}
$mutex=$null
try{
    if($Mode -in @('pilot','collect')){
        $mutex=[Threading.Mutex]::new($false,'Local\A0509_sim_collection')
        try { $locked=$mutex.WaitOne(0) } catch [Threading.AbandonedMutexException] { $locked=$true }
        if(-not $locked){$mutex.Dispose();$mutex=$null;throw 'Another A0509 simulation collector is running.'}
    }
    & $python @arguments
    if($LASTEXITCODE -ne 0){throw "MuJoCo simulation stage failed (exit $LASTEXITCODE). Inspect $output and rerun the same stage to resume."}
}finally{
    if($null -ne $mutex){$mutex.ReleaseMutex();$mutex.Dispose()}
}
