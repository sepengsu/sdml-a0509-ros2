<# Run the same wide16 T1-T16 trajectories ten times in each of seven conditions. #>
param(
    [string]$ResultsName = 'grid_16_wide_v3',
    [ValidateRange(4,64)][int]$MemoryGb = 8
)

$ErrorActionPreference = 'Stop'
if ($ResultsName -notmatch '^[A-Za-z0-9_-]+$') { throw 'Invalid ResultsName.' }
$launcher = Join-Path $PSScriptRoot 'play_grid_dataset.ps1'
$container = "a0509-grid-$ResultsName"

function Wait-Collector([string]$Stage) {
    $exitText = & docker wait $container
    if ($LASTEXITCODE -ne 0) { throw "Could not wait for $Stage container $container." }
    $exitCode = [int]$exitText
    if ($exitCode -ne 0) {
        & docker logs --tail 100 $container
        if ($exitCode -eq 137) {
            $oomKilled = & docker inspect $container --format '{{.State.OOMKilled}}'
            if ($LASTEXITCODE -eq 0 -and $oomKilled -eq 'true') {
                throw "$Stage stopped with exit code 137: Docker killed it for exceeding the memory limit."
            }
            throw "$Stage stopped with exit code 137 (SIGKILL), but Docker did not report an OOM kill. Check whether the container was stopped or the host restarted."
        }
        if ($exitCode -eq 139) {
            throw "$Stage stopped with exit code 139 (segmentation fault in Python or a native library). Check the faulthandler trace above and the host system log before resuming."
        }
        throw "$Stage stopped with exit code $exitCode. Check the logs above."
    }
}

& $launcher -Campaign wide16 -Pilot -UseCadFaceCenterFixture -Trials 10 -ResultsName $ResultsName -MemoryGb $MemoryGb
Wait-Collector 'Pilot'

& $launcher -Campaign wide16 -Play -UseCadFaceCenterFixture -Trials 10 -ResultsName $ResultsName -MemoryGb $MemoryGb
Wait-Collector 'Collection'

& $launcher -Campaign wide16 -Status -ResultsName $ResultsName -Trials 10
