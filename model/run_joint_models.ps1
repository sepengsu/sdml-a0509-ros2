param(
    [string]$Source = 'data/grid_16_wide_v3',
    [string]$Dataset = 'data/wide16_v3_joint_dataset',
    [string]$Models = 'data/wide16_v3_joint_models',
    [string]$Evaluation = 'data/wide16_v3_joint_evaluation',
    [ValidateSet('wide16_v3','trajectory16_legacy')][string]$Profile = 'wide16_v3',
    [int]$Stride = 25,
    [int]$Epochs = 120,
    [int]$BatchSize = 2048,
    [int]$Patience = 10,
    [double]$MinDelta = 0.000001,
    [double]$LearningRate = 0.002,
    [int]$Seed = 42
)

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$python = Join-Path $repo 'model/.venv/Scripts/python.exe'
if (-not (Test-Path -LiteralPath $python -PathType Leaf)) {
    throw "Python environment not found: $python"
}

Push-Location $repo
try {
    & $python -m model.prepare_joint_dataset --source $Source --output $Dataset --stride $Stride --profile $Profile
    if ($LASTEXITCODE -ne 0) { throw 'Dataset preparation failed' }

    foreach ($modelId in @('M1', 'M2', 'M3', 'M4')) {
        & $python -m model.train_joint_models --model $modelId --data $Dataset --output $Models `
            --epochs $Epochs --batch-size $BatchSize --patience $Patience `
            --min-delta $MinDelta --learning-rate $LearningRate --seed $Seed
        if ($LASTEXITCODE -ne 0) { throw "Training failed: $modelId" }
    }

    & $python -m model.evaluate_joint_models --data $Dataset --models $Models --output $Evaluation
    if ($LASTEXITCODE -ne 0) { throw 'Evaluation failed' }
} finally {
    Pop-Location
}
