param(
    [switch]$Resume,
    [switch]$InitializeOnly
)

$ErrorActionPreference = 'Continue'
$experimentRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$taskDirectory = Join-Path $experimentRoot 'public\Environment\real'
$resultsFile = Join-Path $experimentRoot 'results.csv'

$schemes = @(
    @{ Index = 1; Name = 'scheme1'; Directory = 'Scheme1_Full_GeoTime_LSENLP'; Script = 'RunMe1.m' },
    @{ Index = 2; Name = 'scheme2'; Directory = 'Scheme2_NoGeo_Time_LSENLP'; Script = 'RunMe2.m' },
    @{ Index = 3; Name = 'scheme3'; Directory = 'Scheme3_GeoTime_MaxNLP'; Script = 'RunMe3.m' },
    @{ Index = 4; Name = 'scheme4'; Directory = 'Scheme4_EqualTime_BodyOnlyNLP'; Script = 'RunMe4.m' }
)

$taskIds = Get-ChildItem -LiteralPath $taskDirectory -Filter '*.mat' |
    ForEach-Object { [int]$_.BaseName } |
    Sort-Object

if ($taskIds.Count -eq 0) {
    throw "No MAT tasks were found in $taskDirectory"
}

if (-not $Resume -or -not (Test-Path -LiteralPath $resultsFile)) {
    $headers = @(
        'scheme1_task_id','scheme1_success','scheme1_ipopt_cpu_time','scheme1_collision_percent','',
        'scheme2_task_id','scheme2_success','scheme2_ipopt_cpu_time','scheme2_collision_percent','',
        'scheme3_task_id','scheme3_success','scheme3_ipopt_cpu_time','scheme3_collision_percent','',
        'scheme4_task_id','scheme4_success','scheme4_ipopt_cpu_time','scheme4_collision_percent'
    )
    $lines = [System.Collections.Generic.List[string]]::new()
    $lines.Add(($headers -join ','))
    foreach ($taskId in $taskIds) {
        $block = "$taskId,NaN,NaN,NaN"
        $lines.Add("$block,,$block,,$block,,$block")
    }
    [System.IO.File]::WriteAllLines($resultsFile,$lines,[System.Text.UTF8Encoding]::new($false))
}

if ($InitializeOnly) {
    Write-Host "Initialized unified results table: $resultsFile"
    return
}

$previousTaskId = $env:EXPERIMENT_TASK_ID
$previousTaskSource = $env:EXPERIMENT_TASK_SOURCE
$env:EXPERIMENT_TASK_SOURCE = 'real'
$failures = [System.Collections.Generic.List[string]]::new()

try {
    foreach ($scheme in $schemes) {
        $runScript = Join-Path (Join-Path $experimentRoot $scheme.Directory) $scheme.Script
        $matlabScript = $runScript.Replace('\','/').Replace("'","''")
        foreach ($taskId in $taskIds) {
            $env:EXPERIMENT_TASK_ID = [string]$taskId
            Write-Host "`n===== $($scheme.Name), task $taskId ====="
            & matlab -batch "run('$matlabScript')"
            if ($LASTEXITCODE -ne 0) {
                $failures.Add("$($scheme.Name):task_$('{0:D2}' -f $taskId)")
                $csvLines = Get-Content -LiteralPath $resultsFile
                for ($lineIndex = 1; $lineIndex -lt $csvLines.Count; $lineIndex++) {
                    $fields = $csvLines[$lineIndex] -split ',',-1
                    if ([int]$fields[0] -eq $taskId) {
                        $base = ($scheme.Index-1)*5
                        $fields[$base+1] = '0'
                        $csvLines[$lineIndex] = $fields -join ','
                        break
                    }
                }
                [System.IO.File]::WriteAllLines($resultsFile,$csvLines,[System.Text.UTF8Encoding]::new($false))
                Write-Warning "Run failed before normal evaluation; success was recorded as 0."
            }
        }
    }
}
finally {
    $env:EXPERIMENT_TASK_ID = $previousTaskId
    $env:EXPERIMENT_TASK_SOURCE = $previousTaskSource
}

Write-Host "`nUnified results: $resultsFile"
if ($failures.Count -gt 0) {
    Write-Warning ("Failed runs: " + ($failures -join ', '))
    exit 1
}
