[CmdletBinding()]
param(
    [switch]$Scheme1,
    [switch]$Scheme2,
    [switch]$Scheme3,
    [switch]$Scheme4,
    [switch]$Resume,
    [switch]$RetryFailed,
    [switch]$Clear,
    [switch]$InitializeOnly
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

$experimentRoot = [System.IO.Path]::GetFullPath(
    (Split-Path -Parent $MyInvocation.MyCommand.Path))
# Change only this value to switch the experiment data source.
# $taskSource = 'Data_test'
$taskSource = 'real'
$taskDirectory = Join-Path $experimentRoot "public\Environment\$taskSource"
$resultsFile = Join-Path $experimentRoot 'results.csv'

$schemes = @(
    [pscustomobject]@{ Index = 1; Name = 'scheme1'; Directory = 'Scheme1_Full_GeoTime_LSENLP'; Script = 'RunMe1.m' },
    [pscustomobject]@{ Index = 2; Name = 'scheme2'; Directory = 'Scheme2_NoGeo_Time_LSENLP'; Script = 'RunMe2.m' },
    [pscustomobject]@{ Index = 3; Name = 'scheme3'; Directory = 'Scheme3_GeoTime_MaxNLP'; Script = 'RunMe3.m' },
    [pscustomobject]@{ Index = 4; Name = 'scheme4'; Directory = 'Scheme4_body_only_baseline'; Script = 'RunMe4.m' }
)

$expectedHeaders = @(
    'scheme1_task_id','scheme1_success','scheme1_ipopt_cpu_time','scheme1_collision_percent','',
    'scheme2_task_id','scheme2_success','scheme2_ipopt_cpu_time','scheme2_collision_percent','',
    'scheme3_task_id','scheme3_success','scheme3_ipopt_cpu_time','scheme3_collision_percent','',
    'scheme4_task_id','scheme4_success','scheme4_ipopt_cpu_time','scheme4_collision_percent'
)

$standardTemporaryFiles = @(
    'Area',
    'PPP',
    'PV',
    'ig.INIVAL',
    'OBCAData.dat',
    'written_initial_guess_data.mat'
)
$scheme4TemporaryFiles = @(
    'Area_scheme4',
    'PPP_scheme4',
    'PV_scheme4',
    'ig_scheme4.INIVAL',
    'OBCAData_scheme4.dat',
    'written_initial_guess_data_scheme4.mat'
)

function Assert-PathInsideRoot {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$Root
    )

    $fullPath = [System.IO.Path]::GetFullPath($Path)
    $fullRoot = [System.IO.Path]::GetFullPath($Root).TrimEnd('\')
    $rootPrefix = $fullRoot + '\'
    if (-not $fullPath.StartsWith(
            $rootPrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to operate outside the experiment root: $fullPath"
    }
    return $fullPath
}

function Remove-GeneratedDirectory {
    param([Parameter(Mandatory = $true)][string]$Path)

    $safePath = Assert-PathInsideRoot -Path $Path -Root $experimentRoot
    if (Test-Path -LiteralPath $safePath -PathType Container) {
        Remove-Item -LiteralPath $safePath -Recurse -Force
        Write-Host "Removed directory: $safePath"
    }
}

function Remove-GeneratedFile {
    param([Parameter(Mandatory = $true)][string]$Path)

    $safePath = Assert-PathInsideRoot -Path $Path -Root $experimentRoot
    if (Test-Path -LiteralPath $safePath -PathType Leaf) {
        Remove-Item -LiteralPath $safePath -Force
        Write-Host "Removed file: $safePath"
    }
}

function Clear-GeneratedArtifacts {
    Remove-GeneratedFile -Path $resultsFile

    foreach ($scheme in $schemes) {
        $schemeRoot = Assert-PathInsideRoot `
            -Path (Join-Path $experimentRoot $scheme.Directory) `
            -Root $experimentRoot

        Remove-GeneratedDirectory -Path (Join-Path $schemeRoot 'runtime')
        Remove-GeneratedDirectory -Path (Join-Path $schemeRoot 'Results')

        $temporaryFiles = $standardTemporaryFiles
        if ($scheme.Index -eq 4) {
            $temporaryFiles = $scheme4TemporaryFiles
        }
        foreach ($filename in $temporaryFiles) {
            Remove-GeneratedFile -Path (Join-Path $schemeRoot $filename)
        }

        # SaveTaskFigure.m only creates PNG files. Keep the image folders and
        # any README/.gitkeep/non-image files inside them.
        foreach ($imageFolderName in @('EFboxs_photo','vehiclesbody_photo')) {
            $imageFolder = Assert-PathInsideRoot `
                -Path (Join-Path $schemeRoot $imageFolderName) `
                -Root $experimentRoot
            if (-not (Test-Path -LiteralPath $imageFolder -PathType Container)) {
                New-Item -ItemType Directory -Path $imageFolder | Out-Null
            }
            Get-ChildItem -LiteralPath $imageFolder -File -Filter '*.png' |
                ForEach-Object {
                    Remove-Item -LiteralPath $_.FullName -Force
                    Write-Host "Removed image: $($_.FullName)"
                }
        }
    }

    # Scheme1 generates this matched-Nfe input for Scheme4. Keeping it after
    # -Clear could silently mix an old Scheme1 run with a new Scheme4 run.
    Remove-GeneratedFile -Path (
        Join-Path $experimentRoot 'Scheme1_Full_GeoTime_LSENLP\Nfe_config.txt')

    Write-Host 'Experiment-generated artifacts were cleared.'
}

function Reset-SchemeScratch {
    param([Parameter(Mandatory = $true)]$Scheme)

    $schemeRoot = Assert-PathInsideRoot `
        -Path (Join-Path $experimentRoot $Scheme.Directory) `
        -Root $experimentRoot
    Remove-GeneratedDirectory -Path (Join-Path $schemeRoot 'runtime')
    New-Item -ItemType Directory -Path (Join-Path $schemeRoot 'runtime') |
        Out-Null

    $temporaryFiles = $standardTemporaryFiles
    if ($Scheme.Index -eq 4) {
        $temporaryFiles = $scheme4TemporaryFiles
    }
    foreach ($filename in $temporaryFiles) {
        Remove-GeneratedFile -Path (Join-Path $schemeRoot $filename)
    }
}

function Get-TaskIds {
    if (-not (Test-Path -LiteralPath $taskDirectory -PathType Container)) {
        throw "Task directory for source '$taskSource' does not exist: $taskDirectory"
    }

    $ids = @(
        Get-ChildItem -LiteralPath $taskDirectory -File -Filter '*.mat' |
            ForEach-Object {
                $taskId = 0
                if (-not [int]::TryParse($_.BaseName, [ref]$taskId)) {
                    throw "Task filename must be an integer task_id: $($_.Name)"
                }
                $taskId
            } |
            Sort-Object -Unique
    )
    if ($ids.Count -eq 0) {
        throw "No MAT tasks were found for source '$taskSource' in $taskDirectory"
    }
    return $ids
}

function New-EmptyResultRow {
    param([Parameter(Mandatory = $true)][int]$TaskId)
    $block = "$TaskId,NaN,NaN,NaN"
    return "$block,,$block,,$block,,$block"
}

function Read-ValidatedResultsRows {
    if (-not (Test-Path -LiteralPath $resultsFile -PathType Leaf)) {
        throw "Results file does not exist: $resultsFile"
    }

    $lines = @([System.IO.File]::ReadAllLines($resultsFile))
    if ($lines.Count -lt 1) {
        throw 'results.csv is empty.'
    }

    $headers = @($lines[0] -split ',', -1)
    if ($headers.Count -ne 19) {
        throw "results.csv must have 19 columns; found $($headers.Count)."
    }
    for ($column = 0; $column -lt 19; $column++) {
        if ($headers[$column] -cne $expectedHeaders[$column]) {
            throw "Unexpected results.csv header in column $($column + 1)."
        }
    }

    $rows = @{}
    for ($lineIndex = 1; $lineIndex -lt $lines.Count; $lineIndex++) {
        if ([string]::IsNullOrWhiteSpace($lines[$lineIndex])) {
            continue
        }
        $fields = @($lines[$lineIndex] -split ',', -1)
        if ($fields.Count -ne 19) {
            throw "results.csv row $($lineIndex + 1) must have 19 columns."
        }

        $taskId = 0
        if (-not [int]::TryParse($fields[0], [ref]$taskId)) {
            throw "Invalid task_id in results.csv row $($lineIndex + 1)."
        }
        if ($rows.ContainsKey($taskId)) {
            throw "Duplicate task_id=$taskId in results.csv."
        }
        foreach ($taskColumn in @(0,5,10,15)) {
            $blockTaskId = 0
            if (-not [int]::TryParse($fields[$taskColumn], [ref]$blockTaskId) -or
                    $blockTaskId -ne $taskId) {
                throw "Mismatched task_id block in results.csv row $($lineIndex + 1)."
            }
        }
        $rows[$taskId] = $fields
    }
    return $rows
}

function Initialize-ResultsFile {
    param([Parameter(Mandatory = $true)][int[]]$TaskIds)

    if (-not (Test-Path -LiteralPath $resultsFile -PathType Leaf)) {
        $newLines = [System.Collections.Generic.List[string]]::new()
        $newLines.Add(($expectedHeaders -join ','))
        foreach ($taskId in $TaskIds) {
            $newLines.Add((New-EmptyResultRow -TaskId $taskId))
        }
        [System.IO.File]::WriteAllLines(
            $resultsFile, $newLines, [System.Text.UTF8Encoding]::new($false))
        Write-Host "Created results table: $resultsFile"
        return
    }

    $rows = Read-ValidatedResultsRows
    $changed = $false
    foreach ($taskId in $TaskIds) {
        if (-not $rows.ContainsKey($taskId)) {
            $rows[$taskId] = @((New-EmptyResultRow -TaskId $taskId) -split ',', -1)
            $changed = $true
        }
    }

    if ($changed) {
        $outputLines = [System.Collections.Generic.List[string]]::new()
        $outputLines.Add(($expectedHeaders -join ','))
        foreach ($taskId in @($rows.Keys | Sort-Object)) {
            $outputLines.Add(($rows[$taskId] -join ','))
        }
        [System.IO.File]::WriteAllLines(
            $resultsFile, $outputLines, [System.Text.UTF8Encoding]::new($false))
        Write-Host "Added missing $taskSource tasks to results.csv; existing results were preserved."
    } else {
        Write-Host 'Existing results.csv validated and preserved.'
    }
}

function Get-RecordedSuccess {
    param(
        [Parameter(Mandatory = $true)][int]$SchemeIndex,
        [Parameter(Mandatory = $true)][int]$TaskId
    )

    $rows = Read-ValidatedResultsRows
    if (-not $rows.ContainsKey($TaskId)) {
        return $null
    }
    $successColumn = (($SchemeIndex - 1) * 5) + 1
    $value = ([string]$rows[$TaskId][$successColumn]).Trim()
    if ([string]::IsNullOrWhiteSpace($value) -or
            $value.Equals('NaN', [System.StringComparison]::OrdinalIgnoreCase)) {
        return $null
    }
    if ($value -match '^0(?:\.0+)?$') {
        return 0
    }
    if ($value -match '^1(?:\.0+)?$') {
        return 1
    }
    throw "Invalid success value for scheme$SchemeIndex/task_$('{0:D2}' -f $TaskId): $value"
}

function Set-FailedResult {
    param(
        [Parameter(Mandatory = $true)][int]$SchemeIndex,
        [Parameter(Mandatory = $true)][int]$TaskId
    )

    $lines = [System.Collections.Generic.List[string]]::new()
    $lines.AddRange([string[]][System.IO.File]::ReadAllLines($resultsFile))
    $updated = $false
    for ($lineIndex = 1; $lineIndex -lt $lines.Count; $lineIndex++) {
        $fields = @($lines[$lineIndex] -split ',', -1)
        if ($fields.Count -ne 19) {
            throw "Cannot record failure: malformed results.csv row $($lineIndex + 1)."
        }
        $rowTaskId = 0
        if ([int]::TryParse($fields[0], [ref]$rowTaskId) -and
                $rowTaskId -eq $TaskId) {
            $baseColumn = ($SchemeIndex - 1) * 5
            $fields[$baseColumn + 1] = '0'
            $fields[$baseColumn + 2] = 'NaN'
            $fields[$baseColumn + 3] = 'NaN'
            $lines[$lineIndex] = $fields -join ','
            $updated = $true
            break
        }
    }
    if (-not $updated) {
        throw "Cannot record failure: task_id=$TaskId is missing from results.csv."
    }
    [System.IO.File]::WriteAllLines(
        $resultsFile, $lines, [System.Text.UTF8Encoding]::new($false))
}

function Assert-MatchedNfeExists {
    param([Parameter(Mandatory = $true)][int]$TaskId)

    $configFile = Join-Path $experimentRoot `
        'Scheme1_Full_GeoTime_LSENLP\Nfe_config.txt'
    if (-not (Test-Path -LiteralPath $configFile -PathType Leaf)) {
        throw "Scheme4 task $TaskId requires Scheme1 matched Nfe, but Nfe_config.txt is missing. Run Scheme1 for this task first."
    }

    $matched = $false
    foreach ($line in [System.IO.File]::ReadAllLines($configFile)) {
        if ($line -match '^\s*(\d+)\s+(\d+)(?:\s|$)') {
            if ([int]$Matches[1] -eq $TaskId -and [int]$Matches[2] -ge 2) {
                $matched = $true
                break
            }
        }
    }
    if (-not $matched) {
        throw "Scheme4 task $TaskId has no valid matched Nfe in $configFile. Run Scheme1 for this task first."
    }
}

$schemeSelectionWasSpecified = $Scheme1 -or $Scheme2 -or $Scheme3 -or $Scheme4
if ($Resume -and $RetryFailed) {
    throw '-Resume and -RetryFailed are mutually exclusive.'
}
if ($Clear -and ($schemeSelectionWasSpecified -or $Resume -or
        $RetryFailed -or $InitializeOnly)) {
    throw '-Clear must be used by itself.'
}

if ($Clear) {
    Clear-GeneratedArtifacts
    exit 0
}

$taskIds = @(Get-TaskIds)
Initialize-ResultsFile -TaskIds $taskIds

if ($InitializeOnly) {
    Write-Host "Results table is ready: $resultsFile"
    exit 0
}

$selectedSchemes = @($schemes | Where-Object {
    if (-not $schemeSelectionWasSpecified) {
        return $true
    }
    switch ($_.Index) {
        1 { return $Scheme1 }
        2 { return $Scheme2 }
        3 { return $Scheme3 }
        4 { return $Scheme4 }
    }
})

$matlabCommand = Get-Command matlab -ErrorAction SilentlyContinue
if ($null -eq $matlabCommand) {
    throw 'MATLAB was not found on PATH.'
}

$previousTaskId = $env:EXPERIMENT_TASK_ID
$previousTaskSource = $env:EXPERIMENT_TASK_SOURCE
$env:EXPERIMENT_TASK_SOURCE = $taskSource
$failures = [System.Collections.Generic.List[string]]::new()

try {
    foreach ($scheme in $selectedSchemes) {
        $schemeRoot = Join-Path $experimentRoot $scheme.Directory
        $runScript = Join-Path $schemeRoot $scheme.Script
        if (-not (Test-Path -LiteralPath $runScript -PathType Leaf)) {
            throw "Run script does not exist: $runScript"
        }
        $matlabScript = $runScript.Replace('\','/').Replace("'","''")

        foreach ($taskId in $taskIds) {
            $recordedSuccess = Get-RecordedSuccess `
                -SchemeIndex $scheme.Index -TaskId $taskId

            if ($Resume -and $null -ne $recordedSuccess) {
                Write-Host "Skipping $($scheme.Name)/task_$('{0:D2}' -f $taskId): success=$recordedSuccess is already recorded."
                continue
            }
            if ($RetryFailed -and $recordedSuccess -ne 0) {
                $shownStatus = 'NaN'
                if ($null -ne $recordedSuccess) {
                    $shownStatus = [string]$recordedSuccess
                }
                Write-Host "Skipping $($scheme.Name)/task_$('{0:D2}' -f $taskId): success=$shownStatus is not 0."
                continue
            }

            if ($scheme.Index -eq 4) {
                Assert-MatchedNfeExists -TaskId $taskId
            }

            # Each scheme reuses one runtime directory. Reset only scratch
            # artifacts so a partial run cannot inherit the previous task's
            # opti_flag.txt or optimized variables.
            Reset-SchemeScratch -Scheme $scheme
            $env:EXPERIMENT_TASK_ID = [string]$taskId
            Write-Host "`n===== $($scheme.Name), task $taskId ====="
            & $matlabCommand.Source -batch "run('$matlabScript')"
            if ($LASTEXITCODE -ne 0) {
                $runName = "$($scheme.Name):task_$('{0:D2}' -f $taskId)"
                $failures.Add($runName)
                Set-FailedResult -SchemeIndex $scheme.Index -TaskId $taskId
                Write-Warning "$runName failed before normal evaluation; success=0 and unavailable metrics were recorded as NaN."
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
