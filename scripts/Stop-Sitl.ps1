$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$sitlRoot = Join-Path $projectRoot '.tools/sitl'
$statePath = Join-Path $sitlRoot 'processes.json'
if (!(Test-Path -LiteralPath $statePath)) { Write-Output 'No recorded SITL processes.'; return }
$executable = (Resolve-Path (Join-Path $sitlRoot 'bin/ArduCopter.exe')).Path
foreach ($entry in @(Get-Content -LiteralPath $statePath -Raw | ConvertFrom-Json)) {
    $process = Get-Process -Id $entry.ProcessId -ErrorAction SilentlyContinue
    if ($process -and $process.Path -eq $executable -and
        $process.StartTime.ToUniversalTime().ToString('o') -eq $entry.StartedUtc) {
        Stop-Process -Id $process.Id
        Write-Output "Stopped simulated vehicle $($entry.SystemId) (PID $($process.Id))."
    }
}
