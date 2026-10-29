param([ValidateRange(1, 65535)][int]$Port = 14550)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$sitlRoot = Join-Path $projectRoot '.tools/sitl'
$executable = (Resolve-Path (Join-Path $sitlRoot 'bin/ArduCopter.exe')).Path
$statePath = Join-Path $sitlRoot 'processes.json'
if (Test-Path -LiteralPath $statePath) {
    foreach ($entry in @(Get-Content -LiteralPath $statePath -Raw | ConvertFrom-Json)) {
        $existing = Get-Process -Id $entry.ProcessId -ErrorAction SilentlyContinue
        if ($existing -and $existing.Path -eq $executable) {
            throw 'The test swarm is already running. Use scripts/Stop-Sitl.ps1 before restarting.'
        }
    }
}
$instances = @()
for ($instance = 0; $instance -lt 3; $instance++) {
    $systemId = $instance + 1
    $workingDirectory = Join-Path $sitlRoot "vehicle-$systemId"
    New-Item -ItemType Directory -Force $workingDirectory | Out-Null
    @'
FRAME_CLASS 1
FRAME_TYPE 1
SERIAL0_PROTOCOL 2
SR0_EXT_STAT 2
SR0_POSITION 5
SR0_RAW_SENS 2
SR0_EXTRA1 5
SR0_EXTRA2 2
SR0_EXTRA3 2
'@ | Set-Content -LiteralPath (Join-Path $workingDirectory 'defaults.parm') -Encoding ascii
    # Separate homes about 20 m apart, without issuing any arm or flight commands.
    $longitude = (-86.1581 + $instance * 0.00024).ToString('F6', [Globalization.CultureInfo]::InvariantCulture)
    $arguments = @('--model', 'quad', '--instance', "$instance", '--sysid', "$systemId",
        '--home', "39.768400,$longitude,220,0", '--speedup', '1',
        '--defaults', 'defaults.parm', '--serial0', "udpclient:127.0.0.1:$Port")
    $process = Start-Process -FilePath $executable -ArgumentList $arguments -WorkingDirectory $workingDirectory `
        -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $workingDirectory 'stdout.log') `
        -RedirectStandardError (Join-Path $workingDirectory 'stderr.log')
    $instances += [pscustomobject]@{
        SystemId = $systemId
        ProcessId = $process.Id
        StartedUtc = $process.StartTime.ToUniversalTime().ToString('o')
        Destination = "127.0.0.1:$Port"
        WorkingDirectory = $workingDirectory
    }
    $instances | ConvertTo-Json -Depth 3 | Set-Content -LiteralPath $statePath
}
$instances | Format-Table SystemId, ProcessId, Destination
