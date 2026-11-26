param([string]$RunId = ('left_' + (Get-Date -Format 'yyyyMMdd_HHmmss_fff')), [switch]$WaitForReady)
$ErrorActionPreference = 'Stop'
if ($RunId -notmatch '^[a-zA-Z0-9_-]+$') { throw 'Invalid RunId' }
$rawPath = Join-Path $PSScriptRoot ($RunId + '.bin')
$textPath = Join-Path $PSScriptRoot ($RunId + '.txt')
if ((Test-Path -LiteralPath $rawPath) -or (Test-Path -LiteralPath $textPath)) { throw 'RunId already exists; choose a new RunId' }
$rawOutput = $null
$textOutput = $null
$port = [System.IO.Ports.SerialPort]::new('COM17', 115200, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$port.Handshake = [System.IO.Ports.Handshake]::None
$port.DtrEnable = $false
$port.RtsEnable = $false
$port.ReadTimeout = 100
$raw = [System.Collections.Generic.List[byte]]::new()
$clock = [Diagnostics.Stopwatch]::StartNew()
$started = $false
$lineTail = ''
$readySeen = -not $WaitForReady
try {
    $rawOutput = [IO.File]::Open($rawPath, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write, [IO.FileShare]::Read)
    $textOutput = [IO.File]::Open($textPath, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write, [IO.FileShare]::Read)
    $port.Open()
    $started = $true
    Write-Output 'COM17 opened, passive receive only, 115200 8N1, DTR=0 RTS=0'
    while ($clock.Elapsed.TotalSeconds -lt 900) {
        $available = $port.BytesToRead
        if ($available -gt 0) {
            $buffer = [byte[]]::new([Math]::Min($available, 4096))
            try {
                $count = $port.Read($buffer, 0, $buffer.Length)
                if ($count -gt 0) {
                    for ($i = 0; $i -lt $count; $i++) { $raw.Add($buffer[$i]) }
                    $rawOutput.Write($buffer, 0, $count)
                    $rawOutput.Flush()
                    $text = [Text.Encoding]::ASCII.GetString($buffer, 0, $count)
                    $textBytes = [Text.Encoding]::ASCII.GetBytes($text)
                    $textOutput.Write($textBytes, 0, $textBytes.Length)
                    $textOutput.Flush()
                    Write-Output $text.TrimEnd("`r", "`n")
                    $lineTail += $text
                    $completeEnd = $false
                    while (($newline = $lineTail.IndexOf("`n")) -ge 0) {
                        $line = $lineTail.Substring(0, $newline).TrimEnd("`r")
                        $lineTail = $lineTail.Substring($newline + 1)
                        if ($line -eq 'F407 FreeRTOS READY') { $readySeen = $true }
                        if ($readySeen -and $line -match '^T160_END,\d+,\d+,\d+,\d+,\d+$') { $completeEnd = $true }
                    }
                    if ($completeEnd) {
                        Write-Output 'Complete T160_END received; ending passive capture'
                        break
                    }
                }
            } catch [System.TimeoutException] {}
        } else { Start-Sleep -Milliseconds 20 }
    }
} catch {
    Write-Output ('COM17 monitor error, opened=' + $started + ', bytes=' + $raw.Count + ', detail=' + $_.Exception.Message)
} finally {
    if ($port.IsOpen) { $port.Close() }
    if ($rawOutput) { $rawOutput.Dispose() }
    if ($textOutput) { $textOutput.Dispose() }
    Write-Output ('COM17 monitor closed, bytes=' + $raw.Count)
}
