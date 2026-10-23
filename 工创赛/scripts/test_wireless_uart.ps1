param(
    [string]$Port = 'COM17',
    [ValidateRange(1,30)][int]$WaitSeconds = 5,
    [switch]$ListenOnly,
    [switch]$SelfTest,
    [string]$OutputDirectory = ''
)
$ErrorActionPreference = 'Stop'
function Test-LinkHeartbeat([string]$Line) {
    return $Line -cmatch '^LINK,HB,[0-9]{1,10},[0-9]{1,10},VM0$'
}
function Test-LinkReply([string]$Line, [string]$Expected) {
    return $Line -ceq $Expected
}
function New-LinkQueries([string]$nonce, [int]$value) {
    $queries = @(
            @(('PING,'+$nonce),('PONG,'+$nonce)),
            @(('ECHO,'+$nonce),('ECHO,'+$nonce)),
            @(('SET,'+$value),('VALUE,'+$value)),
            @('GET',('VALUE,'+$value)),
            @('INFO','INFO,UART1,115200,8N1,VM0'),
            @('SET,1001','ERR,RANGE'),
            @('GET',('VALUE,'+$value))
        )
    return ,$queries
}
if ($SelfTest) {
    if (!(Test-LinkHeartbeat 'LINK,HB,1,1000,VM0')) { throw 'Heartbeat acceptance failed' }
    foreach ($bad in @('LINK,HB,1,1000,VM1','PING,abc','PONG,abc','LINK,HB,x,1000,VM0')) {
        if (Test-LinkHeartbeat $bad) { throw ('Bad heartbeat accepted: ' + $bad) }
    }
    if (!(Test-LinkReply 'PONG,abc' 'PONG,abc')) { throw 'Reply match failed' }
    foreach ($bad in @('PING,abc','PONG,old','PONG,ABC')) {
        if (Test-LinkReply $bad 'PONG,abc') { throw 'Wrong or locally echoed reply accepted' }
    }
    $cases = New-LinkQueries 'abc' 321
    if ($cases.Count -ne 7) { throw 'Wrong query count' }
    foreach ($case in $cases) { if ($case.Count -ne 2) { throw 'Wrong query shape' } }
    if ($cases[0][0] -cne 'PING,abc' -or $cases[0][1] -cne 'PONG,abc') { throw 'Nonce query malformed' }
    if ($cases[2][0] -cne 'SET,321' -or $cases[3][1] -cne 'VALUE,321') { throw 'Value query malformed' }
    Write-Output 'PASS: 19 host protocol/command-shape checks; no COM port opened, no bytes sent.' 
    exit 0
}
if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $OutputDirectory = Join-Path $PSScriptRoot '../tmp/uart_link_captures'
}
[IO.Directory]::CreateDirectory($OutputDirectory) | Out-Null
$id = (Get-Date -Format 'yyyyMMdd_HHmmss') + '_' + [Guid]::NewGuid().ToString('N').Substring(0,6)
$logPath = Join-Path $OutputDirectory ($id + '.jsonl')
$summaryPath = Join-Path $OutputDirectory ($id + '.summary.json')
$script:linkBuffer = ''
$script:linkLog = New-Object IO.StreamWriter $logPath,$false,(New-Object Text.UTF8Encoding($false))
$script:linkLog.AutoFlush = $true
function Write-LinkLog([string]$Direction, [string]$Text) {
    $entry = [ordered]@{utc=[DateTime]::UtcNow.ToString('o');direction=$Direction;text=$Text}
    $script:linkLog.WriteLine(($entry | ConvertTo-Json -Compress))
}
function Read-LinkLine($Serial, [int]$TimeoutMs) {
    $timer = [Diagnostics.Stopwatch]::StartNew()
    while ($timer.ElapsedMilliseconds -lt $TimeoutMs) {
        $newline = $script:linkBuffer.IndexOf("`n")
        if ($newline -ge 0) {
            $line = $script:linkBuffer.Substring(0,$newline).TrimEnd([char]13)
            $script:linkBuffer = $script:linkBuffer.Substring($newline+1)
            Write-LinkLog 'RX' $line
            return $line
        }
        $script:linkBuffer += $Serial.ReadExisting()
        if ($script:linkBuffer.Length -gt 4096) { throw 'RX line exceeds bound; check baud/wiring' }
        Start-Sleep -Milliseconds 10
    }
    return $null
}
function Invoke-LinkQuery($Serial, [string]$Command, [string]$Expected) {
    # Exactly one transmit per query. No retries, no movement-related command.
    Write-LinkLog 'TX' $Command
    $Serial.Write($Command + "`n")
    $timer = [Diagnostics.Stopwatch]::StartNew()
    while ($timer.ElapsedMilliseconds -lt 2000) {
        $line = Read-LinkLine $Serial 200
        if ($null -ne $line -and (Test-LinkReply $line $Expected)) { return $true }
    }
    return $false
}
$serial = New-Object IO.Ports.SerialPort $Port,115200,([IO.Ports.Parity]::None),8,([IO.Ports.StopBits]::One)
$serial.Encoding = [Text.Encoding]::ASCII
$serial.Handshake = [IO.Ports.Handshake]::None
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$serial.WriteTimeout = 500
$report = [ordered]@{port=$Port;baud=115200;opened=$false;heartbeat=$false;sent=0;matched=0;status='not_started';tests=@();error=$null;log=[IO.Path]::GetFullPath($logPath)}
$exitCode = 2
try {
    $serial.Open()
    $report.opened = $true
    $timer = [Diagnostics.Stopwatch]::StartNew()
    while ($timer.Elapsed.TotalSeconds -lt $WaitSeconds) {
        $line = Read-LinkLine $serial 200
        if ($null -ne $line -and (Test-LinkHeartbeat $line)) { $report.heartbeat=$true;break }
    }
    if (!$report.heartbeat) {
        $report.status = 'no_test_heartbeat_no_bytes_sent'
    } elseif ($ListenOnly) {
        $report.status = 'receive_only_ok'
        $exitCode = 0
    } else {
        $nonce = [Guid]::NewGuid().ToString('N').Substring(0,12)
        $value = Get-Random -Minimum 123 -Maximum 999
        $queries = New-LinkQueries $nonce $value
        foreach ($query in $queries) {
            $report.sent++
            $ok = Invoke-LinkQuery $serial $query[0] $query[1]
            $report.tests += [ordered]@{command=$query[0];expected=$query[1];matched=$ok}
            if ($ok) { $report.matched++ } else { break }
            Start-Sleep -Milliseconds 30
        }
        if ($report.matched -eq $queries.Count) {
            $report.status = 'bidirectional_pass'
            $exitCode = 0
        } else { $report.status = 'reply_missing_or_mismatch' }
    }
} catch {
    $report.status = 'port_or_io_error'
    $report.error = $_.Exception.Message
} finally {
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
    $script:linkLog.Dispose()
}
$text = $report | ConvertTo-Json -Depth 6
[IO.File]::WriteAllText($summaryPath,$text,(New-Object Text.UTF8Encoding($false)))
Write-Output $text
exit $exitCode
