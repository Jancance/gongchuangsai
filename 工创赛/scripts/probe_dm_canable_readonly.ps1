<#
通过CANable SLCAN只读探测达妙参数。默认仅自检；显式-Scan才打开串口。
只发送标准帧0x7FF、操作码0x33，不使能、不保存或修改参数、不发运动命令。
来源：dmBots/motor-sdk的DM_CAN.py及厂家寄存器表。旧电机固件支持情况待实测。
#>
[CmdletBinding()]
param(
    [ValidatePattern('^COM[0-9]+$')][string]$Port = 'COM16',
    [ValidateRange(0, 15)][int]$FirstId = 0,
    [ValidateRange(0, 15)][int]$LastId = 15,
    [switch]$Scan,
    [string]$OutputDirectory = ''
)
$ErrorActionPreference = 'Stop'

function New-ReadFrame([int]$MotorId, [int]$Register) {
    if ($MotorId -lt 0 -or $MotorId -gt 15 -or $Register -notin @(7, 8, 10, 14)) {
        throw '仅允许候选ID0～15和已列出的只读寄存器。'
    }
    # t=经典CAN标准数据帧，7FF=参数访问ID，8=八字节。
    return ('t7FF8{0:X2}0033{1:X2}00000000' -f $MotorId, $Register)
}

function ConvertFrom-SlcanFrame([string]$Line) {
    $match = [regex]::Match($Line, '^t([0-9A-Fa-f]{3})([0-8])([0-9A-Fa-f]*?)$')
    if (-not $match.Success) { return $null }
    $dlc = [int]$match.Groups[2].Value
    $payload = $match.Groups[3].Value
    # 允许四位接收时间戳；不能把不完整/超长帧补成有效回复。
    if ($payload.Length -eq ($dlc * 2 + 4)) { $payload = $payload.Substring(0, $dlc * 2) }
    if ($payload.Length -ne $dlc * 2) { return $null }
    $bytes = @()
    for ($i = 0; $i -lt $dlc; $i++) {
        $bytes += [Convert]::ToByte($payload.Substring($i * 2, 2), 16)
    }
    return [pscustomobject]@{
        CanId = [Convert]::ToInt32($match.Groups[1].Value, 16)
        Dlc = $dlc
        Data = [byte[]]$bytes
        Raw = $Line
    }
}

if ($FirstId -gt $LastId) { throw 'FirstId不能大于LastId。' }
if (-not $Scan) {
    if ((New-ReadFrame 1 8) -cne 't7FF80100330800000000') { throw '查询帧编码自检失败。' }
    $sample = ConvertFrom-SlcanFrame 't00080100330801000000'
    if ($null -eq $sample -or $sample.CanId -ne 0 -or $sample.Data[4] -ne 1) {
        throw '标准帧/零反馈ID解析自检失败。'
    }
    if ($null -ne (ConvertFrom-SlcanFrame 't000801003308')) { throw '短帧未被拒绝。' }
    if ($null -ne (ConvertFrom-SlcanFrame 'T0000000080100330801000000')) {
        throw '扩展帧不应被当作标准帧。'
    }
    [pscustomobject]@{ Mode = 'self-check'; Passed = $true; Port = $Port; CanBitrate = 1000000; HardwareOpened = $false } | ConvertTo-Json
    return
}

if (-not $OutputDirectory) {
    $OutputDirectory = Join-Path (Split-Path $PSScriptRoot -Parent) ('evidence\p2-arm\2026-10-29\canable-readonly\scan-' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
}
$null = New-Item -ItemType Directory -Path $OutputDirectory -Force
$script:Frames = [System.Collections.Generic.List[object]]::new()
$script:RawLines = [System.Collections.Generic.List[string]]::new()
$script:Pending = ''
$script:Requests = [System.Collections.Generic.List[object]]::new()
$script:Device = [System.IO.Ports.SerialPort]::new($Port, 115200, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$script:Device.Handshake = [System.IO.Ports.Handshake]::None
$script:Device.ReadTimeout = 100
$script:Device.WriteTimeout = 300

function Receive-Window([int]$Milliseconds) {
    $watch = [System.Diagnostics.Stopwatch]::StartNew()
    while ($watch.ElapsedMilliseconds -lt $Milliseconds) {
        $chunk = $script:Device.ReadExisting()
        if ($chunk) {
            $script:Pending += $chunk
            if ($script:Pending.Length -gt 8192) { throw '串口半帧超出上限，停止本轮。' }
            while ($script:Pending.Contains("`r")) {
                $index = $script:Pending.IndexOf("`r")
                $line = $script:Pending.Substring(0, $index).Trim("`n")
                $script:Pending = $script:Pending.Substring($index + 1)
                if ($script:RawLines.Count -ge 2048) { throw '接收上限已到，停止本轮。' }
                $script:RawLines.Add($line)
                $frame = ConvertFrom-SlcanFrame $line
                if ($null -ne $frame) { $script:Frames.Add($frame) }
            }
        }
        Start-Sleep -Milliseconds 5
    }
}

function Invoke-Read([int]$Id, [int]$Register) {
    $start = $script:Frames.Count
    $frame = New-ReadFrame $Id $Register
    $script:Requests.Add([pscustomobject]@{ MotorId = $Id; Register = $Register; Slcan = $frame })
    $script:Device.Write($frame + "`r")
    Receive-Window 120
    $responses = @()
    for ($i = $start; $i -lt $script:Frames.Count; $i++) {
        $f = $script:Frames[$i]
        # 排除可能的发送回显；Master ID=7FF的特殊配置需另核。
        if ($f.Dlc -eq 8 -and $f.CanId -ne 0x7FF -and
            ($f.Data[0] -bor ($f.Data[1] -shl 8)) -eq $Id -and
            $f.Data[2] -eq 0x33 -and $f.Data[3] -eq $Register) {
            $responses += [pscustomobject]@{
                ReplyCanId = $f.CanId
                Value = [BitConverter]::ToUInt32($f.Data, 4)
                Raw = $f.Raw
            }
        }
    }
    return $responses
}

$found = [System.Collections.Generic.List[object]]::new()
$failure = $null
try {
    # 串口被cangaroo占用时Open会失败，不抢占其他进程或重装驱动。
    $script:Device.Open()
    $script:Device.DiscardInBuffer()
    foreach ($command in @('C', 'S8', 'A0', 'O', 'V')) {
        $script:Device.Write($command + "`r")
        Receive-Window 40
    }
    # S8=1Mbps；A0关闭自动重发；每个候选只读一次，帧数量有界。
    for ($id = $FirstId; $id -le $LastId; $id++) {
        $esc = @(Invoke-Read $id 8)
        $valid = @($esc | Where-Object { $_.Value -eq $id })
        if ($valid.Count -gt 0) {
            $master = @(Invoke-Read $id 7)
            $mode = @(Invoke-Read $id 10)
            $version = @(Invoke-Read $id 14)
            $found.Add([pscustomobject]@{
                CandidateId = $id
                EscIdReplies = $valid
                MasterIdReplies = $master
                ControlModeReplies = $mode
                SoftwareVersionReplies = $version
            })
        }
    }
} catch {
    $failure = $_.Exception.Message
} finally {
    if ($script:Device.IsOpen) {
        try { $script:Device.Write("C`r") } catch {}
        $script:Device.Close()
    }
    $script:Device.Dispose()
    $report = [pscustomobject]@{
        Timestamp = (Get-Date -Format 'o'); Port = $Port; CanBitrate = 1000000
        FirstId = $FirstId; LastId = $LastId; Failure = $failure
        RequestedFrameCount = $script:Requests.Count
        ReceivedFrameCount = $script:Frames.Count
        Found = $found.ToArray(); Requests = $script:Requests.ToArray()
        Frames = $script:Frames.ToArray(); RawLines = $script:RawLines.ToArray()
        EnableOrMotionCommandsSent = $false; ParameterWritesSent = $false
        Note = '发送请求不等于总线ACK；无回复不能单独判定电机不存在。旧固件参数访问、接线及终端均待实测。'
    }
    $report | ConvertTo-Json -Depth 10 | Set-Content -LiteralPath (Join-Path $OutputDirectory 'report.json') -Encoding UTF8
}
[pscustomobject]@{ Failure = $failure; Requests = $script:Requests.Count; Received = $script:Frames.Count; FoundCount = $found.Count; Report = (Join-Path $OutputDirectory 'report.json') } | ConvertTo-Json
if ($failure) { throw $failure }
