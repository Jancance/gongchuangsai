param(
    [string]$Output = (Join-Path $PSScriptRoot '../tmp/straight-capture-20261120/builds'),
    [string]$Keil = 'D:\Keil_v5\UV4\UV4.exe'
)

$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$source = Join-Path $root '3.stm32f407工程/gongxun-f407.uvprojx'
$definition = 'APP_STRAIGHT_CAPTURE_ENABLE=1,APP_BASELINE40_ENABLE=1,VEHICLE_INTEGRATED_ENABLE=0,VEHICLE_STAGE1_ENABLE=0,APP_DISTANCE_SCURVE_ENABLE=1'
$originalHash = (Get-FileHash -LiteralPath $source).Hash
$xml = [IO.File]::ReadAllText($source)
$xml = $xml.Replace('.\', ([IO.Path]::GetDirectoryName($source) + '\'))
$xml = [regex]::Replace($xml, 'APP_(?:CONTINUOUS_MAP|MAP_NAV_RESIDUAL|ZONE2_NAV|ZONE2_NAV_MOTION|ZONE2_POSE_TRACE)_ENABLE=1,', '')
$xml = [regex]::Replace($xml, '(?<=<Define>)(?=ICM45686_SAMPLE_ONLY=0,STM32F40_41xxx)', ($definition + ','))
$xml = [regex]::Replace($xml, '(?<=<(?:OutputDirectory|ListingPath)>)[^<]+', 'build\')

foreach ($target in 'Vehicle_Integrated', 'Vehicle_Offline') {
    $folder = New-Item -ItemType Directory -Path (Join-Path $Output $target)
    $project = Join-Path $folder.FullName 'gongxun-f407.uvprojx'
    $log = Join-Path $folder.FullName 'rebuild.log'
    [IO.File]::WriteAllText($project, $xml)
    $process = Start-Process -FilePath $Keil -ArgumentList ('-r "{0}" -t {1} -o "{2}"' -f $project,$target,$log) -WindowStyle Hidden -Wait -PassThru
    if ($process.ExitCode -ne 0 -or !(Select-String -LiteralPath $log -Pattern '0 Error\(s\), 0 Warning\(s\)')) {
        Get-Content -LiteralPath $log -Tail 20
        throw "Keil rebuild failed for $target"
    }
    Get-FileHash -LiteralPath (Join-Path $folder.FullName "build/$target.axf") | Format-List Hash,Path
}
if ((Get-FileHash -LiteralPath $source).Hash -ne $originalHash) { throw 'Source project changed during build' }
