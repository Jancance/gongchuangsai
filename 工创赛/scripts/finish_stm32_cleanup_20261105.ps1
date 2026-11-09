$ErrorActionPreference = 'Stop'
$manifestPath = Join-Path $PSScriptRoot '../output/stm32-cleanup-20261105/cleanup-manifest.json'
$plan = Get-Content -LiteralPath $manifestPath -Raw -Encoding utf8 | ConvertFrom-Json
$cleanupRoot = (Resolve-Path -LiteralPath 'D:/新建文件夹4/工创赛/代码/stm32').ProviderPath.TrimEnd('\')
$keepPath = (Resolve-Path -LiteralPath (Join-Path $cleanupRoot '3.stm32f407工程')).ProviderPath
if ($plan.root -ne $cleanupRoot -or $plan.keep -ne $keepPath -or !$plan.archive_verified -or $plan.deleted) {
    throw 'Unexpected cleanup scope or state'
}
if ((Get-FileHash -LiteralPath $plan.archive -Algorithm SHA256).Hash -ne $plan.archive_sha256) {
    throw 'Archive has changed'
}
$targets = @()
foreach ($name in $plan.remove_names) {
    if ([IO.Path]::GetFileName($name) -ne $name) { throw 'Unexpected nested cleanup target' }
    $targetPath = (Resolve-Path -LiteralPath (Join-Path $cleanupRoot $name)).ProviderPath
    if ([IO.Path]::GetDirectoryName($targetPath) -ne $cleanupRoot -or $targetPath -eq $keepPath) {
        throw 'Cleanup target is outside intended root'
    }
    $item = Get-Item -LiteralPath $targetPath -Force
    if ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw 'Refusing reparse point' }
    if ($item.PSIsContainer) {
        $links = @(Get-ChildItem -LiteralPath $targetPath -Recurse -Force |
            Where-Object { $_.Attributes -band [IO.FileAttributes]::ReparsePoint })
        if ($links.Count) { throw 'Refusing directory containing reparse points' }
    }
    $targets += $targetPath
}
$expectedCount = @($plan.archived_files).Count
$actualCount = 0
foreach ($targetPath in $targets) {
    $item = Get-Item -LiteralPath $targetPath -Force
    if ($item.PSIsContainer) {
        $actualCount += @(Get-ChildItem -LiteralPath $targetPath -Recurse -Force -File).Count
    } else { $actualCount++ }
}
if ($actualCount -ne $expectedCount) { throw 'Old project file set changed since archive' }
foreach ($record in $plan.archived_files) {
    $filePath = Join-Path $cleanupRoot $record.path
    if ((Get-FileHash -LiteralPath $filePath -Algorithm SHA256).Hash -ne $record.sha256) {
        throw "File changed since archive: $filePath"
    }
}
foreach ($record in $plan.kept_files_sha256.PSObject.Properties) {
    if ((Get-FileHash -LiteralPath (Join-Path $keepPath $record.Name) -Algorithm SHA256).Hash -ne $record.Value) {
        throw "Current project changed before cleanup: $($record.Name)"
    }
}
foreach ($targetPath in $targets) {
    Remove-Item -LiteralPath $targetPath -Recurse -Force
}
@'
当前唯一保留的STM32工程（2026-11-05整理）

打开：3.stm32f407工程\gongxun-f407.uvprojx
整车目标：Vehicle_Integrated
离线目标：Vehicle_Offline
实际整车主程序：3.stm32f407工程\CHASSIS\USER\main.c
整车参数：3.stm32f407工程\CHASSIS\USER\vehicle_parameters.h

工程内部USER、HARDWARE、CHASSIS等目录有共享依赖，不能按文件夹时间单独删除。
旧工程、旧示例、调试脚本和根目录日志已移出，归档位置：
C:\Users\Tenderness\Documents\ChatGPT\工创赛\output\stm32-cleanup-20261105\old-projects-before-cleanup.zip
逐文件校验记录见同目录cleanup-manifest.json。

本次仅整理文件；未下载固件、未复位主控、未触发运动。
'@ | Set-Content -LiteralPath (Join-Path $cleanupRoot '当前工程入口.txt') -Encoding utf8
foreach ($record in $plan.kept_files_sha256.PSObject.Properties) {
    if ((Get-FileHash -LiteralPath (Join-Path $keepPath $record.Name) -Algorithm SHA256).Hash -ne $record.Value) {
        throw "Kept project mismatch after cleanup: $($record.Name)"
    }
}
$remaining = @(Get-ChildItem -LiteralPath $cleanupRoot -Force | Select-Object -ExpandProperty Name)
if ($remaining.Count -ne 2 -or $remaining -notcontains '3.stm32f407工程' -or $remaining -notcontains '当前工程入口.txt') {
    throw 'Unexpected remaining entries'
}
$plan.deleted = $true
$plan | Add-Member -NotePropertyName kept_project_hashes_verified_after_cleanup -NotePropertyValue $true
$plan | Add-Member -NotePropertyName remaining -NotePropertyValue $remaining
$plan | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath $manifestPath -Encoding utf8
Write-Output "Cleanup complete; retained files: $(@($plan.kept_files_sha256.PSObject.Properties).Count)"
Get-ChildItem -LiteralPath $cleanupRoot -Force | Select-Object Name,Mode
