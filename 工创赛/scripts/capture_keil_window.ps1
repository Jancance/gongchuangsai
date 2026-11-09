param([long]$WindowHandle=1967766,[string]$OutputPath='D:/Codex/工创赛/evidence/p2-arm/2026-11-02/full-cycle/keil-window.png')
Add-Type -AssemblyName System.Drawing
Add-Type @'
using System; using System.Runtime.InteropServices;
public class KeilCapture {
 [StructLayout(LayoutKind.Sequential)] public struct Rect { public int Left,Top,Right,Bottom; }
 [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h,out Rect r);
 [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr h,IntPtr dc,uint flags);
}
'@
$captureRect=New-Object KeilCapture+Rect
[KeilCapture]::GetWindowRect([IntPtr]$WindowHandle,[ref]$captureRect) | Out-Null
$captureWidth=$captureRect.Right-$captureRect.Left
$captureHeight=$captureRect.Bottom-$captureRect.Top
if($captureWidth -le 0 -or $captureHeight -le 0){throw 'Invalid Keil window size'}
$captureBitmap=New-Object System.Drawing.Bitmap($captureWidth,$captureHeight)
$captureGraphics=[System.Drawing.Graphics]::FromImage($captureBitmap)
$captureDc=$captureGraphics.GetHdc()
try{$captureOk=[KeilCapture]::PrintWindow([IntPtr]$WindowHandle,$captureDc,2)}finally{$captureGraphics.ReleaseHdc($captureDc)}
$captureBitmap.Save($OutputPath,[System.Drawing.Imaging.ImageFormat]::Png)
$captureGraphics.Dispose();$captureBitmap.Dispose()
Write-Output "PrintWindow=$captureOk size=${captureWidth}x${captureHeight} output=$OutputPath"
