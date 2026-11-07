[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$Port,
    [int]$Baud = 460800
)

$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$idfPython = 'C:\Espressif\tools\python\v6.0.3\venv\Scripts\python.exe'
if (Test-Path $idfPython) { $python = $idfPython } else { $python = 'python' }
$bin = Join-Path $here 'leap_ros_max_camsense.bin'

# App partition only: keep NVS WiFi / micro-ROS agent settings.
Write-Host "Flashing app-only image: $bin"
& $python -m esptool --chip esp32s3 -p $Port -b $Baud `
    --before default-reset --after hard-reset `
    write-flash -z --flash-mode dio --flash-freq 80m --flash-size 4MB `
    0x10000 $bin

$code = $LASTEXITCODE
if ($code -eq 0) {
    Write-Host 'Flash complete.' -ForegroundColor Green
} else {
    Write-Host "Flash failed (exit code $code)." -ForegroundColor Red
}
exit $code
