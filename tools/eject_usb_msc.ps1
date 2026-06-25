param(
    [string]$VolumeLabel = "P4_BUOY",
    [string]$DriveLetter = ""
)

$ErrorActionPreference = "Stop"

if (-not $DriveLetter) {
    $volume = Get-Volume | Where-Object {
        $_.FileSystemLabel -eq $VolumeLabel -and $_.DriveLetter
    } | Select-Object -First 1
    if (-not $volume) {
        throw "Volume '$VolumeLabel' was not found."
    }
    $DriveLetter = [string]$volume.DriveLetter
}

$DriveLetter = $DriveLetter.TrimEnd(":")
$drivePath = "$DriveLetter`:\"

$shell = New-Object -ComObject Shell.Application
$computer = $shell.Namespace(17)
$drive = $computer.ParseName($drivePath)
if (-not $drive) {
    throw "Drive '$drivePath' was not found by Windows Shell."
}

$drive.InvokeVerb("Eject")
Start-Sleep -Seconds 2

$stillMounted = Get-Volume -DriveLetter $DriveLetter -ErrorAction SilentlyContinue
if ($stillMounted) {
    Write-Warning "Eject was requested, but $drivePath still appears mounted. Use Windows safe eject before resetting the board."
    exit 2
}

Write-Host "Ejected $drivePath. Waited 2 seconds; it is now safe to reset or reflash the board."
