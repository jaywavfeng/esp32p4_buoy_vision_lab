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
try {
    $computer = $shell.Namespace(17)
    $drive = $computer.ParseName($drivePath)
    if (-not $drive) {
        throw "Drive '$drivePath' was not found by Windows Shell."
    }
    $drive.InvokeVerb("Eject")
    Start-Sleep -Seconds 2
}
finally {
    [void][Runtime.InteropServices.Marshal]::FinalReleaseComObject($shell)
}

$stillMounted = Get-Volume -DriveLetter $DriveLetter -ErrorAction SilentlyContinue
if ($stillMounted) {
    Write-Warning "Windows Shell kept $drivePath mounted; flushing and taking the volume offline with mountvol."
    & mountvol.exe "$DriveLetter`:" /p
    if ($LASTEXITCODE -ne 0) {
        throw "mountvol could not safely dismount $drivePath (exit code $LASTEXITCODE)."
    }
    Start-Sleep -Seconds 2
    $stillMounted = Get-Volume -DriveLetter $DriveLetter -ErrorAction SilentlyContinue
    if ($stillMounted) {
        throw "$drivePath is still mounted. Close every file and Explorer window using it, then retry."
    }
}

Write-Host "Safely dismounted $drivePath. Physically unplug the USB data cable before restoring TF from Web."
