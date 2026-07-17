param(
    [string]$VolumeLabel = "P4_BUOY",
    [string]$DriveLetter = ""
)

$ErrorActionPreference = "Stop"

function Get-TargetVolume {
    param(
        [string]$Label,
        [string]$Letter
    )
    if ($Letter) {
        $cleanLetter = $Letter.TrimEnd(":")
        return Get-Volume -DriveLetter $cleanLetter -ErrorAction SilentlyContinue
    }
    return Get-Volume | Where-Object {
        $_.FileSystemLabel -eq $Label
    } | Select-Object -First 1
}

function Test-DriveLetterMounted {
    param(
        [string]$Letter
    )
    if (-not $Letter) {
        return $false
    }
    $cleanLetter = $Letter.TrimEnd(":")
    $deviceId = "$cleanLetter`:"
    $logicalDisk = Get-CimInstance Win32_LogicalDisk -Filter "DeviceID='$deviceId'" -ErrorAction SilentlyContinue
    if ($logicalDisk -and $null -ne $logicalDisk.Size) {
        return $true
    }
    # After a successful eject Windows can keep an empty PSDrive shell such as
    # E:\ with Used=0 and Free=$null.  Treat only a logical disk with a real
    # size as still mounted; otherwise the board may already have restored TF.
    return $false
}

$volume = Get-TargetVolume -Label $VolumeLabel -Letter $DriveLetter
if (-not $volume) {
    throw "Volume '$VolumeLabel' was not found."
}

$initialDriveLetter = if ($volume.DriveLetter) { [string]$volume.DriveLetter } else { "" }
$volumePath = $volume.Path
$temporaryDriveLetter = ""

if (-not $initialDriveLetter) {
    $partition = $volume | Get-Partition
    if (-not $partition) {
        throw "Volume '$VolumeLabel' has no drive letter and its partition could not be found."
    }
    $usedLetters = @(Get-Volume | Where-Object { $_.DriveLetter } | ForEach-Object { [string]$_.DriveLetter })
    $candidateLetters = @("Z","Y","X","W","V","U","T","S","R","Q","P","O","N","M","L","K","J","I","H","G","F","E","D")
    $temporaryDriveLetter = $candidateLetters | Where-Object { $usedLetters -notcontains $_ } | Select-Object -First 1
    if (-not $temporaryDriveLetter) {
        throw "Volume '$VolumeLabel' has no drive letter and no temporary drive letter is available."
    }
    Add-PartitionAccessPath `
        -DiskNumber $partition.DiskNumber `
        -PartitionNumber $partition.PartitionNumber `
        -AccessPath "$temporaryDriveLetter`:" | Out-Null
    Start-Sleep -Seconds 1
    $volume = Get-Volume -DriveLetter $temporaryDriveLetter -ErrorAction Stop
    $initialDriveLetter = $temporaryDriveLetter
    Write-Host "Assigned temporary drive letter $temporaryDriveLetter`: to '$VolumeLabel' for safe eject."
}

if ($initialDriveLetter) {
    $drivePath = "$initialDriveLetter`:\"
    $shell = New-Object -ComObject Shell.Application
    try {
        $computer = $shell.Namespace(17)
        $drive = $computer.ParseName($drivePath)
        if (-not $drive) {
            throw "Drive '$drivePath' was not found by Windows Shell."
        }
        $drive.InvokeVerb("Eject")
    }
    finally {
        [void][Runtime.InteropServices.Marshal]::FinalReleaseComObject($shell)
    }
}

$waitDriveLetter = if ($DriveLetter) { $DriveLetter.TrimEnd(":") } else { $initialDriveLetter }
$stillMounted = $true
for ($i = 0; $i -lt 20; $i++) {
    Start-Sleep -Milliseconds 700
    if ($waitDriveLetter) {
        $stillMounted = Test-DriveLetterMounted -Letter $waitDriveLetter
    }
    else {
        $stillMounted = [bool](Get-TargetVolume -Label $VolumeLabel -Letter "")
    }
    if (-not $stillMounted) {
        break
    }
}
if ($stillMounted) {
    $displayLetter = if ($waitDriveLetter) { "$waitDriveLetter`:" } else { $volumePath }
    throw "Windows did not eject '$VolumeLabel'. Close Explorer/files using $displayLetter and retry from the taskbar or this script. The script will not use mountvol /p because that disables the drive letter on later inserts."
}

if ($initialDriveLetter) {
    Write-Host "Safely dismounted $initialDriveLetter`:\ ($VolumeLabel)."
} else {
    Write-Host "Safely dismounted $volumePath ($VolumeLabel)."
}
Write-Host "The board should restore TF automatically after eject/detach. Watch /api/status until usb_storage_owner=app; use the Web restore button if it stays in USB export."
