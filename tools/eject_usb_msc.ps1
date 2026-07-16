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
        return Get-Volume -DriveLetter $cleanLetter -ErrorAction Stop
    }
    return Get-Volume | Where-Object {
        $_.FileSystemLabel -eq $Label
    } | Select-Object -First 1
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
        Start-Sleep -Seconds 2
    }
    finally {
        [void][Runtime.InteropServices.Marshal]::FinalReleaseComObject($shell)
    }
}

$stillMounted = Get-TargetVolume -Label $VolumeLabel -Letter $DriveLetter
if ($stillMounted) {
    $target = "$initialDriveLetter`:"
    Write-Warning "Windows still has '$VolumeLabel' mounted; taking the volume offline with mountvol ($target)."
    & mountvol.exe $target /p
    if ($LASTEXITCODE -ne 0) {
        $cimVolume = Get-CimInstance Win32_Volume | Where-Object {
            $_.Label -eq $VolumeLabel -or $_.Name -eq $volumePath
        } | Select-Object -First 1
        if (-not $cimVolume) {
            throw "mountvol failed and Win32_Volume could not find '$VolumeLabel'."
        }
        $result = Invoke-CimMethod -InputObject $cimVolume -MethodName Dismount -Arguments @{
            Force = $false
            Permanent = $false
        }
        if ($result.ReturnValue -ne 0) {
            throw "Could not safely dismount '$VolumeLabel' (mountvol exit $LASTEXITCODE, Win32_Volume return $($result.ReturnValue))."
        }
    }
    Start-Sleep -Seconds 2
}

$stillMounted = Get-TargetVolume -Label $VolumeLabel -Letter $DriveLetter
if ($stillMounted) {
    throw "'$VolumeLabel' is still mounted. Close Explorer/files using it, then retry."
}

if ($initialDriveLetter) {
    Write-Host "Safely dismounted $initialDriveLetter`:\ ($VolumeLabel)."
} else {
    Write-Host "Safely dismounted $volumePath ($VolumeLabel)."
}
Write-Host "The board firmware should restore TF automatically after eject/detach; use the Web restore button only if status stays in USB export."
