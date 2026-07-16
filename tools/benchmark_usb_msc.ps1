param(
    [string]$VolumeLabel = "P4_BUOY",
    [int]$TestSizeMiB = 64,
    [double]$MinReadMiBs = 0.5,
    [double]$MinWriteMiBs = 0.5
)

$ErrorActionPreference = "Stop"

if ($TestSizeMiB -lt 50) {
    throw "TestSizeMiB must be at least 50 MiB."
}

$volume = Get-Volume | Where-Object {
    $_.FileSystemLabel -eq $VolumeLabel -and $_.DriveLetter
} | Select-Object -First 1
if (-not $volume) {
    Write-Host "Known volumes:"
    Get-Volume | Select-Object DriveLetter,FileSystemLabel,FileSystem,DriveType,HealthStatus,SizeRemaining,Size |
        Format-Table -AutoSize
    Write-Host "Present USB-like devices:"
    Get-PnpDevice -PresentOnly | Where-Object {
        $_.InstanceId -like "USB*" -or
        $_.FriendlyName -match "ESP|TinyUSB|P4|Buoy|Mass Storage|MSC|Serial|JTAG|CDC|USB"
    } | Select-Object Class,FriendlyName,Status,InstanceId | Format-Table -AutoSize
    throw "Writable USB volume '$VolumeLabel' was not found. Connect the PC to the board's USB 2.0 Type-C DEVICE port with a data-capable cable; do not use the Type-A HOST port or an A-to-A cable."
}

$usbRoot = "{0}:\" -f $volume.DriveLetter
$localRoot = Join-Path $PSScriptRoot "..\artifacts\usb_msc_benchmark"
$localRoot = [System.IO.Path]::GetFullPath($localRoot)
New-Item -ItemType Directory -Force -Path $localRoot | Out-Null

$fileName = "p4_buoy_usb_benchmark.bin"
$sourcePath = Join-Path $localRoot $fileName
$readbackDir = Join-Path $localRoot "readback"
$readbackPath = Join-Path $readbackDir $fileName
$usbPath = Join-Path $usbRoot $fileName
New-Item -ItemType Directory -Force -Path $readbackDir | Out-Null

$targetBytes = [int64]$TestSizeMiB * 1MB
if (-not (Test-Path -LiteralPath $sourcePath) -or
    (Get-Item -LiteralPath $sourcePath).Length -ne $targetBytes) {
    $buffer = New-Object byte[] 1MB
    for ($i = 0; $i -lt $buffer.Length; $i++) {
        $buffer[$i] = [byte](($i * 37 + 0x5a) -band 0xff)
    }
    $stream = [System.IO.FileStream]::new(
        $sourcePath,
        [System.IO.FileMode]::Create,
        [System.IO.FileAccess]::Write,
        [System.IO.FileShare]::None,
        1MB,
        [System.IO.FileOptions]::WriteThrough -bor [System.IO.FileOptions]::SequentialScan
    )
    try {
        for ($written = [int64]0; $written -lt $targetBytes; $written += $buffer.Length) {
            $stream.Write($buffer, 0, $buffer.Length)
        }
        $stream.Flush($true)
    }
    finally {
        $stream.Dispose()
    }
}

function Invoke-UnbufferedCopy {
    param(
        [Parameter(Mandatory)] [string]$SourceDirectory,
        [Parameter(Mandatory)] [string]$DestinationDirectory,
        [Parameter(Mandatory)] [string]$Name
    )

    $watch = [System.Diagnostics.Stopwatch]::StartNew()
    & robocopy.exe $SourceDirectory $DestinationDirectory $Name /J /R:0 /W:0 /NFL /NDL /NJH /NJS /NP | Out-Null
    $exitCode = $LASTEXITCODE
    $watch.Stop()
    if ($exitCode -gt 7) {
        throw "robocopy failed with exit code $exitCode"
    }
    return $watch.Elapsed.TotalSeconds
}

Remove-Item -LiteralPath $usbPath -Force -ErrorAction SilentlyContinue
Remove-Item -LiteralPath $readbackPath -Force -ErrorAction SilentlyContinue

$sourceHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $sourcePath).Hash
$writeSeconds = Invoke-UnbufferedCopy -SourceDirectory $localRoot -DestinationDirectory $usbRoot -Name $fileName
if (-not (Test-Path -LiteralPath $usbPath)) {
    throw "Write test did not create $usbPath"
}
$usbHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $usbPath).Hash

$readSeconds = Invoke-UnbufferedCopy -SourceDirectory $usbRoot -DestinationDirectory $readbackDir -Name $fileName
$readbackHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $readbackPath).Hash
$bytes = (Get-Item -LiteralPath $sourcePath).Length
$writeMiBs = ($bytes / 1MB) / [Math]::Max($writeSeconds, 0.001)
$readMiBs = ($bytes / 1MB) / [Math]::Max($readSeconds, 0.001)

$rwTest = Join-Path $usbRoot "p4_buoy_rw_test.tmp"
$renamedTest = Join-Path $usbRoot "p4_buoy_rw_test_renamed.tmp"
Set-Content -LiteralPath $rwTest -Value "P4 Buoy writable MSC test" -Encoding Ascii -NoNewline
Rename-Item -LiteralPath $rwTest -NewName ([System.IO.Path]::GetFileName($renamedTest))
$renameOk = Test-Path -LiteralPath $renamedTest
Remove-Item -LiteralPath $renamedTest -Force
$deleteOk = -not (Test-Path -LiteralPath $renamedTest)

$hashOk = $sourceHash -eq $usbHash -and $sourceHash -eq $readbackHash
$speedOk = $readMiBs -ge $MinReadMiBs -and $writeMiBs -ge $MinWriteMiBs
$result = [pscustomobject]@{
    Volume = $usbRoot
    VolumeLabel = $volume.FileSystemLabel
    DriveType = $volume.DriveType
    FileSystem = $volume.FileSystem
    Bytes = $bytes
    WriteSeconds = [Math]::Round($writeSeconds, 3)
    WriteMiBs = [Math]::Round($writeMiBs, 2)
    ReadSeconds = [Math]::Round($readSeconds, 3)
    ReadMiBs = [Math]::Round($readMiBs, 2)
    Sha256 = $sourceHash
    HashMatch = $hashOk
    CreateRenameDelete = $renameOk -and $deleteOk
    SpeedThresholdsMet = $speedOk
}
$result | Format-List

Remove-Item -LiteralPath $usbPath -Force

if (-not $hashOk -or -not $renameOk -or -not $deleteOk -or -not $speedOk) {
    throw "USB MSC acceptance failed. Keep the feature branch unmerged and inspect the reported result."
}

Write-Host "USB MSC acceptance passed. Run tools\eject_usb_msc.ps1, physically unplug the USB data cable, then restore TF from Web."
