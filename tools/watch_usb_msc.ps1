param(
    [string]$BaseUrl = "http://169.254.100.2",
    [string]$VolumeLabel = "P4_BUOY",
    [int]$WaitSeconds = 45,
    [int]$PollSeconds = 2
)

$ErrorActionPreference = "Stop"

function Get-BoardUsbStatus {
    param([string]$Url)

    try {
        $status = curl.exe --connect-timeout 1 --max-time 2 --silent --show-error "$Url/api/status" | ConvertFrom-Json
        return [pscustomobject]@{
            Reachable = $true
            AppMode = $status.app_mode
            UsbInitialized = [bool]$status.usb_initialized
            UsbHostConnected = [bool]$status.usb_host_connected
            UsbStorageOwner = [string]$status.usb_storage_owner
            StorageQuiescing = [bool]$status.storage_quiescing
            UsbLastError = [string]$status.usb_last_error
        }
    }
    catch {
        return [pscustomobject]@{
            Reachable = $false
            AppMode = ""
            UsbInitialized = $false
            UsbHostConnected = $false
            UsbStorageOwner = ""
            StorageQuiescing = $false
            UsbLastError = $_.Exception.Message
        }
    }
}

function Get-UsbVolume {
    param([string]$Label)

    Get-Volume | Where-Object {
        $_.FileSystemLabel -eq $Label -and $_.DriveLetter
    } | Select-Object -First 1
}

function Get-UsbDeviceSummary {
    Get-PnpDevice -PresentOnly | Where-Object {
        $_.InstanceId -like "USB*" -or
        $_.FriendlyName -match "ESP|TinyUSB|P4|Buoy|Mass Storage|MSC|Serial|JTAG|CDC|USB"
    } | Select-Object Class,FriendlyName,Status,InstanceId
}

$deadline = (Get-Date).AddSeconds($WaitSeconds)
$lastSnapshot = $null
do {
    $board = Get-BoardUsbStatus -Url $BaseUrl
    $volume = Get-UsbVolume -Label $VolumeLabel
    $disks = Get-Disk | Select-Object Number,FriendlyName,BusType,Size,PartitionStyle,OperationalStatus

    $lastSnapshot = [pscustomobject]@{
        Time = Get-Date
        Board = $board
        Volume = $volume
        Disks = $disks
        UsbDevices = Get-UsbDeviceSummary
    }

    if ($volume) {
        $root = "{0}:\" -f $volume.DriveLetter
        Write-Host "USB MSC volume found at $root"
        $lastSnapshot.Board | Format-List
        $volume | Format-List
        exit 0
    }

    Write-Host ("[{0:HH:mm:ss}] board_reachable={1} mode={2} usb_init={3} host={4} owner={5} last='{6}' volume={7}" -f `
        (Get-Date), $board.Reachable, $board.AppMode, $board.UsbInitialized, `
        $board.UsbHostConnected, $board.UsbStorageOwner, $board.UsbLastError, `
        [bool]$volume)

    Start-Sleep -Seconds ([Math]::Max(1, $PollSeconds))
} while ((Get-Date) -lt $deadline)

Write-Host "Timed out waiting for USB MSC volume '$VolumeLabel'. Last board snapshot:"
$lastSnapshot.Board | Format-List
Write-Host "Present USB-like PnP devices:"
$lastSnapshot.UsbDevices | Format-Table -AutoSize
Write-Host "Current disks:"
$lastSnapshot.Disks | Format-Table -AutoSize
exit 2
