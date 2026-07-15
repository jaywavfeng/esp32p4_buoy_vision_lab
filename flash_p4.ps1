param(
    [string]$Port = "",
    [ValidateSet("rev1", "rev31")]
    [string]$Profile = "rev31",
    [switch]$SkipBuild,
    [switch]$Monitor
)

$ErrorActionPreference = "Stop"

$ProjectDir = $PSScriptRoot
$IdfPath = "C:\esp\v6.0.1\esp-idf"
$Python = "C:\Espressif\tools\python\v6.0.1\venv\Scripts\python.exe"
$IdfPy = Join-Path $IdfPath "tools\idf.py"

$env:IDF_TOOLS_PATH = "C:\Espressif\tools"
$env:IDF_COMPONENT_CACHE_PATH = Join-Path $ProjectDir ".idf_component_cache"
$env:IDF_COMPONENT_LOCAL_STORAGE_URL = "file://C:\Espressif\tools"
$env:IDF_PYTHON_ENV_PATH = "C:\Espressif\tools\python\v6.0.1\venv"
$env:IDF_PATH = $IdfPath
$env:ESP_IDF_VERSION = "6.0"
$env:ESP_ROM_ELF_DIR = "C:\Espressif\tools\esp-rom-elfs\20241011"

$env:GIT_CONFIG_COUNT = "1"
$env:GIT_CONFIG_KEY_0 = "safe.directory"
$env:GIT_CONFIG_VALUE_0 = "C:/esp/v6.0.1/esp-idf"

function Find-EspSerialPort {
    $ports = @(Get-CimInstance Win32_SerialPort -ErrorAction SilentlyContinue |
        Where-Object {
            $_.DeviceID -and (
                $_.Name -match "USB|UART|JTAG|Serial|CP210|CH340|CH343|WCH|Silicon Labs|Espressif|ESP32" -or
                $_.Description -match "USB|UART|JTAG|Serial|CP210|CH340|CH343|WCH|Silicon Labs|Espressif|ESP32"
            )
        } |
        Sort-Object DeviceID)

    if ($ports.Count -eq 0) {
        $ports = @(Get-PnpDevice -Class Ports -ErrorAction SilentlyContinue |
            Where-Object {
                $_.FriendlyName -match "USB|UART|JTAG|Serial|CP210|CH340|CH343|WCH|Silicon Labs|Espressif|ESP32"
            } |
            ForEach-Object {
                if ($_.FriendlyName -match "\(COM[0-9]+\)") {
                    [pscustomobject]@{
                        DeviceID = $Matches[0].Trim("()")
                        Name = $_.FriendlyName
                    }
                }
            } |
            Sort-Object DeviceID)
    }

    if ($ports.Count -eq 0) {
        $ports = @([System.IO.Ports.SerialPort]::GetPortNames() |
            Sort-Object |
            ForEach-Object {
                [pscustomobject]@{
                    DeviceID = $_
                    Name = "Serial port $_"
                }
            })
    }

    if ($ports.Count -eq 1) {
        return $ports[0].DeviceID
    }
    if ($ports.Count -gt 1) {
        Write-Host "Detected serial ports:"
        $ports | ForEach-Object { Write-Host ("  {0}  {1}" -f $_.DeviceID, $_.Name) }
        return $ports[0].DeviceID
    }
    return $null
}

$Port = $Port.Trim()
if (-not $Port) {
    $Port = Find-EspSerialPort
}
if (-not $Port) {
    throw "No ESP32-P4 UART/USB serial port detected. Connect the board serial port or rerun with -Port COMx."
}
Write-Host "Flashing ESP32-P4 on $Port"

$ToolPath = @(
    "C:\Espressif\tools\ccache\4.12.1\ccache-4.12.1-windows-x86_64",
    "C:\Espressif\tools\cmake\4.0.3\bin",
    "C:\Espressif\tools\ninja\1.12.1",
    "C:\Espressif\tools\riscv32-esp-elf\esp-15.2.0_20251204\riscv32-esp-elf\bin",
    "C:\Espressif\tools\python\v6.0.1\venv\Scripts"
)
$env:PATH = ($ToolPath -join ";") + ";" + $env:PATH

Push-Location $ProjectDir
try {
    $BuildDir = Join-Path $ProjectDir ("build_" + $Profile)
    if (-not $SkipBuild) {
        # build_tmp.ps1 intentionally calls `exit`; run it in a child host so
        # a successful build cannot terminate this script before flashing.
        $PowerShellHost = (Get-Process -Id $PID).Path
        & $PowerShellHost -NoProfile -ExecutionPolicy Bypass -File `
            (Join-Path $ProjectDir "build_tmp.ps1") -Profile $Profile
        if ($LASTEXITCODE -ne 0) {
            throw "ESP-IDF $Profile build failed with exit code $LASTEXITCODE"
        }
    }
    $AppImage = Join-Path $BuildDir "esp32p4_buoy_vision_lab.bin"
    if (-not (Test-Path -LiteralPath $AppImage)) {
        throw "Missing $Profile application image: $AppImage. Build it before flashing."
    }
    Write-Host "Flashing profile $Profile from $BuildDir"
    Write-Host "This writes bootloader, partition table and app only; it does not erase NVS or TF."
    & $Python $IdfPy -B $BuildDir -p $Port flash
    if ($LASTEXITCODE -ne 0) {
        throw "ESP-IDF flash failed with exit code $LASTEXITCODE"
    }
    if ($Monitor) {
        & $Python $IdfPy -B $BuildDir -p $Port monitor
    }
}
finally {
    Pop-Location
}
