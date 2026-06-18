param(
    [string]$Port = "COM3"
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
    & $Python $IdfPy set-target esp32p4
    & $Python $IdfPy build
    & $Python $IdfPy -p $Port flash monitor
}
finally {
    Pop-Location
}
