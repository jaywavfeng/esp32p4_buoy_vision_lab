param(
    [string]$Port = "COM3"
)

$ErrorActionPreference = "Stop"

$ProjectDir = $PSScriptRoot

if (-not $env:IDF_PATH) {
    throw "Set IDF_PATH first, or run this script from an ESP-IDF PowerShell environment."
}

$IdfPath = $env:IDF_PATH
$IdfPy = Join-Path $IdfPath "tools\idf.py"
if (-not (Test-Path $IdfPy)) {
    throw "idf.py not found under IDF_PATH: $IdfPy"
}

$Python = "python"
if ($env:IDF_PYTHON_ENV_PATH) {
    $EnvPython = Join-Path $env:IDF_PYTHON_ENV_PATH "Scripts\python.exe"
    if (Test-Path $EnvPython) {
        $Python = $EnvPython
    }
}

if (-not $env:ESP_IDF_VERSION) {
    $env:ESP_IDF_VERSION = "6.0"
}
if (-not $env:IDF_COMPONENT_CACHE_PATH) {
    $env:IDF_COMPONENT_CACHE_PATH = Join-Path $ProjectDir ".idf_component_cache"
}

Push-Location $ProjectDir
try {
    & $Python $IdfPy set-target esp32p4
    & $Python $IdfPy build
    & $Python $IdfPy -p $Port flash monitor
}
finally {
    Pop-Location
}
