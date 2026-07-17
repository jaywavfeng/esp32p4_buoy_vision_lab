param(
    [ValidateSet('rev1', 'rev31')]
    [string]$Profile = 'rev1',
    [switch]$Clean
)

$ProjectDir = $PSScriptRoot
$IdfPath = 'C:\esp\v6.0.1\esp-idf'
$Python = 'C:\Espressif\tools\python\v6.0.1\venv\Scripts\python.exe'
$IdfPy = Join-Path $IdfPath 'tools\idf.py'
$Objdump = 'C:\Espressif\tools\riscv32-esp-elf\esp-15.2.0_20251204\riscv32-esp-elf\bin\riscv32-esp-elf-objdump-xespv2p2.exe'

$env:IDF_TOOLS_PATH = 'C:\Espressif\tools'
$ComponentCachePath = Join-Path (Split-Path $env:IDF_TOOLS_PATH -Parent) 'idf_component_cache'
New-Item -ItemType Directory -Force -Path $ComponentCachePath | Out-Null
$env:IDF_COMPONENT_CACHE_PATH = $ComponentCachePath
$env:IDF_COMPONENT_LOCAL_STORAGE_URL = 'file://C:\Espressif\tools'
$env:IDF_PYTHON_ENV_PATH = 'C:\Espressif\tools\python\v6.0.1\venv'
$env:IDF_PATH = $IdfPath
$env:ESP_IDF_VERSION = '6.0'
$env:ESP_ROM_ELF_DIR = 'C:\Espressif\tools\esp-rom-elfs\20241011'

$env:GIT_CONFIG_COUNT = '1'
$env:GIT_CONFIG_KEY_0 = 'safe.directory'
$env:GIT_CONFIG_VALUE_0 = 'C:/esp/v6.0.1/esp-idf'

$ToolPath = @(
    'C:\Espressif\tools\ccache\4.12.1\ccache-4.12.1-windows-x86_64',
    'C:\Espressif\tools\cmake\4.0.3\bin',
    'C:\Espressif\tools\ninja\1.12.1',
    'C:\Espressif\tools\riscv32-esp-elf\esp-15.2.0_20251204\riscv32-esp-elf\bin',
    'C:\Espressif\tools\python\v6.0.1\venv\Scripts'
)
$env:PATH = ($ToolPath -join ';') + ';' + $env:PATH

Push-Location $ProjectDir
$BuildDir = Join-Path $ProjectDir ("build_" + $Profile)
$Sdkconfig = Join-Path $BuildDir 'sdkconfig'
$Defaults = @(
    (Join-Path $ProjectDir 'sdkconfig.defaults')
)
if ($Profile -eq 'rev31') {
    $Defaults += Join-Path $ProjectDir 'sdkconfig.defaults.rev31'
} else {
    $Defaults += Join-Path $ProjectDir 'sdkconfig.defaults.esp32p4'
}

$SdkconfigArg = 'SDKCONFIG=' + ($Sdkconfig -replace '\\', '/')
$DefaultsArg = 'SDKCONFIG_DEFAULTS=' + (($Defaults | ForEach-Object { $_ -replace '\\', '/' }) -join ';')
$ObjdumpArg = 'CMAKE_OBJDUMP=' + ($Objdump -replace '\\', '/')

Write-Host "Building profile $Profile in $BuildDir"
if ($Clean -and (Test-Path $BuildDir)) {
    # idf.py fullclean rejects the project's deterministic patch to the managed
    # TinyUSB component. Removing only this profile's generated build directory
    # gives the same compiler-level clean build without touching dependencies.
    $projectFull = [IO.Path]::GetFullPath($ProjectDir).TrimEnd('\', '/')
    $buildFull = [IO.Path]::GetFullPath($BuildDir).TrimEnd('\', '/')
    $expectedPrefix = $projectFull + [IO.Path]::DirectorySeparatorChar
    if (-not $buildFull.StartsWith($expectedPrefix, [StringComparison]::OrdinalIgnoreCase) -or
        -not ([IO.Path]::GetFileName($buildFull)).StartsWith('build_', [StringComparison]::Ordinal)) {
        throw "Refusing to remove unexpected build path: $buildFull"
    }
    Write-Host "Removing generated build directory $buildFull"
    Remove-Item -LiteralPath $buildFull -Recurse -Force
}
& $Python $IdfPy -B $BuildDir -D $SdkconfigArg -D $DefaultsArg -D $ObjdumpArg build
$exitCode = $LASTEXITCODE
Pop-Location
exit $exitCode
