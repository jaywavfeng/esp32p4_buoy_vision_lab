param([switch]$Clean)

& (Join-Path $PSScriptRoot 'build_tmp.ps1') -Profile rev31 -Clean:$Clean
exit $LASTEXITCODE
