# Builds Signal Finder & Analyzer on Windows. Output: build\signal_finder.bin
#
# Expects ESP-IDF v5.5.x. Defaults below can be overridden with environment
# variables, or by a build.local.ps1 next to this file (not checked in), e.g.
#   $env:PATH = 'C:\path\to\Python312;' + $env:PATH
$ErrorActionPreference = 'Continue'
if (Test-Path "$PSScriptRoot\build.local.ps1") { . "$PSScriptRoot\build.local.ps1" }
if (-not $env:IDF_PATH) { $env:IDF_PATH = 'C:\esp\esp-idf-v5.5.4' }
if (-not $env:IDF_TOOLS_PATH) { $env:IDF_TOOLS_PATH = 'C:\esp\.espressif' }
# Short cache path: ESP-Hosted ships deeply nested files that exceed
# Windows' 260-character limit under the default cache location.
if (-not $env:IDF_COMPONENT_CACHE_PATH) { $env:IDF_COMPONENT_CACHE_PATH = 'C:\esp\cc' }
. "$env:IDF_PATH\export.ps1" | Out-Null
Set-Location $PSScriptRoot
if (-not (Test-Path sdkconfig)) { idf.py set-target esp32p4 }
# Extra arguments go to idf.py, for example: build.ps1 -DFINDER_DEMO=1
idf.py @args build
exit $LASTEXITCODE
