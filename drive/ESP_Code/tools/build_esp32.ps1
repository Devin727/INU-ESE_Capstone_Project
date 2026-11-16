param(
    [Parameter(Mandatory = $true)][string]$Fqbn,
    [switch]$Upload,
    [string]$Port
)
$ErrorActionPreference = 'Stop'
if (!(Get-Command arduino-cli -ErrorAction SilentlyContinue)) { throw 'Install arduino-cli and the selected Arduino-ESP32 3.x core first.' }
if ($Upload -and !$Port) { throw '-Upload requires -Port.' }
$projectDir = Split-Path -Parent $PSScriptRoot
$sketchDir = Join-Path $projectDir 'build\arduino\ESP_Code'
New-Item -ItemType Directory -Force -Path $sketchDir | Out-Null
Get-ChildItem -LiteralPath (Join-Path $projectDir 'src') -Filter '*.cpp' | Copy-Item -Destination $sketchDir -Force
Get-ChildItem -LiteralPath (Join-Path $projectDir 'include') -Filter '*.h' | Copy-Item -Destination $sketchDir -Force
$localConfig = Join-Path $sketchDir 'board_config.local.h'
if (!(Test-Path -LiteralPath (Join-Path $projectDir 'include\board_config.local.h')) -and (Test-Path -LiteralPath $localConfig)) {
    Remove-Item -LiteralPath $localConfig
}
Set-Content -LiteralPath (Join-Path $sketchDir 'ESP_Code.ino') -Value '// setup() and loop() are defined in main.cpp.' -Encoding ASCII
& arduino-cli compile --fqbn $Fqbn --warnings all $sketchDir
if ($LASTEXITCODE -ne 0) { throw 'ESP32 compilation failed.' }
if ($Upload) {
    & arduino-cli upload --fqbn $Fqbn --port $Port $sketchDir
    if ($LASTEXITCODE -ne 0) { throw 'ESP32 upload failed.' }
}
