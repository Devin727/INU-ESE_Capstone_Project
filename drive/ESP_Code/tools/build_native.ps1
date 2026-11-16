param()
$ErrorActionPreference = 'Stop'
$projectDir = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $projectDir 'build'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
$vswherePath = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
if (!(Test-Path -LiteralPath $vswherePath)) { throw 'Visual Studio C++ Build Tools required.' }
$visualStudioDir = & $vswherePath -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (!$visualStudioDir) { throw 'MSVC C++ Build Tools not found.' }
$vcvarsPath = Join-Path $visualStudioDir 'VC\Auxiliary\Build\vcvars64.bat'
$core = '..\src\controller.cpp ..\src\motor.cpp ..\src\encoder.cpp ..\src\speed_control.cpp ..\src\communication.cpp'
$batch = @"
@echo off
call "$vcvarsPath" >nul
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /EHsc /W4 /WX /utf-8 /I ..\include $core ..\tests\test_core.cpp /Fe:test_core.exe
if errorlevel 1 exit /b 1
test_core.exe
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /EHsc /W4 /WX /utf-8 /I ..\include $core ..\tools\native_sim.cpp /Fe:native_sim.exe
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /EHsc /W4 /WX /utf-8 /I ..\include $core ..\examples\motor_duty_example.cpp /Fe:motor_duty_example.exe
if errorlevel 1 exit /b 1
motor_duty_example.exe
"@
$batchPath = Join-Path $buildDir 'build_native.cmd'
Set-Content -LiteralPath $batchPath -Value $batch -Encoding Default
Push-Location $buildDir
try {
    & $env:ComSpec /d /c "`"$batchPath`""
    if ($LASTEXITCODE -ne 0) { throw "Build or test failed: $LASTEXITCODE" }
} finally { Pop-Location }
