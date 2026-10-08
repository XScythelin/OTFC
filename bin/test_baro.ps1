param([string]$Compiler = "g++")

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$output = Join-Path $root ".pio\barometer-tests"
New-Item -ItemType Directory -Force -Path $output | Out-Null
$executable = Join-Path $output "barometer-tests.exe"
$source = Join-Path $root "lib\Xsfc\src"
$stubs = Join-Path $root "test\barometer\stubs"
$ahrs = Join-Path $root "lib\AHRS\src"
$files = @(
  (Join-Path $root "test\barometer\test_baro.cpp"),
  (Join-Path $root "test\barometer\test_navigation.cpp"),
  (Join-Path $source "Device\Baro\BaroMS5611.cpp"),
  (Join-Path $source "Device\Rangefinder\RangefinderVL53L0X.cpp"),
  (Join-Path $source "Device\BusSPI.cpp"),
  (Join-Path $source "Control\Pid.cpp"),
  (Join-Path $source "Utils\Filter.cpp"),
  (Join-Path $source "Connect\Msp.cpp"),
  (Join-Path $source "Connect\MspParser.cpp"),
  (Join-Path $source "Utils\Crc.cpp")
)
$compilerPath = (Get-Command $Compiler -ErrorAction Stop).Source
$previousPath = $env:PATH
try {
  $env:PATH = (Split-Path -Parent $compilerPath) + ";" + $previousPath
  & $compilerPath -std=c++17 -static -pthread -DUNIT_TEST -Wall -Wextra -Werror -Wno-unused-parameter "-I$stubs" "-I$source" "-I$ahrs" @files -o $executable
  if ($LASTEXITCODE -ne 0) { throw "Navigation test compilation failed ($LASTEXITCODE)" }
  & $executable
  if ($LASTEXITCODE -ne 0) { throw "Navigation regression tests failed ($LASTEXITCODE)" }
}
finally { $env:PATH = $previousPath }
