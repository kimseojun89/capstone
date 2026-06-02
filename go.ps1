param([string]$SerialPort = 'COM4')

$ROOT   = Split-Path -Parent $MyInvocation.MyCommand.Path
$ANTID  = Join-Path $ROOT 'antidrone'
$RUNSCR = Join-Path $ANTID 'run_system.ps1'

Write-Host ''
Write-Host '=======================================' -ForegroundColor Cyan
Write-Host '   Anti-Drone System Launcher'          -ForegroundColor Cyan
Write-Host '=======================================' -ForegroundColor Cyan
Write-Host ''
Write-Host '  [1] Track   Flash + tracker'          -ForegroundColor White
Write-Host '  [2] Track   tracker (Flash skip)'     -ForegroundColor Gray
Write-Host '  [3] Calib   Flash + calibration'      -ForegroundColor Yellow
Write-Host '  [4] Calib   calibration (Flash skip)' -ForegroundColor DarkYellow
Write-Host '  [5] Check   preflight only'           -ForegroundColor DarkGray
Write-Host '  [6] Stop    motor stop (M:0)'         -ForegroundColor Red
Write-Host ''

$choice = Read-Host 'Select'

Set-Location $ANTID

switch ($choice) {
    '1' { & $RUNSCR -Flash     -SerialPort $SerialPort -EnableMotor }
    '2' { & $RUNSCR            -SerialPort $SerialPort -EnableMotor }
    '3' { & $RUNSCR -Flash     -SerialPort $SerialPort -Calibrate   }
    '4' { & $RUNSCR            -SerialPort $SerialPort -Calibrate   }
    '5' { & $RUNSCR -Preflight -SerialPort $SerialPort              }
    '6' {
        $PYTHON = Join-Path $ANTID '.venv\Scripts\python.exe'
        & $PYTHON -c "import serial,time; s=serial.Serial('$SerialPort',256000); s.write(b'M:0\n'); time.sleep(0.1); s.close(); print('Motor stopped.')"
    }
    default { Write-Host 'Invalid.' -ForegroundColor Red }
}
