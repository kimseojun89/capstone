# run_system.ps1
param(
    [switch]$Flash,
    [string]$SerialPort = 'COM4',
    [string]$Baud       = '256000',
    [switch]$EnableMotor,
    [string]$Camera     = '1',
    [string]$Model      = ''
)

$ROOT    = Split-Path -Parent $MyInvocation.MyCommand.Path
$TRACKER = "$ROOT\cpp\build_win\ptcamera_tracker.exe"
$VIEWER  = "$ROOT\unified_gui.py"
$TCL     = "$ROOT\flash.tcl"
$XSCT    = 'C:\Xilinx\Vitis\2023.2\bin\xsct.bat'
$PYTHON  = "$ROOT\.venv\Scripts\python.exe"

# 1. FPGA flash
if ($Flash) {
    Write-Host '[1/3] Flashing FPGA...' -ForegroundColor Cyan
    & $XSCT $TCL
    if ($LASTEXITCODE -ne 0) {
        Write-Host '[ERROR] Flash failed' -ForegroundColor Red
        exit 1
    }
    Write-Host '[OK] Flash done' -ForegroundColor Green
    Start-Sleep -Seconds 2
} else {
    Write-Host '[1/3] Flash skipped (no -Flash flag)' -ForegroundColor DarkGray
}

# 2. PPI viewer
Write-Host '[2/3] Starting PPI viewer (UDP:9999)...' -ForegroundColor Cyan
if (-not (Test-Path $PYTHON)) {
    Write-Host '[ERROR] .venv not found. Run: python -m venv .venv && .venv\Scripts\pip install -r requirements-ppi.txt' -ForegroundColor Red
    exit 1
}
Start-Process -FilePath $PYTHON -ArgumentList "`"$VIEWER`""
Start-Sleep -Seconds 1

# 3. Camera tracker
Write-Host '[3/3] Starting ptcamera_tracker...' -ForegroundColor Cyan

$argStr = "--serial-port $SerialPort --baud $Baud --camera $Camera"
if ($EnableMotor) { $argStr += ' --enable-motor' }
if ($Model -ne '')  { $argStr += " --model `"$Model`"" }

Start-Process -FilePath $TRACKER -ArgumentList $argStr

Write-Host '[DONE] System running.' -ForegroundColor Green
Write-Host "       $TRACKER owns $SerialPort, relaying [RADAR] to UDP:9999"
