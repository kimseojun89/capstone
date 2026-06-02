# run_system.ps1 — Anti-Drone 통합 실행 파이프라인 (단일 진입점)
#
# 사용 예:
#   .\run_system.ps1 -Preflight
#   .\run_system.ps1 -Flash -SerialPort COM4 -EnableMotor
#   .\run_system.ps1 -SerialPort COM4 -EnableMotor
#   .\run_system.ps1 -Flash -SerialPort COM4 -Calibrate
#   .\run_system.ps1 -SerialPort COM4 -Calibrate
param(
    [switch]$Flash,
    [switch]$Preflight,
    [switch]$Calibrate,
    [string]$SerialPort = 'COM4',
    [string]$Baud       = '256000',
    [switch]$EnableMotor,
    [string]$Camera     = '1',
    [string]$Model      = ''
)

$ErrorActionPreference = 'Stop'
$ROOT = Split-Path -Parent $MyInvocation.MyCommand.Path

# ── 경로 정의 ──────────────────────────────────────────────────────────
$TRACKER = Join-Path $ROOT 'cpp\build_win\ptcamera_tracker.exe'
$ONNXDLL = Join-Path $ROOT 'cpp\build_win\onnxruntime.dll'
$VIEWER  = Join-Path $ROOT 'unified_gui.py'
$TCL     = Join-Path $ROOT 'flash.tcl'
$XSCT    = 'C:\Xilinx\Vitis\2023.2\bin\xsct.bat'
$PYTHON  = Join-Path $ROOT '.venv\Scripts\python.exe'
$BIT     = Join-Path $ROOT 'vitis_workspace\antidrone_app\_ide\bitstream\antidrone_wrapper.bit'
$ELF     = Join-Path $ROOT 'vitis_workspace\antidrone_app\build\antidrone_app.elf'
$CAPTURE = Join-Path $ROOT '..\scripts\pantilt_calibration\capture_pantilt_motor_dataset.py'
$DATADIR = Join-Path $ROOT '..\data\pantilt'

if ($Model -ne '') {
    $MODELPATH = $Model
} else {
    $MODELPATH = Join-Path $ROOT 'models\drone_yolov8x\best.onnx'
}

# ── 사전점검 ────────────────────────────────────────────────────────────
function Invoke-Preflight {
    Write-Host '── Preflight check ────────────────────────' -ForegroundColor Cyan
    $script:fail = 0

    function Check-File($label, $path, $required) {
        if (Test-Path $path) {
            Write-Host ("  [OK]   {0}" -f $label) -ForegroundColor Green
        } elseif ($required) {
            Write-Host ("  [FAIL] {0} not found: {1}" -f $label, $path) -ForegroundColor Red
            $script:fail++
        } else {
            Write-Host ("  [WARN] {0} not found: {1}" -f $label, $path) -ForegroundColor Yellow
        }
    }

    if (-not $Calibrate) {
        Check-File 'ptcamera_tracker.exe' $TRACKER   $true
        Check-File 'onnxruntime.dll'      $ONNXDLL   $true
        Check-File 'best.onnx'            $MODELPATH $true
    }
    Check-File 'python (venv)' $PYTHON $true
    Check-File 'unified_gui.py' $VIEWER $true

    if ($Flash) {
        Check-File 'xsct.bat'    $XSCT $true
        Check-File 'flash.tcl'   $TCL  $true
        Check-File '.bit file'   $BIT  $true
        Check-File '.elf file'   $ELF  $true
    }

    $ports = [System.IO.Ports.SerialPort]::GetPortNames()
    if ($ports -contains $SerialPort) {
        Write-Host ("  [OK]   Serial port {0} found" -f $SerialPort) -ForegroundColor Green
    } else {
        $list = if ($ports) { $ports -join ', ' } else { '(none)' }
        Write-Host ("  [WARN] Serial port {0} not found. Available: {1}" -f $SerialPort, $list) -ForegroundColor Yellow
    }

    Write-Host '───────────────────────────────────────────' -ForegroundColor Cyan
    if ($script:fail -gt 0) {
        Write-Host ("[Preflight FAIL] {0} required file(s) missing" -f $script:fail) -ForegroundColor Red
        return $false
    }
    Write-Host '[Preflight OK]' -ForegroundColor Green
    return $true
}

$ok = Invoke-Preflight
if (-not $ok) { exit 1 }
if ($Preflight) {
    Write-Host '[Preflight only] Done.' -ForegroundColor DarkGray
    exit 0
}

# ── 1. FPGA 플래시 ─────────────────────────────────────────────────────
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
    Write-Host '[1/3] Flash skipped' -ForegroundColor DarkGray
}

# ── 2. 통합 GUI ────────────────────────────────────────────────────────
Write-Host '[2/3] Starting unified_gui.py...' -ForegroundColor Cyan
Start-Process -FilePath $PYTHON -ArgumentList ('"{0}"' -f $VIEWER)
Start-Sleep -Seconds 1

# ── 3. 트래커 또는 캘리브레이션 스크립트 ──────────────────────────────
if ($Calibrate) {
    Write-Host '[3/3] Calibration mode — tracker NOT started (no COM port conflict)' -ForegroundColor Yellow
    Write-Host '      Keys: arrows=pan/tilt  Space=save  q=quit' -ForegroundColor DarkGray

    $captureArgList = @(
        ('"{0}"' -f $CAPTURE),
        '--serial-port', $SerialPort,
        '--baud', $Baud,
        '--camera-index', $Camera,
        '--show-detections'
    )
    & $PYTHON $captureArgList

    Write-Host '[DONE] Calibration session ended.' -ForegroundColor Green
    Write-Host ('       Data saved to: {0}' -f $DATADIR)
} else {
    $argList = @(
        '--serial-port', $SerialPort,
        '--baud', $Baud,
        '--camera', $Camera,
        '--model', ('"{0}"' -f $MODELPATH)
    )
    if ($EnableMotor) { $argList += '--enable-motor' }

    Write-Host '[3/3] Starting ptcamera_tracker...' -ForegroundColor Cyan
    Start-Process -FilePath $TRACKER -ArgumentList $argList

    Write-Host '[DONE] System running.' -ForegroundColor Green
    Write-Host ('       Tracker: {0}' -f $TRACKER)
    Write-Host ('       Serial:  {0}  relaying [RADAR] to UDP:9999' -f $SerialPort)
}
