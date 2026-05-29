# run_system.ps1 — Anti-Drone 통합 실행 파이프라인 (단일 진입점)
#
# 사용 예:
#   .\run_system.ps1 -Preflight                                   # 하드웨어 무접촉 사전점검만
#   .\run_system.ps1 -Flash -SerialPort COM4 -EnableMotor         # FPGA 플래시 포함 전체 실행
#   .\run_system.ps1 -SerialPort COM4 -EnableMotor                # 플래시 생략 (FPGA 이미 실행 중)
param(
    [switch]$Flash,
    [switch]$Preflight,                 # 점검만 하고 종료 (하드웨어 무접촉)
    [string]$SerialPort = 'COM4',
    [string]$Baud       = '256000',
    [switch]$EnableMotor,
    [string]$Camera     = '1',
    [string]$Model      = ''
)

$ErrorActionPreference = 'Stop'
$ROOT = Split-Path -Parent $MyInvocation.MyCommand.Path

# ── 경로 정의 ───────────────────────────────────────────
$TRACKER = "$ROOT\cpp\build_win\ptcamera_tracker.exe"
$ONNXDLL = "$ROOT\cpp\build_win\onnxruntime.dll"
$VIEWER  = "$ROOT\unified_gui.py"
$TCL     = "$ROOT\flash.tcl"
$XSCT    = 'C:\Xilinx\Vitis\2023.2\bin\xsct.bat'
$PYTHON  = "$ROOT\.venv\Scripts\python.exe"
$BIT     = "$ROOT\vitis_workspace\antidrone_app\_ide\bitstream\antidrone_wrapper.bit"
$ELF     = "$ROOT\vitis_workspace\antidrone_app\build\antidrone_app.elf"
if ($Model -ne '') { $MODELPATH = $Model } else { $MODELPATH = "$ROOT\models\drone_yolov8x\best.onnx" }

# ── 사전점검 ────────────────────────────────────────────
function Invoke-Preflight {
    Write-Host '── Preflight 점검 ─────────────────────────' -ForegroundColor Cyan
    $fail = 0
    function Check-File($label, $path, $required) {
        if (Test-Path $path) {
            Write-Host ("  [OK]   {0}" -f $label) -ForegroundColor Green
        } elseif ($required) {
            Write-Host ("  [FAIL] {0} : 없음 -> {1}" -f $label, $path) -ForegroundColor Red
            $script:fail++
        } else {
            Write-Host ("  [WARN] {0} : 없음 -> {1}" -f $label, $path) -ForegroundColor Yellow
        }
    }

    # 필수: 호스트 실행 일체
    Check-File 'tracker 실행파일'      $TRACKER  $true
    Check-File 'ONNX Runtime DLL'      $ONNXDLL  $true
    Check-File 'YOLO 모델(best.onnx)'  $MODELPATH $true
    Check-File 'venv Python'           $PYTHON   $true
    Check-File '통합 GUI'              $VIEWER   $true

    # -Flash 시 필수
    if ($Flash) {
        Check-File 'XSCT (xsct.bat)'        $XSCT $true
        Check-File 'flash.tcl'              $TCL  $true
        Check-File 'PL 비트스트림(.bit)'    $BIT  $true
        Check-File '베어메탈 ELF(.elf)'     $ELF  $true
    }

    # COM 포트 (없어도 경고만; 보드 미연결 상태 점검 가능)
    $ports = [System.IO.Ports.SerialPort]::GetPortNames()
    if ($ports -contains $SerialPort) {
        Write-Host ("  [OK]   시리얼 포트 {0} 감지 (사용 가능: {1})" -f $SerialPort, ($ports -join ', ')) -ForegroundColor Green
    } else {
        $list = if ($ports) { $ports -join ', ' } else { '(없음)' }
        Write-Host ("  [WARN] 시리얼 포트 {0} 미감지. 현재 포트: {1}" -f $SerialPort, $list) -ForegroundColor Yellow
    }

    Write-Host '───────────────────────────────────────────' -ForegroundColor Cyan
    if ($script:fail -gt 0) {
        Write-Host ("[Preflight 실패] 필수 항목 {0}개 누락" -f $script:fail) -ForegroundColor Red
        return $false
    }
    Write-Host '[Preflight 통과] 필수 항목 모두 확인' -ForegroundColor Green
    return $true
}

$ok = Invoke-Preflight
if (-not $ok) { exit 1 }
if ($Preflight) {
    Write-Host '[Preflight 모드] 점검만 수행하고 종료 (하드웨어 무접촉).' -ForegroundColor DarkGray
    exit 0
}

# ── 1. FPGA 플래시 ──────────────────────────────────────
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

# ── 2. 통합 GUI ────────────────────────────────────────
Write-Host '[2/3] Starting unified GUI (UDP:9998/9999)...' -ForegroundColor Cyan
Start-Process -FilePath $PYTHON -ArgumentList "`"$VIEWER`""
Start-Sleep -Seconds 1

# ── 3. 카메라 트래커 ───────────────────────────────────
Write-Host '[3/3] Starting ptcamera_tracker...' -ForegroundColor Cyan
$argStr = "--serial-port $SerialPort --baud $Baud --camera $Camera --model `"$MODELPATH`""
if ($EnableMotor) { $argStr += ' --enable-motor' }
Start-Process -FilePath $TRACKER -ArgumentList $argStr

Write-Host '[DONE] System running.' -ForegroundColor Green
Write-Host "       $TRACKER owns $SerialPort, relaying [RADAR] to UDP:9999"
