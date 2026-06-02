/**
 * Anti-Drone Sensor Fusion Core (ps_main.cpp)
 * FPGA PS(ARM) 실행: 레이더 UART 파싱 → CORDIC/Kalman HLS IP → Pan/Tilt 모터 제어
 * PC는 AI 추론 결과(bbox 오차)만 B:ex,ey로 전송하며, PID 계산은 FPGA 전담.
 */

#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>

#include "xparameters.h"
#include "xil_printf.h"
#include "xil_io.h"
#include "xil_cache.h"
#include "xuartps.h"
#include "sleep.h"

// ============================================================
//  컴파일 타임 설정
// ============================================================
#define LOG_LEVEL           1
#define CAM_HFOV_DEG        60.0f
#define CAM_W_PX            320
#define CAM_H_PX            240
#define RADAR_UART_BAUD     256000u
#define CONSOLE_UART_BAUD   256000u
#define RADAR_STALE_FRAMES  5
// 30Hz 타깃 (DT=0.033s). Kalman IP도 DT=0.033f로 맞춰야 함 → kalman_filter.cpp 수정 필요
#define MAIN_LOOP_SLEEP_US  33000u
#define RADAR_LOG_PERIOD_FRAMES  1

#ifndef XPAR_XUARTPS_0_BASEADDR
    #ifdef XPAR_UART0_BASEADDR
        #define CONSOLE_BASEADDR XPAR_UART0_BASEADDR
    #else
        #define CONSOLE_BASEADDR 0xE0000000u
    #endif
#else
    #define CONSOLE_BASEADDR XPAR_XUARTPS_0_BASEADDR
#endif

// ============================================================
//  HLS IP 공통 ap_ctrl 비트 (AXI4-Lite 오프셋 0x00)
// ============================================================
#define AP_START  (1u << 0)
#define AP_DONE   (1u << 1)
#define AP_IDLE   (1u << 2)

// ============================================================
//  ULN2003 모터 컨트롤러 IP 레지스터 맵
// ============================================================
#ifndef XPAR_ULN2003_CONTROLLER_0_BASEADDR
    #define MOTOR_BASEADDR  0x40030000
#else
    #define MOTOR_BASEADDR  XPAR_ULN2003_CONTROLLER_0_BASEADDR
#endif

#define MOTOR_WR(off, v)  Xil_Out32(MOTOR_BASEADDR + (off), (u32)(v))
#define MOTOR_RD(off)     Xil_In32 (MOTOR_BASEADDR + (off))

// 28BYJ-48 half-step 사양 (pan/tilt 중력 부하 없음 → 동일 속도)
#define MOTOR_STEPS_PER_REV     4096         // 1회전 = 4096 step
// hw_delay 루프카운트: 300 → 0.85ms/step
#define MOTOR_SPEED_DELAY       300          // pan/tilt 공통
// 1프레임(33ms) 내 완료 가능 최대 스텝: 33ms / 0.85ms ≈ 38
#define MOTOR_PAN_MAX_STEP      38
#define MOTOR_TILT_MAX_STEP     38
// 절대좌표 직변환: (4096/360) × 60° / 320px ≈ 2.134 step/px
// 4:3 화면(240px VFOV≈45°) 기준 수직도 동일: (4096/360) × 45° / 240px ≈ 2.134
#define STEPS_PER_DEG           (4096.0f / 360.0f)
#define STEPS_PER_PX            (STEPS_PER_DEG * CAM_HFOV_DEG / (float)CAM_W_PX)
// 픽셀 데드밴드: ±8px 이내 미동 없음 (≈17 step, ≈1.5°)
#define CAM_DEADBAND_PX         8

// ============================================================
//  CORDIC Polar IP (AXI4-Lite, 0x40000000)
//  각도 출력 단위: 0.1도 (angle_deg × 10)
// ============================================================
#ifndef XPAR_CORDIC_POLAR_0_BASEADDR
    #define CORDIC_BASEADDR  0x40000000
#else
    #define CORDIC_BASEADDR  XPAR_CORDIC_POLAR_0_BASEADDR
#endif

#define CORDIC_WR(off, v)  Xil_Out32(CORDIC_BASEADDR + (off), (u32)(v))
#define CORDIC_RD(off)     Xil_In32 (CORDIC_BASEADDR + (off))
// 레지스터: 0x10=x_in, 0x18=y_in, 0x20=distance(out), 0x30=angle_deg(out)

// ============================================================
//  Kalman Filter IP (AXI4-Lite, 0x40010000)
// ============================================================
#ifndef XPAR_KALMAN_FILTER_0_BASEADDR
    #define KALMAN_BASEADDR  0x40010000
#else
    #define KALMAN_BASEADDR  XPAR_KALMAN_FILTER_0_BASEADDR
#endif

#define KALMAN_WR(off, v)  Xil_Out32(KALMAN_BASEADDR + (off), (u32)(v))
#define KALMAN_RD(off)     Xil_In32 (KALMAN_BASEADDR + (off))

// Kalman HLS scalar-port register map from xkalman_filter_hw.h.
// Pointer outputs also have *_CTRL ap_vld registers; PS reads only *_DATA.
#define KALMAN_ADDR_AP_CTRL     0x00u
#define KALMAN_ADDR_RESET_DATA  0x10u

#define KALMAN_ADDR_T0_W1_DATA  0x18u
#define KALMAN_ADDR_T0_W0_DATA  0x20u
#define KALMAN_ADDR_T1_W0_DATA  0x28u
#define KALMAN_ADDR_T2_W0_DATA  0x30u
#define KALMAN_ADDR_T1_W1_DATA  0x38u
#define KALMAN_ADDR_T2_W1_DATA  0x40u

#define KALMAN_ADDR_S0Y_DATA    0x48u
#define KALMAN_ADDR_S1VY_DATA   0x50u
#define KALMAN_ADDR_S0VY_DATA   0x58u
#define KALMAN_ADDR_S2Y_DATA    0x60u
#define KALMAN_ADDR_S1Y_DATA    0x68u
#define KALMAN_ADDR_S2VY_DATA   0x70u
#define KALMAN_ADDR_S0X_DATA    0x80u
#define KALMAN_ADDR_S0VX_DATA   0x88u
#define KALMAN_ADDR_S0INIT_DATA 0x90u
#define KALMAN_ADDR_S1X_DATA    0xA0u
#define KALMAN_ADDR_S1VX_DATA   0xA8u
#define KALMAN_ADDR_S1INIT_DATA 0xB0u
#define KALMAN_ADDR_S2X_DATA    0xC0u
#define KALMAN_ADDR_S2VX_DATA   0xC8u
#define KALMAN_ADDR_S2INIT_DATA 0xD0u

#define MAX_TARGETS  3

// ============================================================
//  자료형
// ============================================================
typedef struct __attribute__((packed)) { int16_t x,y,speed; bool valid; } RadarTarget_t;
typedef struct __attribute__((packed)) { float x,y,vx,vy; bool init; } KalmanState_t;

static XUartPs g_console_uart;

extern "C" void outbyte(char c)
{
    while (XUartPs_IsTransmitFull(CONSOLE_BASEADDR)) {
        ;
    }
    XUartPs_WriteReg(CONSOLE_BASEADDR, XUARTPS_FIFO_OFFSET, (u32)c);
}

static int console_init(void)
{
    XUartPs_Config* c = NULL;

#if defined(SDT)
    c = XUartPs_LookupConfig(CONSOLE_BASEADDR);
#else
    c = XUartPs_LookupConfig(XPAR_XUARTPS_0_DEVICE_ID);
#endif

    if (!c) return -1;
    if (XUartPs_CfgInitialize(&g_console_uart, c, c->BaseAddress) != XST_SUCCESS) return -1;

    XUartPs_SetBaudRate(&g_console_uart, CONSOLE_UART_BAUD);
    XUartPs_SetOperMode(&g_console_uart, XUARTPS_OPER_MODE_NORMAL);
    XUartPs_SetFifoThreshold(&g_console_uart, 1);
    return 0;
}

// ============================================================
//  레이더 UART1 수신 링버퍼
// ============================================================
// 2048바이트: 30Hz 루프에서 30-byte 패킷 여러 개 누적되어도 오버플로우 없음
#define RING_SZ  2048
static XUartPs g_uart;
static uint8_t g_ring[RING_SZ];
static uint32_t g_head = 0, g_tail = 0;
static bool g_uart_ready = false;
static uint32_t g_rx_bytes = 0;
static uint32_t g_rx_packets = 0;
static uint32_t g_rx_ring_overflows = 0;
static uint32_t g_rx_hw_overruns = 0;
static uint32_t g_rx_hw_errors = 0;

static void ring_push(uint8_t b)
{
    // 오버플로우 시 가장 오래된 데이터를 버리고 최신 데이터 우선 보존
    if ((g_head - g_tail) >= RING_SZ) {
        g_tail = g_head - RING_SZ + 1u;
        g_rx_ring_overflows++;
    }
    g_ring[g_head % RING_SZ] = b;
    g_head++;
    g_rx_bytes++;
}
static uint8_t ring_at(uint32_t i) { return g_ring[i % RING_SZ]; }

// ============================================================
//  호스트(Windows PC) UART0 수신 링버퍼 및 명령 파서
// ============================================================
#define HOST_RING_SZ  256
static uint8_t  g_host_ring[HOST_RING_SZ];
static uint32_t g_host_head = 0;
static uint32_t g_host_tail = 0;
static int      g_motor_abs_pan       = 0;
static int      g_motor_abs_tilt      = 0;
static int      g_manual_pending_pan  = 0;  // M:1 수동 모드 미실행 pan 잔량
static int      g_manual_pending_tilt = 0;  // M:1 수동 모드 미실행 tilt 잔량
static int      g_abs_pending_pan     = 0;  // A: 절대좌표 명령 미실행 pan 잔량
static int      g_abs_pending_tilt    = 0;  // A: 절대좌표 명령 미실행 tilt 잔량
static int      g_host_bbox_ex        = 0;
static int      g_host_bbox_ey        = 0;
static bool     g_host_bbox_valid     = false;  // 이번 프레임에 bbox 수신됐는지 여부
static bool     g_manual_mode         = false;  // true 시 minStep 클램프 해제 (캘리브레이션용)

static void host_ring_push(uint8_t b)
{
    if ((g_host_head - g_host_tail) >= HOST_RING_SZ)
        g_host_tail = g_host_head - HOST_RING_SZ + 1u;
    g_host_ring[g_host_head % HOST_RING_SZ] = b;
    g_host_head++;
}
static uint8_t host_ring_at(uint32_t i) { return g_host_ring[i % HOST_RING_SZ]; }

static void host_accumulate(void)
{
    uint8_t tmp[64];
    int n = 0;
    while ((n = XUartPs_Recv(&g_console_uart, tmp, sizeof(tmp))) > 0)
        for (int i = 0; i < n; ++i) host_ring_push(tmp[i]);
}

// '\n' 종단 라인 단위로 B:/A:/P:/T:/M: 명령 파싱
static void host_parse_commands(void)
{
    while ((int32_t)(g_host_head - g_host_tail) > 0) {
        uint32_t scan = g_host_tail;
        bool found_nl = false;
        uint32_t nl_pos = 0;
        while ((int32_t)(g_host_head - scan) > 0) {
            if (host_ring_at(scan) == '\n') { nl_pos = scan; found_nl = true; break; }
            scan++;
        }
        if (!found_nl) break;

        int32_t len = (int32_t)(nl_pos - g_host_tail);
        if (len < 3 || len > 24) { g_host_tail = nl_pos + 1; continue; }

        uint8_t cmd = host_ring_at(g_host_tail);
        if ((cmd == 'T' || cmd == 'P' || cmd == 'M' || cmd == 'B' || cmd == 'A') &&
             host_ring_at(g_host_tail + 1) == ':') {
            if (cmd == 'B') {
                // B:ex,ey — AI bbox 중심 오차 (PC에서 스케일 변환 후 전송)
                int ex = 0, ey = 0, sign = 1;
                uint32_t p = g_host_tail + 2;
                bool ok = false;
                if ((int32_t)(nl_pos - p) > 0 && host_ring_at(p) == '-') { sign = -1; p++; }
                else if ((int32_t)(nl_pos - p) > 0 && host_ring_at(p) == '+') { p++; }
                while ((int32_t)(nl_pos - p) > 0) {
                    uint8_t c = host_ring_at(p++);
                    if (c >= '0' && c <= '9') {
                        ex = ex * 10 + (c - '0');
                        if (ex > 32767) { ok = false; break; }
                        ok = true;
                    } else if (c == ',') { ex = sign * ex; sign = 1; break; }
                    else { ok = false; break; }
                }
                if (ok) {
                    ok = false;
                    if ((int32_t)(nl_pos - p) > 0 && host_ring_at(p) == '-') { sign = -1; p++; }
                    else if ((int32_t)(nl_pos - p) > 0 && host_ring_at(p) == '+') { p++; }
                    while ((int32_t)(nl_pos - p) > 0) {
                        uint8_t c = host_ring_at(p++);
                        if (c >= '0' && c <= '9') {
                            ey = ey * 10 + (c - '0');
                            if (ey > 32767) { ok = false; break; }
                            ok = true;
                        } else { ok = false; break; }
                    }
                }
                if (ok) {
                    g_host_bbox_ex    = ex;
                    g_host_bbox_ey    = sign * ey;
                    g_host_bbox_valid = true;
                }
            } else if (cmd == 'A') {
                // A:pan,tilt — 절대 스텝 좌표 지정 (pose table 기반)
                int pan = 0, tilt_val = 0, sign = 1;
                uint32_t p = g_host_tail + 2;
                bool ok = false;
                if ((int32_t)(nl_pos-p) > 0 && host_ring_at(p) == '-') { sign = -1; p++; }
                else if ((int32_t)(nl_pos-p) > 0 && host_ring_at(p) == '+') { p++; }
                while ((int32_t)(nl_pos-p) > 0) {
                    uint8_t c = host_ring_at(p++);
                    if (c >= '0' && c <= '9') {
                        pan = pan*10 + (c-'0');
                        if (pan > 32767) { ok = false; break; }
                        ok = true;
                    } else if (c == ',') { pan = sign*pan; sign = 1; break; }
                    else { ok = false; break; }
                }
                if (ok) {
                    ok = false;
                    if ((int32_t)(nl_pos-p) > 0 && host_ring_at(p) == '-') { sign = -1; p++; }
                    else if ((int32_t)(nl_pos-p) > 0 && host_ring_at(p) == '+') { p++; }
                    while ((int32_t)(nl_pos-p) > 0) {
                        uint8_t c = host_ring_at(p++);
                        if (c >= '0' && c <= '9') {
                            tilt_val = tilt_val*10 + (c-'0');
                            if (tilt_val > 32767) { ok = false; break; }
                            ok = true;
                        } else { ok = false; break; }
                    }
                }
                if (ok) {
                    // 현재 위치 기준 상대 스텝을 펜딩 큐에 적재
                    g_abs_pending_pan  = pan           - g_motor_abs_pan;
                    g_abs_pending_tilt = sign*tilt_val - g_motor_abs_tilt;
                }
            } else {
                int val = 0, sign = 1;
                uint32_t p = g_host_tail + 2;
                if ((int32_t)(nl_pos - p) > 0 && host_ring_at(p) == '-') { sign = -1; p++; }
                else if ((int32_t)(nl_pos - p) > 0 && host_ring_at(p) == '+') { p++; }
                bool ok = false;
                while ((int32_t)(nl_pos - p) > 0) {
                    uint8_t c = host_ring_at(p++);
                    if (c >= '0' && c <= '9') {
                        val = val * 10 + (c - '0');
                        if (val > 32767) { ok = false; break; }  // 모터 스텝 범위 초과 방지
                        ok = true;
                    } else { ok = false; break; }
                }
                if (ok) {
                    if (cmd == 'T') {
                        if (g_manual_mode) g_manual_pending_tilt += sign * val;
                    } else if (cmd == 'P') {
                        if (g_manual_mode) g_manual_pending_pan  += sign * val;
                    } else if (cmd == 'M') {
                        // 모드 전환 시 잔량 클리어: 이전 명령이 모드 변경 후 실행되는 오작동 방지
                        g_manual_pending_pan  = 0;
                        g_manual_pending_tilt = 0;
                        g_manual_mode = (val != 0);
                    }
                }
            }
        }
        g_host_tail = nl_pos + 1;
    }
}

static void uart_poll_errors(void)
{
    if (!g_uart_ready) return;

    const u32 err_mask = XUARTPS_IXR_OVER | XUARTPS_IXR_PARITY |
                         XUARTPS_IXR_FRAMING | XUARTPS_IXR_RBRK;
    u32 isr = XUartPs_ReadReg(g_uart.Config.BaseAddress, XUARTPS_ISR_OFFSET);
    u32 err = isr & err_mask;
    if (err == 0u) return;

    g_rx_hw_errors |= err;
    if (err & XUARTPS_IXR_OVER) g_rx_hw_overruns++;
    XUartPs_WriteReg(g_uart.Config.BaseAddress, XUARTPS_ISR_OFFSET, err);
}

static void uart_accumulate(void)
{
    if (!g_uart_ready) return;
    uart_poll_errors();

    uint8_t tmp[128];
    int n = 0;
    while ((n = XUartPs_Recv(&g_uart, tmp, sizeof(tmp))) > 0) {
        for (int i = 0; i < n; ++i) ring_push(tmp[i]);
    }
}

// usleep() 대기 중에도 UART 수신을 1ms마다 계속 수행 → 고속 레이더 데이터 손실 방지
static void responsive_sleep_us(uint32_t total_us)
{
    while (total_us > 0u) {
        uart_accumulate();
        host_accumulate();
        const uint32_t step_us = (total_us > 1000u) ? 1000u : total_us;
        usleep(step_us);
        total_us -= step_us;
    }
    uart_accumulate();
    host_accumulate();
}

// 레이더 원시 좌표 부호 복원: MSB=1→양수, MSB=0→음수 (레이더 자체 인코딩 규약)
static int16_t decode_coord(uint16_t raw)
{
    return (raw & 0x8000) ? (int16_t)(raw - 0x8000) : -(int16_t)(raw & 0x7FFF);
}

// 링버퍼에서 헤더(0xAAFF)·푸터(0x55CC) 검증 후 최신 유효 패킷으로 덮어씀
static bool uart_parse(RadarTarget_t tgt[3], int* vc)
{
    bool found_valid = false;

    while ((int32_t)(g_head - g_tail) >= 30) {
        if (ring_at(g_tail) != 0xAA || ring_at(g_tail+1) != 0xFF)
            { g_tail++; continue; }

        uint8_t pkt[30];
        for (int i = 0; i < 30; ++i) pkt[i] = ring_at(g_tail + i);

        if (pkt[2]!=0x03 || pkt[3]!=0x00 || pkt[28]!=0x55 || pkt[29]!=0xCC)
            { g_tail++; continue; }

        *vc = 0;
        for (int t = 0; t < 3; ++t) {
            const uint8_t* p = &pkt[4 + t*8];
            uint16_t rx = p[0]|((uint16_t)p[1]<<8);
            uint16_t ry = p[2]|((uint16_t)p[3]<<8);
            uint16_t rs = p[4]|((uint16_t)p[5]<<8);
            tgt[t] = { decode_coord(rx), decode_coord(ry),
                       decode_coord(rs), (bool)(rx||ry) };
            if (tgt[t].valid) (*vc)++;
        }
        g_tail += 30;
        g_rx_packets++;
        found_valid = true;
    }
    return found_valid;
}

static int uart_init(void)
{
    XUartPs_Config* c = NULL;
    g_uart_ready = false;

#if defined(SDT)
    c = XUartPs_LookupConfig(XPAR_XUARTPS_1_BASEADDR);
#else
    c = XUartPs_LookupConfig(XPAR_XUARTPS_1_DEVICE_ID);
#endif

    if (!c) return -1;
    if (XUartPs_CfgInitialize(&g_uart, c, c->BaseAddress) != XST_SUCCESS) return -1;

    XUartPs_SetBaudRate(&g_uart, RADAR_UART_BAUD);
    XUartPs_SetOperMode(&g_uart, XUARTPS_OPER_MODE_NORMAL);
    XUartPs_WriteReg(g_uart.Config.BaseAddress, XUARTPS_ISR_OFFSET, XUARTPS_IXR_MASK);
    g_head = 0;
    g_tail = 0;
    g_uart_ready = true;
    XUartPs_SetFifoThreshold(&g_uart, 1);

    return 0;
}

// ============================================================
//  CORDIC Polar IP 호출
// ============================================================
// (y, x) 순서로 입력: 레이더 좌표계에서 정면=0도 유지를 위한 축 교환
static void cordic_ip_call(int16_t xi, int16_t yi,
                            uint16_t* dm, int16_t* at)
{
    CORDIC_WR(0x10, (u32)(int32_t)xi);
    CORDIC_WR(0x18, (u32)(int32_t)yi);
    CORDIC_WR(0x00, AP_START);
    u32 to = 500000;
    while (!(CORDIC_RD(0x00) & AP_DONE)) {
        if ((to & 0x3FFu) == 0u) uart_accumulate();
        if (--to == 0) {
            xil_printf("[WARN] CORDIC IP timeout\n");
            *dm = 0; *at = 0; return;
        }
    }
    *dm = (uint16_t)(CORDIC_RD(0x20) & 0xFFFFu);
    *at = (int16_t) (CORDIC_RD(0x30) & 0xFFFFu);
}

static void process_radar_target(const RadarTarget_t& t, uint16_t* dist_mm, int16_t* yaw_deg10)
{
    cordic_ip_call(t.y, t.x, dist_mm, yaw_deg10);
}

// ============================================================
//  Kalman Filter IP
//  내부 상태(공분산 행렬 등)는 PL LUTRAM에 유지 → PS가 별도 보관 불필요
// ============================================================

// RadarTarget_t → 64-bit AXI 패킹 (IP 입력 포맷)
static void kalman_write_target(int n, const RadarTarget_t* t)
{
    static const u32 w0_addr[MAX_TARGETS] = {
        KALMAN_ADDR_T0_W0_DATA,
        KALMAN_ADDR_T1_W0_DATA,
        KALMAN_ADDR_T2_W0_DATA
    };
    static const u32 w1_addr[MAX_TARGETS] = {
        KALMAN_ADDR_T0_W1_DATA,
        KALMAN_ADDR_T1_W1_DATA,
        KALMAN_ADDR_T2_W1_DATA
    };
    u32 w0 = ((u32)(uint16_t)(int16_t)t->y << 16)
           | ((u32)(uint16_t)(int16_t)t->x);
    u32 w1 = ((u32)(t->valid ? 1u : 0u) << 16)
           | ((u32)(uint16_t)(int16_t)t->speed);
    KALMAN_WR(w0_addr[n], w0);
    KALMAN_WR(w1_addr[n], w1);
}

// 32-byte 블록 → KalmanState_t (5 word 유효, 나머지 reserved)
static void kalman_read_state(int n, KalmanState_t* s)
{
    static const u32 x_addr[MAX_TARGETS] = {
        KALMAN_ADDR_S0X_DATA,
        KALMAN_ADDR_S1X_DATA,
        KALMAN_ADDR_S2X_DATA
    };
    static const u32 y_addr[MAX_TARGETS] = {
        KALMAN_ADDR_S0Y_DATA,
        KALMAN_ADDR_S1Y_DATA,
        KALMAN_ADDR_S2Y_DATA
    };
    static const u32 vx_addr[MAX_TARGETS] = {
        KALMAN_ADDR_S0VX_DATA,
        KALMAN_ADDR_S1VX_DATA,
        KALMAN_ADDR_S2VX_DATA
    };
    static const u32 vy_addr[MAX_TARGETS] = {
        KALMAN_ADDR_S0VY_DATA,
        KALMAN_ADDR_S1VY_DATA,
        KALMAN_ADDR_S2VY_DATA
    };
    static const u32 init_addr[MAX_TARGETS] = {
        KALMAN_ADDR_S0INIT_DATA,
        KALMAN_ADDR_S1INIT_DATA,
        KALMAN_ADDR_S2INIT_DATA
    };
    u32 tmp;
    float fx, fy, fvx, fvy;
    tmp = KALMAN_RD(x_addr[n]);  memcpy(&fx,  &tmp, 4);
    tmp = KALMAN_RD(y_addr[n]);  memcpy(&fy,  &tmp, 4);
    tmp = KALMAN_RD(vx_addr[n]); memcpy(&fvx, &tmp, 4);
    tmp = KALMAN_RD(vy_addr[n]); memcpy(&fvy, &tmp, 4);
    bool init = (KALMAN_RD(init_addr[n]) & 0x1u) != 0u;
    *s = {fx, fy, fvx, fvy, init};
}

static bool kalman_wait_done(void)
{
    u32 to = 500000;
    while (!(KALMAN_RD(KALMAN_ADDR_AP_CTRL) & AP_DONE)) {
        if ((to & 0x3FFu) == 0u) uart_accumulate();
        if (--to == 0) {
            xil_printf("[WARN] Kalman IP timeout\n");
            return false;
        }
    }
    return true;
}

// reset=1로 IP 실행 → 내부 상태(위치·공분산) 전부 초기화
static void kalman_ip_reset(void)
{
    for (int n = 0; n < MAX_TARGETS; ++n) {
        RadarTarget_t zero = {0, 0, 0, false};
        kalman_write_target(n, &zero);
    }
    KALMAN_WR(KALMAN_ADDR_RESET_DATA, 1u);
    KALMAN_WR(KALMAN_ADDR_AP_CTRL, AP_START);
    kalman_wait_done();
    KALMAN_WR(KALMAN_ADDR_RESET_DATA, 0u);
}

static void kalman_ip_run(const RadarTarget_t* tgt, KalmanState_t* out)
{
    for (int n = 0; n < MAX_TARGETS; ++n)
        kalman_write_target(n, &tgt[n]);
    KALMAN_WR(KALMAN_ADDR_AP_CTRL, AP_START);
    if (!kalman_wait_done()) return;
    for (int n = 0; n < MAX_TARGETS; ++n)
        kalman_read_state(n, &out[n]);
}

// ============================================================
//  모터 제어 상태 변수
// ============================================================
static int   g_host_cmd_cooldown = 0;
static int   g_ai_lock_frames    = 0;
// IP 이동 완료까지 최소 2프레임(66ms) 보호 → 다음 명령이 이전 이동과 충돌 방지
#define HOST_CMD_COOLDOWN_FRAMES  2
// AI 명령 후 레이더가 즉시 오버라이드하지 않도록 4프레임(132ms) 잠금
#define AI_LOCK_FRAMES            4


// ============================================================
//  레이더 방위각 → Pan 절대 스텝 직변환 (PID 없음)
//  rang_lp: LP 필터 적용된 방위각 (0.1도 단위)
// ============================================================
static const float RADAR_STEPS_PER_DECDEG = 4096.0f / 3600.0f;
// ±2 step(≈0.18도) 이내 미세 진동 무시
#define RADAR_DEADBAND_STEPS  2

// 정지 상태에서 첫 포착 시 충격 방지용 가속 램프
#define RADAR_ACCEL_INIT_STEP  8
#define RADAR_ACCEL_INC        6
static int g_radar_accel_step = RADAR_ACCEL_INIT_STEP;

static void motor_update_radar_abs(float rang_lp_val)
{
    if (!(MOTOR_RD(0x00) & AP_IDLE)) return;

    int target_pan = (int)(rang_lp_val * RADAR_STEPS_PER_DECDEG);
    int delta      = target_pan - g_motor_abs_pan;

    if (delta >= -RADAR_DEADBAND_STEPS && delta <= RADAR_DEADBAND_STEPS) {
        g_radar_accel_step = RADAR_ACCEL_INIT_STEP;
        return;
    }

    if (g_radar_accel_step > MOTOR_PAN_MAX_STEP) g_radar_accel_step = MOTOR_PAN_MAX_STEP;
    int clamp = g_radar_accel_step;
    if (delta >  clamp) delta =  clamp;
    if (delta < -clamp) delta = -clamp;
    g_radar_accel_step += RADAR_ACCEL_INC;

    g_motor_abs_pan += delta;

    MOTOR_WR(0x10, (u32)g_motor_abs_pan);
    MOTOR_WR(0x18, (u32)g_motor_abs_tilt);
    MOTOR_WR(0x20, MOTOR_SPEED_DELAY);
    MOTOR_WR(0x00, AP_START);
}

// ============================================================
//  수동(M:1+P:/T:) 및 절대좌표(A:) 공용 펜딩 큐 실행기
//  MAX_STEP씩 분할 실행 → 모든 잔량 소진까지 유실 없음
// ============================================================
static bool motor_try_move_pending(int* ppan, int* ptilt)
{
    if (*ppan == 0 && *ptilt == 0) return false;
    if (!(MOTOR_RD(0x00) & AP_IDLE)) return false;

    int dpan  = *ppan;
    int dtilt = *ptilt;
    if (dpan  >  MOTOR_PAN_MAX_STEP)  dpan  =  MOTOR_PAN_MAX_STEP;
    if (dpan  < -MOTOR_PAN_MAX_STEP)  dpan  = -MOTOR_PAN_MAX_STEP;
    if (dtilt >  MOTOR_TILT_MAX_STEP) dtilt =  MOTOR_TILT_MAX_STEP;
    if (dtilt < -MOTOR_TILT_MAX_STEP) dtilt = -MOTOR_TILT_MAX_STEP;

    *ppan  -= dpan;
    *ptilt -= dtilt;

    g_motor_abs_pan  += dpan;
    g_motor_abs_tilt += dtilt;

    u32 speed = (u32)MOTOR_SPEED_DELAY;

    MOTOR_WR(0x10, (u32)g_motor_abs_pan);
    MOTOR_WR(0x18, (u32)g_motor_abs_tilt);
    MOTOR_WR(0x20, speed);
    MOTOR_WR(0x00, AP_START);
    return true;
}

// ============================================================
//  AI bbox 픽셀 오차 → 절대 스텝 직변환 이동
//  28BYJ-48: STEPS_PER_PX ≈ 2.134 step/px (60° HFOV / 320px)
//  PID 없음: 오차를 스텝으로 직접 변환 후 한 번에 이동
// ============================================================
static void motor_update_ai_abs(int bbox_ex, int bbox_ey)
{
    if (!(MOTOR_RD(0x00) & AP_IDLE)) return;

    if (bbox_ex > -CAM_DEADBAND_PX && bbox_ex < CAM_DEADBAND_PX &&
        bbox_ey > -CAM_DEADBAND_PX && bbox_ey < CAM_DEADBAND_PX) return;

    // 픽셀 오차 → 스텝 변환: 부호 절삭(truncate) 방지를 위해 반올림
    int dpan  = (int)((float)bbox_ex * STEPS_PER_PX + (bbox_ex >= 0 ? 0.5f : -0.5f));
    int dtilt = (int)((float)bbox_ey * STEPS_PER_PX + (bbox_ey >= 0 ? 0.5f : -0.5f));

    // 1프레임 내 완료 가능 스텝으로 클램프 (탈조 방지)
    if (dpan  >  MOTOR_PAN_MAX_STEP)  dpan  =  MOTOR_PAN_MAX_STEP;
    if (dpan  < -MOTOR_PAN_MAX_STEP)  dpan  = -MOTOR_PAN_MAX_STEP;
    if (dtilt >  MOTOR_TILT_MAX_STEP) dtilt =  MOTOR_TILT_MAX_STEP;
    if (dtilt < -MOTOR_TILT_MAX_STEP) dtilt = -MOTOR_TILT_MAX_STEP;

    if (dpan == 0 && dtilt == 0) return;

    g_motor_abs_pan  += dpan;
    g_motor_abs_tilt += dtilt;

    u32 speed = (u32)MOTOR_SPEED_DELAY;
    MOTOR_WR(0x10, (u32)g_motor_abs_pan);
    MOTOR_WR(0x18, (u32)g_motor_abs_tilt);
    MOTOR_WR(0x20, speed);
    MOTOR_WR(0x00, AP_START);
}

// ============================================================
//  메인 루프
// ============================================================
int main(void)
{
    console_init();

    xil_printf("\r\n============================================\r\n");
    xil_printf("  Anti-Drone Sensor Fusion Core Booting...\r\n");
    xil_printf("  Phase 1: Zero-Latency Radar Tracking Mode\r\n");
    xil_printf("  Console: UART0 @ %u bps\r\n", (unsigned)CONSOLE_UART_BAUD);
    xil_printf("  Radar  : UART1 EMIO @ %u bps\r\n", (unsigned)RADAR_UART_BAUD);
    xil_printf("============================================\r\n\r\n");

    if(uart_init()!=0) xil_printf("[-] Radar UART1 init failed; check xparameters.h\n");
    else xil_printf("[+] Radar UART1 OK (%u bps, EMIO PMODA)\n", (unsigned)RADAR_UART_BAUD);

    kalman_ip_reset();
    xil_printf("[+] Kalman IP (PL HLS) OK\n");

    MOTOR_WR(0x10, 0); MOTOR_WR(0x18, 0); MOTOR_WR(0x20, MOTOR_SPEED_DELAY);
    MOTOR_WR(0x00, AP_START);
    xil_printf("[+] Motor IP OK (home position)\n\n");

    static RadarTarget_t rtgt[3];
    static KalmanState_t ks[MAX_TARGETS];

    int      fid       = 0;
    int      rvc       = 0;
    uint16_t rdist     = 0;
    int16_t  rang      = 0;
    int      radar_age = RADAR_STALE_FRAMES;
    // LP 필터 상태: α=0.4로 레이더 좌우 지터 억제 (KD항 미분 증폭 방지)
    static float rang_lp = 0.0f;

    while(true){

        // 1. 레이더 UART 수신 및 최신 패킷 파싱
        uart_accumulate();
        if(uart_parse(rtgt, &rvc)){
            radar_age = 0;
            if(rvc>0) {
                process_radar_target(rtgt[0], &rdist, &rang);
                rang_lp = 0.4f * (float)rang + 0.6f * rang_lp;
                rang = (int16_t)rang_lp;
            }
        } else if (radar_age < RADAR_STALE_FRAMES) {
            radar_age++;
        } else {
            rvc = 0;
            rdist = 0;
            rang = 0;
            rang_lp = 0.0f;  // stale 후 재포착 시 잘못된 각도에서 수렴하는 헌팅 방지
            memset(rtgt, 0, sizeof(rtgt));
        }

        // 2. 칼만 필터 예측·보정
        kalman_ip_run(rtgt, ks);

        // 3. 모터 제어 — 우선순위: 수동(M:1) > 절대좌표(A:) > AI bbox > 레이더 > 정지
        host_accumulate();
        host_parse_commands();

        if (g_manual_mode) {
            motor_try_move_pending(&g_manual_pending_pan, &g_manual_pending_tilt);
        } else if (g_abs_pending_pan != 0 || g_abs_pending_tilt != 0) {
            if (motor_try_move_pending(&g_abs_pending_pan, &g_abs_pending_tilt))
                g_host_cmd_cooldown = HOST_CMD_COOLDOWN_FRAMES;
        } else if (g_host_bbox_valid && g_host_cmd_cooldown == 0) {
            motor_update_ai_abs(g_host_bbox_ex, g_host_bbox_ey);
            g_host_cmd_cooldown = HOST_CMD_COOLDOWN_FRAMES;
            g_ai_lock_frames    = AI_LOCK_FRAMES;
        } else if (g_host_cmd_cooldown > 0) {
            g_host_cmd_cooldown--;
            if (g_ai_lock_frames > 0) g_ai_lock_frames--;
        } else if (g_ai_lock_frames > 0) {
            g_ai_lock_frames--;
        } else if (rvc > 0) {
            motor_update_radar_abs(rang_lp);
        }
        // 매 프레임 끝 리셋: PC가 30Hz로 재전송하지 않으면 다음 프레임은 bbox 없음으로 처리
        g_host_bbox_valid = false;

        if(rvc>0 && (fid % RADAR_LOG_PERIOD_FRAMES) == 0){
            xil_printf("[RADAR] T0:(%d,%d)mm spd=%dcm/s | dist=%dmm ang=%d.%ddeg\r\n",
                       rtgt[0].x, rtgt[0].y, rtgt[0].speed, rdist, rang/10, abs(rang%10));
        }

#if LOG_LEVEL >= 2
        if(fid % 30 == 0){
            xil_printf("\r\n[Frame %4d]===========================\r\n", fid);
            xil_printf("  [UART ] bytes=%u packets=%u ring_ovf=%u hw_ovr=%u hw_err=0x%08x\r\n",
                       (unsigned)g_rx_bytes, (unsigned)g_rx_packets,
                       (unsigned)g_rx_ring_overflows, (unsigned)g_rx_hw_overruns,
                       (unsigned)g_rx_hw_errors);

            for(int i=0;i<MAX_TARGETS;++i){
                if(!ks[i].init) continue;
                xil_printf("  [KALM ] T%d (%d,%d)mm v=(%d,%d)cm/s\r\n",
                           i, (int)ks[i].x, (int)ks[i].y, (int)ks[i].vx, (int)ks[i].vy);
            }
        }
#endif
        fid++;
        responsive_sleep_us(MAIN_LOOP_SLEEP_US);
    }

    return 0;
}
