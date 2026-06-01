/**
 * ============================================================
 * ps_main.cpp  —  Anti-Drone Sensor Fusion Core
 * [Phase 2: FPGA-Only Control — AI 추론 제외 전부 FPGA 처리]
 * 변경 이력:
 * - SW CORDIC/Kalman 폴백 제거 (HLS IP 전용)
 * - 프로토콜 P:/T: → B:ex,ey (bbox 픽셀 오차) 추가
 *   PC는 bbox 중심 오차만 전송, PID 계산은 FPGA 전담
 * - motor_update_ai_bbox(): 카메라 Pan+Tilt PID 신설
 * - P:/T:는 수동 캘리브레이션 전용으로 유지
 * ============================================================
 */

#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>

// Xilinx BSP
#include "xparameters.h"
#include "xil_printf.h"
#include "xil_io.h"
#include "xil_cache.h"
#include "xuartps.h"
#include "sleep.h" // 블로킹 딜레이 제거됨

// ============================================================
//  컴파일 타임 설정
// ============================================================
#define LOG_LEVEL           1       // 1 = 요약 로그, 2 = 상세 로그
#define CAM_HFOV_DEG        60.0f
#define CAM_W_PX            320
#define CAM_H_PX            240
#define RADAR_UART_BAUD     256000u
#define CONSOLE_UART_BAUD   256000u  // 호스트 tilt 명령 채널 겸용
#define RADAR_STALE_FRAMES  5
#define MAIN_LOOP_SLEEP_US  33000u  // 30Hz (복원: DT=0.033과 일치)
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
//  HLS IP 공통 ap_ctrl 비트 (CORDIC/Kalman/Motor 공용)
//  ※ MTI 서브시스템은 legacy/mti_subsystem/ 로 분리됨 (2026-06-01)
// ============================================================

#define AP_START  (1u << 0)
#define AP_DONE   (1u << 1)
#define AP_IDLE   (1u << 2)

// ============================================================
//  ULN2003 모터 컨트롤러 IP 레지스터 맵
//  (xuln2003_controller_hw.h 기준)
// ============================================================
#ifndef XPAR_ULN2003_CONTROLLER_0_BASEADDR
    #define MOTOR_BASEADDR  0x40030000
#else
    #define MOTOR_BASEADDR  XPAR_ULN2003_CONTROLLER_0_BASEADDR
#endif

#define MOTOR_WR(off, v)  Xil_Out32(MOTOR_BASEADDR + (off), (u32)(v))
#define MOTOR_RD(off)     Xil_In32 (MOTOR_BASEADDR + (off))

// Pan/Tilt PID 파라미터 — C++ TrackerSettings 기본값과 동일
#define MOTOR_KP            1.5f
#define MOTOR_KD            0.0f   // D항 제거: 레이더 노이즈 증폭 방지 (P제어만 사용)
#define MOTOR_PAN_DEADBAND  35      // 픽셀
#define MOTOR_TILT_DEADBAND 35      // 픽셀
// MAX_STEP = 33ms(프레임) / ms/step 으로 맞춰야 IP실행≈프레임 → 30Hz 연속 업데이트
// MAX_STEP이 너무 크면 IP가 수백ms 점유 → 2~3Hz로 떨어져 오히려 느림
#define MOTOR_PAN_MIN_STEP  5
#define MOTOR_PAN_MAX_STEP  58    // 33ms / 0.57ms = 58 스텝 → IP≈프레임, 30Hz 연속
#define MOTOR_TILT_MIN_STEP 5
#define MOTOR_TILT_MAX_STEP 58
// 명령 슬루율 — AccelStepper의 setAcceleration에 해당
#define MOTOR_CMD_RAMP      20.0f
// hw_delay() 루프 카운트 — 실측 기준표:
//   80000 = 226ms/step  (35도: ~90초)
//     300 =   0.85ms/step → MAX_STEP=38 최적 (101°/s)
//     200 =   0.57ms/step → MAX_STEP=58 최적 (154°/s) ← 현재 (pan 탈조 없는 최고속)
//     170 =   0.48ms/step → pan 58스텝 연속 시 탈조 확인됨
//     150 =   0.42ms/step → 탈조 위험
// 탈조 시 → SPEED_DELAY=300, MAX_STEP=38 으로 되돌릴 것
#define MOTOR_SPEED_DELAY   200
#define MOTOR_ACQUIRE_RAMP  1.0f    // 첫 표적 획득 가속비 (1.0=억제 없음; control.cpp acquireRampScale)
#define MOTOR_DT            0.033f  // 30fps 기준 루프 주기

// ============================================================
//  CORDIC Polar IP (AXI4-Lite, 0x40000000)
//  cordic_polar.cpp 동일 알고리즘 — 출력 단위: 0.1도
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
//  kalman_filter.cpp 동일 알고리즘 — 내부 DT=0.1f (10Hz 기준)
//  ※ 현재 루프 주기 ~33ms(30fps)와 불일치 → 속도 추정치 스케일 차이 발생
//     추후 HLS DT 수정 또는 호출 주기 100ms 조정 필요
// ============================================================
#ifndef XPAR_KALMAN_FILTER_0_BASEADDR
    #define KALMAN_BASEADDR  0x40010000
#else
    #define KALMAN_BASEADDR  XPAR_KALMAN_FILTER_0_BASEADDR
#endif

#define KALMAN_WR(off, v)  Xil_Out32(KALMAN_BASEADDR + (off), (u32)(v))
#define KALMAN_RD(off)     Xil_In32 (KALMAN_BASEADDR + (off))
// targets_in: 0x20~0x3F (3×64-bit, 8byte/target)
//   Word n*2+0 at 0x20+8n: (uint16)y<<16 | (uint16)x
//   Word n*2+1 at 0x24+8n: (uint32)valid<<16 | (uint16)speed
// states_out: 0x80~0xDF (3×5×32-bit, 32byte/state, 5 words used + 3 reserved)
//   per state n at 0x80+32n: [x(f32), y(f32), vx(f32), vy(f32), init(u32 bit0)]

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
//  UART 링 버퍼 및 파싱 (지연 시간 0ms 달성 로직)
// ============================================================
#define RING_SZ  2048 // 오버플로우 방지를 위해 버퍼 증대
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
//  HOST UART0 링버퍼 — Windows 호스트 "T:±N\n" tilt 명령 수신
// ============================================================
#define HOST_RING_SZ  256
static uint8_t  g_host_ring[HOST_RING_SZ];
static uint32_t g_host_head = 0;
static uint32_t g_host_tail = 0;
static int      g_host_tilt_steps = 0;  // 수동 모드 tilt 직접 스텝 (M:1 전용)
static int      g_host_pan_steps  = 0;  // 수동 모드 pan 직접 스텝
static int      g_host_bbox_ex    = 0;  // AI 모드: bbox 중심 x 오차 (320px 기준 스케일)
static int      g_host_bbox_ey    = 0;  // AI 모드: bbox 중심 y 오차 (240px 기준 스케일)
static bool     g_host_bbox_valid = false;  // 이번 프레임 bbox 수신 여부
static bool     g_manual_mode     = false;  // 수동 모드: minStep 제한 해제 (캘리브레이션용)

static void host_ring_push(uint8_t b)
{
    if ((g_host_head - g_host_tail) >= HOST_RING_SZ)
        g_host_tail = g_host_head - HOST_RING_SZ + 1u;
    g_host_ring[g_host_head % HOST_RING_SZ] = b;
    g_host_head++;
}
static uint8_t host_ring_at(uint32_t i) { return g_host_ring[i % HOST_RING_SZ]; }

// UART0(g_console_uart) RX FIFO → 호스트 링버퍼 (논블로킹)
static void host_accumulate(void)
{
    uint8_t tmp[64];
    int n = 0;
    while ((n = XUartPs_Recv(&g_console_uart, tmp, sizeof(tmp))) > 0)
        for (int i = 0; i < n; ++i) host_ring_push(tmp[i]);
}

// "T:±N\n" 및 "P:±N\n" 파서 — 완전한 라인을 소비해 g_host_tilt/pan_steps 갱신
static void host_parse_commands(void)
{
    while ((int32_t)(g_host_head - g_host_tail) > 0) {
        // '\n' 위치 탐색
        uint32_t scan = g_host_tail;
        bool found_nl = false;
        uint32_t nl_pos = 0;
        while ((int32_t)(g_host_head - scan) > 0) {
            if (host_ring_at(scan) == '\n') { nl_pos = scan; found_nl = true; break; }
            scan++;
        }
        if (!found_nl) break;  // 아직 완전한 라인 없음

        int32_t len = (int32_t)(nl_pos - g_host_tail);
        if (len < 3 || len > 24) { g_host_tail = nl_pos + 1; continue; }

        uint8_t cmd = host_ring_at(g_host_tail);
        if ((cmd == 'T' || cmd == 'P' || cmd == 'M' || cmd == 'B') &&
             host_ring_at(g_host_tail + 1) == ':') {
            if (cmd == 'B') {
                // "B:ex,ey" — 부호 있는 정수 두 개, 쉼표 구분
                // ex: bbox_cx 오차 (320px 기준 스케일), ey: bbox_cy 오차 (240px 기준 스케일)
                int ex = 0, ey = 0, sign = 1;
                uint32_t p = g_host_tail + 2;
                bool ok = false;
                if ((int32_t)(nl_pos - p) > 0 && host_ring_at(p) == '-') { sign = -1; p++; }
                else if ((int32_t)(nl_pos - p) > 0 && host_ring_at(p) == '+') { p++; }
                while ((int32_t)(nl_pos - p) > 0) {
                    uint8_t c = host_ring_at(p++);
                    if (c >= '0' && c <= '9') { ex = ex * 10 + (c - '0'); ok = true; }
                    else if (c == ',') { ex = sign * ex; sign = 1; break; }
                    else { ok = false; break; }
                }
                if (ok) {
                    ok = false;
                    if ((int32_t)(nl_pos - p) > 0 && host_ring_at(p) == '-') { sign = -1; p++; }
                    else if ((int32_t)(nl_pos - p) > 0 && host_ring_at(p) == '+') { p++; }
                    while ((int32_t)(nl_pos - p) > 0) {
                        uint8_t c = host_ring_at(p++);
                        if (c >= '0' && c <= '9') { ey = ey * 10 + (c - '0'); ok = true; }
                        else { ok = false; break; }
                    }
                }
                if (ok) {
                    g_host_bbox_ex    = ex;
                    g_host_bbox_ey    = sign * ey;
                    g_host_bbox_valid = true;
                }
            } else {
                int val = 0, sign = 1;
                uint32_t p = g_host_tail + 2;
                if ((int32_t)(nl_pos - p) > 0 && host_ring_at(p) == '-') { sign = -1; p++; }
                else if ((int32_t)(nl_pos - p) > 0 && host_ring_at(p) == '+') { p++; }
                bool ok = false;
                while ((int32_t)(nl_pos - p) > 0) {
                    uint8_t c = host_ring_at(p++);
                    if (c >= '0' && c <= '9') { val = val * 10 + (c - '0'); ok = true; }
                    else { ok = false; break; }
                }
                if (ok) {
                    if (cmd == 'T')      g_host_tilt_steps = sign * val;
                    else if (cmd == 'P') g_host_pan_steps  = sign * val;
                    else if (cmd == 'M') g_manual_mode     = (val != 0);
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
    // 버퍼에 있는 데이터를 남김없이 모두 긁어옴 (Non-blocking)
    while ((n = XUartPs_Recv(&g_uart, tmp, sizeof(tmp))) > 0) {
        for (int i = 0; i < n; ++i) ring_push(tmp[i]);
    }
}

static void responsive_sleep_us(uint32_t total_us)
{
    while (total_us > 0u) {
        uart_accumulate();
        host_accumulate();  // 호스트 tilt 명령 폴링
        const uint32_t step_us = (total_us > 1000u) ? 1000u : total_us;
        usleep(step_us);
        total_us -= step_us;
    }
    uart_accumulate();
    host_accumulate();
}

static int16_t decode_coord(uint16_t raw)
{
    return (raw & 0x8000) ? (int16_t)(raw - 0x8000) : -(int16_t)(raw & 0x7FFF);
}

static bool uart_parse(RadarTarget_t tgt[3], int* vc)
{
    bool found_valid = false;
    
    // 버퍼를 끝까지 뒤져서 가장 '최신' 패킷으로 덮어씀
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
    
    // Vitis 2023.2 SDT vs Legacy 분기 처리
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
    XUartPs_SetFifoThreshold(&g_uart, 1); // 즉각적인 수신 반응
    
    return 0;
}

// ============================================================
//  CORDIC IP 호출 (AXI4-Lite)
//  입력 순서: (y, x) — 카메라 좌표계(정면 0도) 변환 유지
// ============================================================
static void cordic_ip_call(int16_t xi, int16_t yi,
                            uint16_t* dm, int16_t* at)
{
    CORDIC_WR(0x10, (u32)(int32_t)xi);   // x_in
    CORDIC_WR(0x18, (u32)(int32_t)yi);   // y_in
    CORDIC_WR(0x00, AP_START);
    u32 to = 500000;
    while (!(CORDIC_RD(0x00) & AP_DONE)) {
        if ((to & 0x3FFu) == 0u) uart_accumulate();
        if (--to == 0) {
            xil_printf("[WARN] CORDIC IP timeout\n");
            *dm = 0; *at = 0; return;
        }
    }
    *dm = (uint16_t)(CORDIC_RD(0x20) & 0xFFFFu);          // distance
    *at = (int16_t) (CORDIC_RD(0x30) & 0xFFFFu);          // angle_deg (0.1도 단위)
}

static void process_radar_target(const RadarTarget_t& t, uint16_t* dist_mm, int16_t* yaw_deg10)
{
    // (Y, X) 순서: 카메라 좌표계(정면 0도)로 변환
    cordic_ip_call(t.y, t.x, dist_mm, yaw_deg10);
}

// ============================================================
//  Kalman Filter IP 함수
//  내부 상태(st, P, st_init)는 PL LUTRAM에 유지 — PS 측 보관 불필요
// ============================================================

// targets_in 쓰기 헬퍼: RadarTarget_t → 64-bit AXI 메모리 패킹
//   Word 0 at 0x20+8n : (uint16)y<<16 | (uint16)x
//   Word 1 at 0x24+8n : (uint32)valid<<16 | (uint16)speed
static void kalman_write_target(int n, const RadarTarget_t* t)
{
    u32 w0 = ((u32)(uint16_t)(int16_t)t->y << 16)
           | ((u32)(uint16_t)(int16_t)t->x);
    u32 w1 = ((u32)(t->valid ? 1u : 0u) << 16)
           | ((u32)(uint16_t)(int16_t)t->speed);
    KALMAN_WR(0x20 + 8*n,     w0);
    KALMAN_WR(0x20 + 8*n + 4, w1);
}

// states_out 읽기 헬퍼: 32-byte 블록 → KalmanState_t
//   5 words: x(f32), y(f32), vx(f32), vy(f32), init(u32 bit0)
//   나머지 3 words는 reserved (읽지 않음)
static void kalman_read_state(int n, KalmanState_t* s)
{
    u32 base = 0x80 + 32 * n;
    u32 tmp;
    float fx, fy, fvx, fvy;
    tmp = KALMAN_RD(base +  0); memcpy(&fx,  &tmp, 4);
    tmp = KALMAN_RD(base +  4); memcpy(&fy,  &tmp, 4);
    tmp = KALMAN_RD(base +  8); memcpy(&fvx, &tmp, 4);
    tmp = KALMAN_RD(base + 12); memcpy(&fvy, &tmp, 4);
    bool init = (KALMAN_RD(base + 16) & 0x1u) != 0u;
    *s = {fx, fy, fvx, fvy, init};
}

// 내부 대기 헬퍼
static bool kalman_wait_done(void)
{
    u32 to = 500000;
    while (!(KALMAN_RD(0x00) & AP_DONE)) {
        if ((to & 0x3FFu) == 0u) uart_accumulate();
        if (--to == 0) {
            xil_printf("[WARN] Kalman IP timeout\n");
            return false;
        }
    }
    return true;
}

// IP 초기화: 빈 입력 + reset=1 → 내부 상태 전부 클리어
static void kalman_ip_reset(void)
{
    for (int n = 0; n < MAX_TARGETS; ++n) {
        KALMAN_WR(0x20 + 8*n,     0u);
        KALMAN_WR(0x20 + 8*n + 4, 0u);
    }
    KALMAN_WR(0x10, 1u);    // reset = true
    KALMAN_WR(0x00, AP_START);
    kalman_wait_done();
    KALMAN_WR(0x10, 0u);    // reset 플래그 해제 (다음 run 에서 0 유지)
}

// IP 실행: 표적 입력 → 처리 → 상태 읽기
static void kalman_ip_run(const RadarTarget_t* tgt, KalmanState_t* out)
{
    for (int n = 0; n < MAX_TARGETS; ++n)
        kalman_write_target(n, &tgt[n]);
    // reset 레지스터는 kalman_ip_reset() 이후 0으로 유지됨
    KALMAN_WR(0x00, AP_START);
    if (!kalman_wait_done()) return;
    for (int n = 0; n < MAX_TARGETS; ++n)
        kalman_read_state(n, &out[n]);
}

// ULN2003 모터 PID 상태 (pan/tilt 각축 독립)
static float g_pan_prev_err  = 0.0f;
static float g_tilt_prev_err = 0.0f;
static float g_pan_cmd       = 0.0f;
static float g_tilt_cmd      = 0.0f;
static int   g_motor_abs_pan  = 0;   // IP의 static current_pan 기준 누적 목표
static int   g_motor_abs_tilt = 0;
static bool  g_had_target     = false; // 직전 프레임 표적 유무 (acquireRampScale 판별)
static int   g_host_cmd_cooldown = 0; // 명령 실행 보호 (IP 이동 완료 대기)
static int   g_ai_lock_frames    = 0; // AI 추적 모드 유지 — 레이더 오버라이드 차단
#define HOST_CMD_COOLDOWN_FRAMES  2   // 2프레임(66ms): 모터 이동 완료 대기
#define AI_LOCK_FRAMES            4   // 4프레임(132ms): C++ 다음 명령 도착 전까지 레이더 개입 방지

// ============================================================
//  레이더 방위각 → 픽셀 변환 (angle_to_px)
// ============================================================
static int angle_to_px(int16_t at_10)
{
    float deg = at_10 / 10.0f;
    float normalized_x = deg / (CAM_HFOV_DEG / 2.0f);
    int px = (int)((normalized_x + 1.0f) / 2.0f * CAM_W_PX);
    
    if(px < 0) px = 0; 
    if(px >= CAM_W_PX) px = CAM_W_PX - 1;
    return px;
}

// ============================================================
//  ULN2003 모터 제어 (HLS IP AXI4-Lite)
//  control.cpp ControlLoop / AxisController 로직 이식
// ============================================================

// PID → 슬루 → 스텝 변환 (단일 축)
// control.cpp AxisController::stepFromCommand + PIDController::update 이식
// ramp_scale: 1.0f 평상시, MOTOR_ACQUIRE_RAMP 첫 표적 획득 시 (control.cpp acquireRampScale)
static int motor_pid_step(
    float err_px, float frame_half,
    int deadband, float kp, float kd,
    int min_step, int max_step,
    float* prev_err, float* smooth_cmd,
    float ramp_scale)
{
    float ramp = MOTOR_CMD_RAMP * MOTOR_DT * ramp_scale;

    if (fabsf(err_px) <= (float)deadband) {
        // 데드밴드 내 → 명령을 0으로 서서히 감속 (control.cpp: no target branch)
        if      (*smooth_cmd >  ramp) *smooth_cmd -= ramp;
        else if (*smooth_cmd < -ramp) *smooth_cmd += ramp;
        else                          *smooth_cmd  = 0.0f;
        *prev_err = 0.0f;
        return 0;
    }

    float norm  = err_px / (frame_half > 0.1f ? frame_half : 0.1f);
    float deriv = (norm - *prev_err) / MOTOR_DT;
    *prev_err   = norm;

    float target = kp * norm + kd * deriv;
    if (target >  1.0f) target =  1.0f;
    if (target < -1.0f) target = -1.0f;

    // 슬루 리미트 (control.cpp slewLimit)
    float diff = target - *smooth_cmd;
    if      (diff >  ramp) *smooth_cmd += ramp;
    else if (diff < -ramp) *smooth_cmd -= ramp;
    else                   *smooth_cmd  = target;

    float cmd = *smooth_cmd;
    if (fabsf(cmd) < 1e-3f) return 0;

    // 최소 스텝 비율 보정 (control.cpp AxisController::normalizeCommandFloor)
    float span      = (float)(max_step - min_step);
    float min_ratio = (float)min_step / ((float)min_step + span);
    float abs_cmd   = fabsf(cmd) < min_ratio ? min_ratio : fabsf(cmd);

    int step = min_step + (int)(span * abs_cmd);
    return cmd > 0.0f ? step : -step;
}

// ============================================================
//  하이브리드 모터 제어
//    Pan  : 레이더 방위각 오차 → 기존 PID 경로
//    Tilt : 호스트 카메라 직접 스텝 (g_host_tilt_steps)
// ============================================================
static void motor_update_hybrid(int pan_err_x, bool has_target, int direct_tilt_steps)
{
    float ramp = (has_target && !g_had_target) ? MOTOR_ACQUIRE_RAMP : 1.0f;
    g_had_target = has_target;

    // Pan: 기존 PID (레이더 방위각 오차 기반) — IP 대기 중에도 slew 상태 진행
    int dpan = motor_pid_step((float)pan_err_x, (float)(CAM_W_PX / 2),
                              MOTOR_PAN_DEADBAND, MOTOR_KP, MOTOR_KD,
                              MOTOR_PAN_MIN_STEP, MOTOR_PAN_MAX_STEP,
                              &g_pan_prev_err, &g_pan_cmd, ramp);

    // Tilt: 호스트 직접 스텝 (MOTOR_TILT_MIN_STEP 미만은 데드밴드로 무시)
    int dtilt = 0;
    if (abs(direct_tilt_steps) >= MOTOR_TILT_MIN_STEP) {
        dtilt = direct_tilt_steps;
        if (dtilt >  MOTOR_TILT_MAX_STEP) dtilt =  MOTOR_TILT_MAX_STEP;
        if (dtilt < -MOTOR_TILT_MAX_STEP) dtilt = -MOTOR_TILT_MAX_STEP;
    }
    // 호스트 모드에서 tilt PID 상태 초기화 (모드 전환 시 글리치 방지)
    g_tilt_prev_err = 0.0f;
    g_tilt_cmd      = 0.0f;

    if (dpan == 0 && dtilt == 0) return;

    // IP 실행 중이면 누적하지 않음 — 완료 시 최신 1프레임치만 적용 (연쇄 지연 방지)
    if (!(MOTOR_RD(0x00) & AP_IDLE)) return;

    g_motor_abs_pan  += dpan;
    g_motor_abs_tilt += dtilt;

    MOTOR_WR(0x10, (u32)g_motor_abs_pan);
    MOTOR_WR(0x18, (u32)g_motor_abs_tilt);
    MOTOR_WR(0x20, MOTOR_SPEED_DELAY);
    MOTOR_WR(0x00, AP_START);
}

// ============================================================
//  AI 완전 제어 모드 — Pan+Tilt 모두 호스트(PC YOLO) 직접 스텝
//  레이더가 없거나 YOLO가 드론 추적 중일 때 사용
// ============================================================
static void motor_update_full_host(int direct_pan_steps, int direct_tilt_steps)
{
    g_had_target = true;

    int dpan = direct_pan_steps;
    int dtilt = direct_tilt_steps;

    // maxStep 클램프 (자동/수동 공통)
    if (dpan >  MOTOR_PAN_MAX_STEP)  dpan =  MOTOR_PAN_MAX_STEP;
    if (dpan < -MOTOR_PAN_MAX_STEP)  dpan = -MOTOR_PAN_MAX_STEP;
    if (dtilt >  MOTOR_TILT_MAX_STEP)  dtilt =  MOTOR_TILT_MAX_STEP;
    if (dtilt < -MOTOR_TILT_MAX_STEP)  dtilt = -MOTOR_TILT_MAX_STEP;

    if (!g_manual_mode) {
        // 자동 모드: minStep 미만 명령 무시 (미세 진동 방지)
        if (abs(dpan) < MOTOR_PAN_MIN_STEP) dpan = 0;
        if (abs(dtilt) < MOTOR_TILT_MIN_STEP) dtilt = 0;
    }
    // 수동 모드: minStep 제한 없음 → 1스텝(0.011°)부터 정밀 이동 가능

    // 모드 전환 시 PID 글리치 방지
    g_pan_prev_err  = 0.0f; g_pan_cmd  = 0.0f;
    g_tilt_prev_err = 0.0f; g_tilt_cmd = 0.0f;

    if (dpan == 0 && dtilt == 0) return;

    // IP 실행 중이면 누적하지 않음 — 완료 시 최신 1프레임치만 적용 (연쇄 지연 방지)
    if (!(MOTOR_RD(0x00) & AP_IDLE)) return;

    g_motor_abs_pan  += dpan;
    g_motor_abs_tilt += dtilt;

    MOTOR_WR(0x10, (u32)g_motor_abs_pan);
    MOTOR_WR(0x18, (u32)g_motor_abs_tilt);
    MOTOR_WR(0x20, MOTOR_SPEED_DELAY);
    MOTOR_WR(0x00, AP_START);
}

// ============================================================
//  AI 카메라 PID 모드 — bbox 픽셀 오차 → FPGA PID → Pan+Tilt 스텝
//  PC는 bbox 중심 오차(ex, ey)만 전송. PID/슬루/스텝 계산은 FPGA 전담.
//  ex: (center_x - frame_w/2) / (frame_w/2) * 160  (320px 기준 스케일)
//  ey: (center_y - frame_h/2) / (frame_h/2) * 120  (240px 기준 스케일)
// ============================================================
static void motor_update_ai_bbox(int bbox_ex, int bbox_ey)
{
    g_had_target = true;

    int dpan = motor_pid_step(
        (float)bbox_ex, (float)(CAM_W_PX / 2),
        MOTOR_PAN_DEADBAND,  MOTOR_KP, MOTOR_KD,
        MOTOR_PAN_MIN_STEP,  MOTOR_PAN_MAX_STEP,
        &g_pan_prev_err, &g_pan_cmd, 1.0f);

    int dtilt = motor_pid_step(
        (float)bbox_ey, (float)(CAM_H_PX / 2),
        MOTOR_TILT_DEADBAND, MOTOR_KP, MOTOR_KD,
        MOTOR_TILT_MIN_STEP, MOTOR_TILT_MAX_STEP,
        &g_tilt_prev_err, &g_tilt_cmd, 1.0f);

    if (dpan == 0 && dtilt == 0) return;
    if (!(MOTOR_RD(0x00) & AP_IDLE)) return;

    g_motor_abs_pan  += dpan;
    g_motor_abs_tilt += dtilt;

    MOTOR_WR(0x10, (u32)g_motor_abs_pan);
    MOTOR_WR(0x18, (u32)g_motor_abs_tilt);
    MOTOR_WR(0x20, MOTOR_SPEED_DELAY);
    MOTOR_WR(0x00, AP_START);
}

// ============================================================
//  MTI(영상 모션 감지) 서브시스템 제거됨 → legacy/mti_subsystem/ (2026-06-01)
//  사유: PC YOLO 경로와 중복, Mock 상태(죽은 코드). 부활법은 해당 폴더 README 참조.
// ============================================================

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
    static float rang_lp = 0.0f;   // rang 저역통과 필터 상태 (지터 억제)

    while(true){

        // 1. 레이더 UART 고속 수신 및 파싱 (지연 제거 완벽 적용)
        uart_accumulate();
        if(uart_parse(rtgt, &rvc)){
            radar_age = 0;
            if(rvc>0) {
                process_radar_target(rtgt[0], &rdist, &rang);
                // LP 필터 (α=0.4): 레이더 좌표 지터 억제, KD 항 진동 방지
                rang_lp = 0.4f * (float)rang + 0.6f * rang_lp;
                rang = (int16_t)rang_lp;
            }
        } else if (radar_age < RADAR_STALE_FRAMES) {
            radar_age++;
        } else {
            rvc = 0;
            rdist = 0;
            rang = 0;
            memset(rtgt, 0, sizeof(rtgt));
        }

        // 2. 칼만 필터 예측/보정
        kalman_ip_run(rtgt, ks);

        // 3. 모터 제어 (AI추적 > 레이더 단독 > 정지)
        host_accumulate();
        host_parse_commands();

        if (g_host_bbox_valid && g_host_cmd_cooldown == 0) {
            // AI 추적 모드: PC bbox 오차 → FPGA PID → Pan+Tilt
            motor_update_ai_bbox(g_host_bbox_ex, g_host_bbox_ey);
            g_host_cmd_cooldown = HOST_CMD_COOLDOWN_FRAMES;
            g_ai_lock_frames    = AI_LOCK_FRAMES;
        } else if (g_manual_mode &&
                   (g_host_pan_steps != 0 || g_host_tilt_steps != 0) &&
                   g_host_cmd_cooldown == 0) {
            // 수동 모드: PC 방향키 직접 스텝 (캘리브레이션/점검용)
            motor_update_full_host(g_host_pan_steps, g_host_tilt_steps);
            g_host_cmd_cooldown = HOST_CMD_COOLDOWN_FRAMES;
        } else if (g_host_cmd_cooldown > 0) {
            // 모터 이동 완료 대기 — 레이더 개입 없음
            g_host_cmd_cooldown--;
            if (g_ai_lock_frames > 0) g_ai_lock_frames--;
        } else if (g_ai_lock_frames > 0) {
            // AI 잠금 중 — 레이더 오버라이드 차단 (다음 bbox 도착 전 공백 보호)
            g_ai_lock_frames--;
        } else if (rvc > 0) {
            // 레이더 단독 모드: AI 잠금 완전 해제 후에만 진입
            motor_update_hybrid(angle_to_px(rang) - CAM_W_PX / 2, true, 0);
        } else {
            // 표적 없음
            g_had_target = false;
        }
        g_host_bbox_valid = false;
        g_host_pan_steps  = 0;
        g_host_tilt_steps = 0;

        // 💡 [아키텍트 패치] Python UI 실시간 트래킹을 위해 레이더 좌표는 '매 프레임' 즉각 전송!
        if(rvc>0 && (fid % RADAR_LOG_PERIOD_FRAMES) == 0){
            xil_printf("[RADAR] T0:(%d,%d)mm spd=%dcm/s | dist=%dmm ang=%d.%ddeg\r\n",
                       rtgt[0].x, rtgt[0].y, rtgt[0].speed, rdist, rang/10, abs(rang%10));
        }

        // 5. 콘솔 로그 출력 (누락 없이 전체 모듈 상태 보고)
#if LOG_LEVEL >= 2
        if(fid % 30 == 0){
            xil_printf("\r\n[Frame %4d]===========================\r\n", fid);
            xil_printf("  [UART ] bytes=%u packets=%u ring_ovf=%u hw_ovr=%u hw_err=0x%08x\r\n",
                       (unsigned)g_rx_bytes, (unsigned)g_rx_packets,
                       (unsigned)g_rx_ring_overflows, (unsigned)g_rx_hw_overruns,
                       (unsigned)g_rx_hw_errors);
            
            // 칼만 필터 로그
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
