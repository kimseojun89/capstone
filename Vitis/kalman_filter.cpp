/**
 * kalman_filter.cpp — 2D Kalman Filter IP (3 targets)
 * Target: xc7z020clg400-1 / 100MHz
 * Resource goal: BRAM 0 (all LUTRAM), DSP <= 24
 *
 * AXI4-Lite register map (ps_main.cpp KALMAN_WR/RD 기준):
 *   0x00        ap_ctrl  (AP_START bit0, AP_DONE bit1, AP_IDLE bit2)
 *   0x10        reset    (bit0=1: 내부 상태 초기화)
 *
 *   targets_in (write, 8byte/target):
 *   0x20/0x24   target[0] w0=(uint16)y<<16|(uint16)x, w1=valid<<16|(uint16)speed
 *   0x28/0x2C   target[1]
 *   0x30/0x34   target[2]
 *
 *   states_out (read, 32byte/target, 5 word 유효):
 *   0x80..0x90  state[0]: x(f32) y(f32) vx(f32) vy(f32) init(u32 bit0)
 *   0xA0..0xB0  state[1]
 *   0xC0..0xD0  state[2]
 */

#include <cstdint>
#include <cstring>
#include <cmath>

#ifdef __SYNTHESIS__
#include "ap_int.h"
#include "ap_fixed.h"
#else
#include <cstdio>
#endif

constexpr int MAX_TARGETS = 3;
constexpr int STATE_DIM   = 4;   /* x, y, vx, vy */
constexpr float DT        = 0.033f; /* 30Hz 루프 주기 */

/* Process noise diagonal */
static const float Q_DIAG[STATE_DIM] = {1.0f, 1.0f, 10.0f, 10.0f};
/* Measurement noise diagonal */
static const float R_DIAG[2] = {100.0f, 100.0f};

/* Per-target persistent state (diagonal P) */
static float st[MAX_TARGETS][STATE_DIM];
static float P[MAX_TARGETS][STATE_DIM];
static bool  st_init[MAX_TARGETS];
static int   miss_count[MAX_TARGETS];

// 인터페이스: ps_main.cpp 가 직접 쓰는 packed uint32_t 입력 / float-ptr 출력
// 입력 target 패킹 (ps_main.cpp kalman_write_target 기준):
//   tN_w0 = ((uint16_t)(int16_t)y << 16) | (uint16_t)(int16_t)x
//   tN_w1 = ((valid ? 1u : 0u) << 16)    | (uint16_t)(int16_t)speed
void kalman_filter(
    /* reset: 0x10 */
    bool     reset,
    /* target inputs: 0x20~0x34 */
    uint32_t t0_w0,  uint32_t t0_w1,
    uint32_t t1_w0,  uint32_t t1_w1,
    uint32_t t2_w0,  uint32_t t2_w1,
    /* state[0] outputs: 0x80~0x90 */
    float   *s0x,    float *s0y,  float *s0vx,  float *s0vy,  uint32_t *s0init,
    /* state[1] outputs: 0xA0~0xB0 */
    float   *s1x,    float *s1y,  float *s1vx,  float *s1vy,  uint32_t *s1init,
    /* state[2] outputs: 0xC0~0xD0 */
    float   *s2x,    float *s2y,  float *s2vx,  float *s2vy,  uint32_t *s2init
)
{
#ifdef __SYNTHESIS__
/* AXI4-Lite 오프셋: ps_main.cpp KALMAN_WR(0x10/0x20~0x34) · KALMAN_RD(0x80~0xD0) 와 일치 */
#pragma HLS INTERFACE s_axilite port=return  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=reset   bundle=CTRL offset=0x10

#pragma HLS INTERFACE s_axilite port=t0_w0   bundle=CTRL offset=0x20
#pragma HLS INTERFACE s_axilite port=t0_w1   bundle=CTRL offset=0x24
#pragma HLS INTERFACE s_axilite port=t1_w0   bundle=CTRL offset=0x28
#pragma HLS INTERFACE s_axilite port=t1_w1   bundle=CTRL offset=0x2C
#pragma HLS INTERFACE s_axilite port=t2_w0   bundle=CTRL offset=0x30
#pragma HLS INTERFACE s_axilite port=t2_w1   bundle=CTRL offset=0x34

#pragma HLS INTERFACE s_axilite port=s0x     bundle=CTRL offset=0x80
#pragma HLS INTERFACE s_axilite port=s0y     bundle=CTRL offset=0x84
#pragma HLS INTERFACE s_axilite port=s0vx    bundle=CTRL offset=0x88
#pragma HLS INTERFACE s_axilite port=s0vy    bundle=CTRL offset=0x8C
#pragma HLS INTERFACE s_axilite port=s0init  bundle=CTRL offset=0x90

#pragma HLS INTERFACE s_axilite port=s1x     bundle=CTRL offset=0xA0
#pragma HLS INTERFACE s_axilite port=s1y     bundle=CTRL offset=0xA4
#pragma HLS INTERFACE s_axilite port=s1vx    bundle=CTRL offset=0xA8
#pragma HLS INTERFACE s_axilite port=s1vy    bundle=CTRL offset=0xAC
#pragma HLS INTERFACE s_axilite port=s1init  bundle=CTRL offset=0xB0

#pragma HLS INTERFACE s_axilite port=s2x     bundle=CTRL offset=0xC0
#pragma HLS INTERFACE s_axilite port=s2y     bundle=CTRL offset=0xC4
#pragma HLS INTERFACE s_axilite port=s2vx    bundle=CTRL offset=0xC8
#pragma HLS INTERFACE s_axilite port=s2vy    bundle=CTRL offset=0xCC
#pragma HLS INTERFACE s_axilite port=s2init  bundle=CTRL offset=0xD0

#pragma HLS BIND_STORAGE variable=Q_DIAG     type=ROM_1P impl=LUTRAM
#pragma HLS BIND_STORAGE variable=R_DIAG     type=ROM_1P impl=LUTRAM
#pragma HLS BIND_STORAGE variable=st         type=RAM_2P impl=LUTRAM
#pragma HLS BIND_STORAGE variable=P          type=RAM_2P impl=LUTRAM
#pragma HLS BIND_STORAGE variable=st_init    type=RAM_2P impl=LUTRAM
#pragma HLS BIND_STORAGE variable=miss_count type=RAM_2P impl=LUTRAM
#endif

    /* ---- packed word 언팩: ps_main.cpp kalman_write_target 역연산 ---- */
    /* w0 = (uint16)y<<16 | (uint16)x,  w1 = valid<<16 | (uint16)speed  */
    const uint32_t raw_w0[MAX_TARGETS] = {t0_w0, t1_w0, t2_w0};
    const uint32_t raw_w1[MAX_TARGETS] = {t0_w1, t1_w1, t2_w1};

    int16_t  tgt_x[MAX_TARGETS], tgt_y[MAX_TARGETS], tgt_spd[MAX_TARGETS];
    bool     tgt_valid[MAX_TARGETS];

    for (int n = 0; n < MAX_TARGETS; ++n) {
#ifdef __SYNTHESIS__
#pragma HLS LOOP_TRIPCOUNT min=3 max=3
#endif
        tgt_x[n]     = (int16_t)( raw_w0[n]        & 0xFFFFu);
        tgt_y[n]     = (int16_t)((raw_w0[n] >> 16) & 0xFFFFu);
        tgt_spd[n]   = (int16_t)( raw_w1[n]        & 0xFFFFu);
        tgt_valid[n] = ((raw_w1[n] >> 16) & 0x1u) != 0u;
    }

    /* ---- reset ---- */
    if (reset) {
    RESET_LOOP:
        for (int i = 0; i < MAX_TARGETS; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS LOOP_TRIPCOUNT min=3 max=3
#endif
            for (int j = 0; j < STATE_DIM; ++j) {
                st[i][j] = 0.0f;
                P[i][j]  = 1000.0f;
            }
            st_init[i]    = false;
            miss_count[i] = 0;
        }
    }

    /* ---- Kalman predict + update (3 targets 순차 처리) ---- */
    float out_x[MAX_TARGETS], out_y[MAX_TARGETS];
    float out_vx[MAX_TARGETS], out_vy[MAX_TARGETS];
    uint32_t out_init[MAX_TARGETS];

TARGET_LOOP:
    for (int t = 0; t < MAX_TARGETS; ++t) {
#ifdef __SYNTHESIS__
#pragma HLS LOOP_TRIPCOUNT min=3 max=3
/*
 * 파이프라인하지 않음: st/P 는 read-modify-write 경로를 가지며,
 * MAX_TARGETS=3이므로 순차 스케줄이 100MHz 제약 충족에 더 안전함.
 */
#endif
        const bool valid = tgt_valid[t];

        float sx  = st[t][0], sy  = st[t][1];
        float svx = st[t][2], svy = st[t][3];
        float p0  = P[t][0],  p1  = P[t][1];
        float p2  = P[t][2],  p3  = P[t][3];
        bool  init = st_init[t];
        int   miss = miss_count[t];

        if (!init && valid) {
            /* 최초 측정: 초기화 */
            sx = (float)tgt_x[t];  sy = (float)tgt_y[t];
            svx = 0.0f;            svy = 0.0f;
            p0 = p1 = p2 = p3 = 1000.0f;
            init = true;
            miss = 0;
        } else if (init) {
            /* === PREDICT === */
            sx += DT * svx;
            sy += DT * svy;

            const float dt2   = DT * DT;
            p0 = p0 + Q_DIAG[0] + dt2 * p2;
            p1 = p1 + Q_DIAG[1] + dt2 * p3;
            p2 = p2 + Q_DIAG[2];
            p3 = p3 + Q_DIAG[3];

            if (valid) {
                /* === UPDATE === */
                const float zx = (float)tgt_x[t];
                const float zy = (float)tgt_y[t];

                const float s0_inv = (p0 + R_DIAG[0] > 1e-6f) ? 1.0f / (p0 + R_DIAG[0]) : 0.0f;
                const float s1_inv = (p1 + R_DIAG[1] > 1e-6f) ? 1.0f / (p1 + R_DIAG[1]) : 0.0f;

                const float k0   = p0  * s0_inv;
                const float k1   = p1  * s1_inv;
                const float k_vx = DT * p2 * s0_inv;
                const float k_vy = DT * p3 * s1_inv;

                const float innov_x = zx - sx;
                const float innov_y = zy - sy;

                sx  += k0   * innov_x;
                sy  += k1   * innov_y;
                svx += k_vx * innov_x;
                svy += k_vy * innov_y;

                p0 *= (1.0f - k0);
                p1 *= (1.0f - k1);
                p2 *= (1.0f - k_vx * DT);
                p3 *= (1.0f - k_vy * DT);

                miss = 0;
            } else {
                /* 측정 없음: 예측값만 유지, 1프레임 후 무효화 */
                miss++;
                if (miss > 1) init = false;
            }
        }

        /* write-back */
        st[t][0] = sx;  st[t][1] = sy;
        st[t][2] = svx; st[t][3] = svy;
        P[t][0]  = p0;  P[t][1]  = p1;
        P[t][2]  = p2;  P[t][3]  = p3;
        st_init[t]    = init;
        miss_count[t] = miss;

        out_x[t]    = sx;  out_y[t]  = sy;
        out_vx[t]   = svx; out_vy[t] = svy;
        out_init[t] = init ? 1u : 0u;
    }

    /* ---- AXI 출력 레지스터에 기록 (ps_main.cpp kalman_read_state 와 일치) ---- */
    *s0x = out_x[0];  *s0y = out_y[0];  *s0vx = out_vx[0];  *s0vy = out_vy[0];  *s0init = out_init[0];
    *s1x = out_x[1];  *s1y = out_y[1];  *s1vx = out_vx[1];  *s1vy = out_vy[1];  *s1init = out_init[1];
    *s2x = out_x[2];  *s2y = out_y[2];  *s2vx = out_vx[2];  *s2vy = out_vy[2];  *s2init = out_init[2];
}

/* ===== Test Harness ===== */
#ifndef __SYNTHESIS__
static uint32_t pack_w0(int16_t x, int16_t y) {
    return ((uint32_t)(uint16_t)y << 16) | (uint32_t)(uint16_t)x;
}
static uint32_t pack_w1(int16_t speed, bool valid) {
    return ((uint32_t)(valid ? 1u : 0u) << 16) | (uint32_t)(uint16_t)speed;
}

static void print_states(const char* label,
    float x0, float y0, float vx0, float vy0, uint32_t i0,
    float x1, float y1, float vx1, float vy1, uint32_t i1,
    float x2, float y2, float vx2, float vy2, uint32_t i2)
{
    printf("[%s]\n", label);
    if (i0) printf("  T0: (%.1f, %.1f) v=(%.2f, %.2f)\n", x0, y0, vx0, vy0);
    else    printf("  T0: --- inactive ---\n");
    if (i1) printf("  T1: (%.1f, %.1f) v=(%.2f, %.2f)\n", x1, y1, vx1, vy1);
    else    printf("  T1: --- inactive ---\n");
    if (i2) printf("  T2: (%.1f, %.1f) v=(%.2f, %.2f)\n", x2, y2, vx2, vy2);
    else    printf("  T2: --- inactive ---\n");
}

int main()
{
    float x0,y0,vx0,vy0; uint32_t i0;
    float x1,y1,vx1,vy1; uint32_t i1;
    float x2,y2,vx2,vy2; uint32_t i2;

    /* Reset */
    kalman_filter(true,
        0,0, 0,0, 0,0,
        &x0,&y0,&vx0,&vy0,&i0,
        &x1,&y1,&vx1,&vy1,&i1,
        &x2,&y2,&vx2,&vy2,&i2);

    /* --- Scenario 1: Single target convergence --- */
    printf("=== Scenario 1: Single Drone ===\n");
    uint32_t a0w0 = pack_w0(1713, -782), a0w1 = pack_w1(-16, true);
    uint32_t nw0  = pack_w0(0, 0),       nw1  = pack_w1(0, false);

    for (int step = 0; step < 5; ++step) {
        kalman_filter(false, a0w0,a0w1, nw0,nw1, nw0,nw1,
            &x0,&y0,&vx0,&vy0,&i0, &x1,&y1,&vx1,&vy1,&i1, &x2,&y2,&vx2,&vy2,&i2);
        char lbl[32]; snprintf(lbl, 32, "Step %d", step);
        print_states(lbl, x0,y0,vx0,vy0,i0, x1,y1,vx1,vy1,i1, x2,y2,vx2,vy2,i2);
    }
    float err_x = fabsf(x0 - 1713.0f), err_y = fabsf(y0 - (-782.0f));
    printf("Convergence: err_x=%.1f err_y=%.1f %s\n\n",
           err_x, err_y, (err_x < 50 && err_y < 50) ? "PASS" : "FAIL");

    /* --- Scenario 2: Two targets --- */
    printf("=== Scenario 2: Drone + Bird ===\n");
    kalman_filter(true, 0,0,0,0,0,0,
        &x0,&y0,&vx0,&vy0,&i0, &x1,&y1,&vx1,&vy1,&i1, &x2,&y2,&vx2,&vy2,&i2);
    uint32_t b0w0 = pack_w0(1713,-782), b0w1 = pack_w1(-16,true);
    uint32_t b1w0 = pack_w0(930, 935),  b1w1 = pack_w1(8,  true);
    for (int step = 0; step < 5; ++step)
        kalman_filter(false, b0w0,b0w1, b1w0,b1w1, nw0,nw1,
            &x0,&y0,&vx0,&vy0,&i0, &x1,&y1,&vx1,&vy1,&i1, &x2,&y2,&vx2,&vy2,&i2);
    print_states("After 5 steps", x0,y0,vx0,vy0,i0, x1,y1,vx1,vy1,i1, x2,y2,vx2,vy2,i2);
    bool sep = (fabsf(x0 - x1) > 500);
    printf("Separation: %s\n\n", sep ? "PASS" : "FAIL");

    /* --- Scenario 3: Target loss --- */
    printf("=== Scenario 3: Target Loss ===\n");
    kalman_filter(true, 0,0,0,0,0,0,
        &x0,&y0,&vx0,&vy0,&i0, &x1,&y1,&vx1,&vy1,&i1, &x2,&y2,&vx2,&vy2,&i2);
    for (int step = 0; step < 3; ++step)
        kalman_filter(false, b0w0,b0w1, b1w0,b1w1, nw0,nw1,
            &x0,&y0,&vx0,&vy0,&i0, &x1,&y1,&vx1,&vy1,&i1, &x2,&y2,&vx2,&vy2,&i2);
    print_states("Before loss", x0,y0,vx0,vy0,i0, x1,y1,vx1,vy1,i1, x2,y2,vx2,vy2,i2);

    /* target 1 소실 */
    uint32_t b1nw1 = pack_w1(0, false);
    kalman_filter(false, b0w0,b0w1, b1w0,b1nw1, nw0,nw1,
        &x0,&y0,&vx0,&vy0,&i0, &x1,&y1,&vx1,&vy1,&i1, &x2,&y2,&vx2,&vy2,&i2);
    print_states("1 frame coast", x0,y0,vx0,vy0,i0, x1,y1,vx1,vy1,i1, x2,y2,vx2,vy2,i2);
    bool coast = (i1 != 0);

    kalman_filter(false, b0w0,b0w1, b1w0,b1nw1, nw0,nw1,
        &x0,&y0,&vx0,&vy0,&i0, &x1,&y1,&vx1,&vy1,&i1, &x2,&y2,&vx2,&vy2,&i2);
    print_states("2 frames: invalidated", x0,y0,vx0,vy0,i0, x1,y1,vx1,vy1,i1, x2,y2,vx2,vy2,i2);
    bool inval = (i1 == 0);
    printf("Coast+Invalidate: %s\n", (coast && inval) ? "PASS" : "FAIL");

    return 0;
}
#endif
