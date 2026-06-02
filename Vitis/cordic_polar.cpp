/**
 * cordic_polar.cpp — CORDIC Vectoring Mode IP
 * Target: xc7z020clg400-1 / 100MHz
 * Resource goal: BRAM 0, DSP <= 8
 *
 * AXI4-Lite register map (ps_main.cpp CORDIC_WR/RD 기준):
 *   0x00  ap_ctrl  (AP_START bit0, AP_DONE bit1, AP_IDLE bit2)
 *   0x10  x_in     (int16_t → u32 캐스트, write)
 *   0x18  y_in     (int16_t → u32 캐스트, write)
 *   0x20  distance (uint16_t 출력, read [15:0])
 *   0x30  angle_deg(int16_t 출력, read [15:0] → int16_t 재해석)
 */

#include <cstdint>

#ifdef __SYNTHESIS__
  #include "ap_int.h"
  #include "ap_fixed.h"
#else
  #include <cstdio>
  #include <cstdlib>
  #include <cmath>
#endif

constexpr int CORDIC_ITER = 16;

static const int32_t ATAN_TABLE_TENTHS[CORDIC_ITER] = {
    450, 265, 140, 71, 36, 18, 9, 4, 2, 1, 1, 0, 0, 0, 0, 0
};

static const int32_t K_INV_FP16 = 39797; /* 0.60725 * 65536 */

void cordic_polar(
    int16_t   x_in,
    int16_t   y_in,
    uint16_t* distance,
    int16_t*  angle_deg)
{
#ifdef __SYNTHESIS__
/* AXI4-Lite 오프셋: ps_main.cpp CORDIC_WR(0x10/0x18) · CORDIC_RD(0x20/0x30) 와 일치 */
#pragma HLS INTERFACE s_axilite port=x_in      bundle=CTRL offset=0x10
#pragma HLS INTERFACE s_axilite port=y_in      bundle=CTRL offset=0x18
#pragma HLS INTERFACE s_axilite port=distance  bundle=CTRL offset=0x20
#pragma HLS INTERFACE s_axilite port=angle_deg bundle=CTRL offset=0x30
#pragma HLS INTERFACE s_axilite port=return    bundle=CTRL
#pragma HLS PIPELINE II=1
#pragma HLS BIND_STORAGE variable=ATAN_TABLE_TENTHS type=ROM_1P impl=LUTRAM
#endif

    int32_t x = (int32_t)x_in;
    int32_t y = (int32_t)y_in;
    int32_t z = 0;

    /* Pre-rotate: ensure x >= 0 for CORDIC convergence */
    int angle_offset = 0;
    if (x < 0) {
        angle_offset = (y >= 0) ? 1800 : -1800;
        x = -x;
        y = -y;
    }

    /* Vectoring: drive y → 0 */
    CORDIC_LOOP:
    for (int i = 0; i < CORDIC_ITER; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS LOOP_TRIPCOUNT min=16 max=16
#pragma HLS UNROLL
#endif
        int32_t x_s = x >> i;
        int32_t y_s = y >> i;
        if (y < 0) {
            x = x - y_s;
            y = y + x_s;
            z = z - ATAN_TABLE_TENTHS[i];
        } else {
            x = x + y_s;
            y = y - x_s;
            z = z + ATAN_TABLE_TENTHS[i];
        }
    }

    /* Distance: x_final * K_inv (FP16 스케일) */
    int64_t d64 = (int64_t)x * K_INV_FP16;
    int32_t dist = (int32_t)(d64 >> 16);
    if (dist < 0) dist = -dist;
    *distance = (uint16_t)((dist > 65535) ? 65535 : dist);

    /* Angle: 0.1도 단위, Q2/Q3 오프셋 보정 후 [-1800, +1800] 클램프 */
    int32_t final_angle = z + angle_offset;
    if (final_angle > 1800) final_angle -= 3600;
    if (final_angle < -1800) final_angle += 3600;
    *angle_deg = (int16_t)final_angle;
}

#ifndef __SYNTHESIS__
int main()
{
    struct TC { int16_t x; int16_t y; int exp_d; int exp_a; int d_tol; int a_tol; };
    TC cases[] = {
        {1713, -782, 1886, -245, 50, 15},
        {0,    1000, 1000,  900, 50, 10},
        {1000, 0,    1000,    0, 50, 10},
        {707,  707,  1000,  450, 50, 15},
        {-1000,0,    1000, -1799, 50, 15},
    };
    int n = 5, pass = 0;
    for (int t = 0; t < n; ++t) {
        uint16_t dist; int16_t ang;
        cordic_polar(cases[t].x, cases[t].y, &dist, &ang);
        int d_err = abs((int)dist - cases[t].exp_d);
        int a_err = abs((int)ang  - cases[t].exp_a);
        bool ok = (d_err <= cases[t].d_tol) && (a_err <= cases[t].a_tol);
        printf("TC%d: in(%d,%d) dist=%u(exp%d err%d) ang=%d(exp%d err%d) %s\n",
               t, cases[t].x, cases[t].y,
               dist, cases[t].exp_d, d_err,
               ang,  cases[t].exp_a, a_err,
               ok ? "PASS" : "FAIL");
        if (ok) pass++;
    }
    printf("\nResult: %d/%d passed\n", pass, n);
    return (pass == n) ? 0 : 1;
}
#endif
