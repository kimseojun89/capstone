/**
 * kalman_filter.cpp — 2D Kalman Filter IP (3 targets)
 * Target: xc7z020clg400-1 / 100MHz
 * Resource goal: BRAM 0 (all LUTRAM), DSP <= 24
 */

#include <cstdint>
#include <cstdio>
#include <cmath>
#include <cstring>

#ifdef __SYNTHESIS__
#include "ap_int.h"
#include "ap_fixed.h"
#endif

constexpr int MAX_TARGETS = 3;
constexpr int STATE_DIM = 4; /* x, y, vx, vy */
constexpr int MEAS_DIM = 2;  /* x, y */
constexpr float DT = 0.033f; /* ~30fps loop period (was 0.1f @ 10Hz) */

/* Process noise diagonal */
static const float Q_DIAG[STATE_DIM] = {1.0f, 1.0f, 10.0f, 10.0f};

/* Measurement noise diagonal */
static const float R_DIAG[MEAS_DIM] = {100.0f, 100.0f};

struct RadarTarget
{
    int16_t x, y;
    int16_t speed;
    bool valid;
};

struct KalmanState
{
    float x, y, vx, vy;
    bool initialized;
};

/* Per-target persistent state (diagonal P) */
static float st[MAX_TARGETS][STATE_DIM]; /* state vector */
static float P[MAX_TARGETS][STATE_DIM];  /* diagonal covariance */
static bool st_init[MAX_TARGETS];
static int miss_count[MAX_TARGETS];

void kalman_filter(
    const RadarTarget targets_in[MAX_TARGETS],
    KalmanState states_out[MAX_TARGETS],
    bool reset)
{
#ifdef __SYNTHESIS__
/* 인터페이스 프라그마 */
#pragma HLS INTERFACE s_axilite port = targets_in
#pragma HLS INTERFACE s_axilite port = states_out
#pragma HLS INTERFACE s_axilite port = reset
#pragma HLS INTERFACE s_axilite port = return

/* 스토리지 바인딩 프라그마 (모두 함수 내부로 이동 완료) */
#pragma HLS BIND_STORAGE variable = Q_DIAG type = ROM_1P impl = LUTRAM
#pragma HLS BIND_STORAGE variable = R_DIAG type = ROM_1P impl = LUTRAM
#pragma HLS BIND_STORAGE variable = st type = RAM_2P impl = LUTRAM
#pragma HLS BIND_STORAGE variable = P type = RAM_2P impl = LUTRAM
#pragma HLS BIND_STORAGE variable = st_init type = RAM_2P impl = LUTRAM
#pragma HLS BIND_STORAGE variable = miss_count type = RAM_2P impl = LUTRAM
#endif

    if (reset)
    {
    RESET_LOOP:
        for (int i = 0; i < MAX_TARGETS; ++i)
        {
#ifdef __SYNTHESIS__
#pragma HLS LOOP_TRIPCOUNT min = 3 max = 3
#endif
            for (int j = 0; j < STATE_DIM; ++j)
            {
                st[i][j] = 0.0f;
                P[i][j] = 1000.0f;
            }
            st_init[i] = false;
            miss_count[i] = 0;
        }
    }

TARGET_LOOP:
    for (int t = 0; t < MAX_TARGETS; ++t)
    {
#ifdef __SYNTHESIS__
#pragma HLS LOOP_TRIPCOUNT min = 3 max = 3
/*
 * Do not pipeline this loop: st/P are persistent state variables and each
 * target update has a true read-modify-write path. Pipelining this loop makes
 * Vitis try to start the next iteration before the previous write-back is done,
 * which causes HLS 200-880 carried-dependence warnings. MAX_TARGETS is only 3,
 * so sequential scheduling is safer and lets FP add/div operations use
 * multi-cycle schedules to meet 100 MHz.
 */
#endif
        const bool valid = targets_in[t].valid;

        /*
         * Use local scalar copies. This reduces repeated RAM reads/writes and,
         * when TARGET_LOOP is not pipelined, lets HLS schedule the floating-point
         * add/mul/div chain over multiple cycles instead of forcing an II=1
         * read-modify-write recurrence.
         */
        float sx = st[t][0];
        float sy = st[t][1];
        float svx = st[t][2];
        float svy = st[t][3];

        float p0 = P[t][0];
        float p1 = P[t][1];
        float p2 = P[t][2];
        float p3 = P[t][3];

        bool init = st_init[t];
        int miss = miss_count[t];

        if (!init && valid)
        {
            /* First measurement: initialize */
            sx = (float)targets_in[t].x;
            sy = (float)targets_in[t].y;
            svx = 0.0f;
            svy = 0.0f;

            p0 = 1000.0f;
            p1 = 1000.0f;
            p2 = 1000.0f;
            p3 = 1000.0f;

            init = true;
            miss = 0;
        }
        else if (init)
        {
            /* === PREDICT === */
            sx += DT * svx;
            sy += DT * svy;

            /* P predict (diagonal approx). Split the add chain to avoid
             * p0 = p0 + a + b being scheduled as two fadds in one cycle. */
            const float dt2 = DT * DT;
            const float p0_inc = dt2 * p2;
            const float p1_inc = dt2 * p3;
            const float p0_q = p0 + Q_DIAG[0];
            const float p1_q = p1 + Q_DIAG[1];

            p0 = p0_q + p0_inc;
            p1 = p1_q + p1_inc;
            p2 = p2 + Q_DIAG[2];
            p3 = p3 + Q_DIAG[3];

            if (valid)
            {
                /* === UPDATE === */
                const float zx = (float)targets_in[t].x;
                const float zy = (float)targets_in[t].y;

                /* Kalman gain (scalar, per axis) */
                const float s0 = p0 + R_DIAG[0];
                const float s1 = p1 + R_DIAG[1];
                const float k0 = (s0 > 1e-6f) ? p0 / s0 : 0.0f;
                const float k1 = (s1 > 1e-6f) ? p1 / s1 : 0.0f;

                /* Also update velocity via cross-covariance approx:
                   k_vx = dt * p_vx / s0 */
                const float k_vx = (s0 > 1e-6f) ? DT * p2 / s0 : 0.0f;
                const float k_vy = (s1 > 1e-6f) ? DT * p3 / s1 : 0.0f;

                const float innov_x = zx - sx;
                const float innov_y = zy - sy;

                sx = sx + k0 * innov_x;
                sy = sy + k1 * innov_y;
                svx = svx + k_vx * innov_x;
                svy = svy + k_vy * innov_y;

                p0 = p0 * (1.0f - k0);
                p1 = p1 * (1.0f - k1);
                /* Simplified P update for velocity */
                p2 = p2 * (1.0f - k_vx * DT);
                p3 = p3 * (1.0f - k_vy * DT);

                miss = 0;
            }
            else
            {
                /* No measurement: prediction only */
                miss++;
                if (miss > 1)
                {
                    init = false; /* invalidate after 1 frame coast */
                }
            }
        }

        /* Write-back */
        st[t][0] = sx;
        st[t][1] = sy;
        st[t][2] = svx;
        st[t][3] = svy;

        P[t][0] = p0;
        P[t][1] = p1;
        P[t][2] = p2;
        P[t][3] = p3;

        st_init[t] = init;
        miss_count[t] = miss;

        /* Output */
        states_out[t].x = sx;
        states_out[t].y = sy;
        states_out[t].vx = svx;
        states_out[t].vy = svy;
        states_out[t].initialized = init;
    }
}

/* ===== Test Harness ===== */
#ifndef __SYNTHESIS__
static void print_states(const char *label, KalmanState s[3])
{
    printf("[%s]\n", label);
    for (int i = 0; i < 3; ++i)
    {
        if (s[i].initialized)
            printf("  T%d: (%.1f, %.1f) v=(%.2f, %.2f)\n",
                   i, s[i].x, s[i].y, s[i].vx, s[i].vy);
        else
            printf("  T%d: --- inactive ---\n", i);
    }
}

int main()
{
    KalmanState out[3];
    RadarTarget inp[3];

    /* Reset */
    kalman_filter(inp, out, true);

    /* --- Scenario 1: Single target convergence --- */
    printf("=== Scenario 1: Single Drone ===\n");
    inp[0] = {1713, -782, -16, true};
    inp[1] = {0, 0, 0, false};
    inp[2] = {0, 0, 0, false};

    for (int step = 0; step < 5; ++step)
    {
        kalman_filter(inp, out, false);
        char lbl[32];
        snprintf(lbl, 32, "Step %d", step);
        print_states(lbl, out);
    }
    float err_x = fabsf(out[0].x - 1713.0f);
    float err_y = fabsf(out[0].y - (-782.0f));
    printf("Convergence: err_x=%.1f err_y=%.1f %s\n\n",
           err_x, err_y, (err_x < 50 && err_y < 50) ? "PASS" : "FAIL");

    /* --- Scenario 2: Two targets --- */
    printf("=== Scenario 2: Drone + Bird ===\n");
    kalman_filter(inp, out, true); /* reset */
    inp[0] = {1713, -782, -16, true};
    inp[1] = {930, 935, 8, true};
    inp[2] = {0, 0, 0, false};
    for (int step = 0; step < 5; ++step)
    {
        kalman_filter(inp, out, false);
    }
    print_states("After 5 steps", out);
    bool sep = (fabsf(out[0].x - out[1].x) > 500);
    printf("Separation: %s\n\n", sep ? "PASS" : "FAIL");

    /* --- Scenario 3: Target loss --- */
    printf("=== Scenario 3: Target Loss ===\n");
    kalman_filter(inp, out, true);
    inp[0] = {1713, -782, -16, true};
    inp[1] = {930, 935, 8, true};
    for (int step = 0; step < 3; ++step)
        kalman_filter(inp, out, false);
    print_states("Before loss", out);

    /* Now target 1 disappears */
    inp[1].valid = false;
    kalman_filter(inp, out, false);
    print_states("1 frame coast", out);
    bool coast = out[1].initialized;

    kalman_filter(inp, out, false);
    print_states("2 frames: invalidated", out);
    bool inval = !out[1].initialized;
    printf("Coast+Invalidate: %s\n", (coast && inval) ? "PASS" : "FAIL");

    return 0;
}
#endif
