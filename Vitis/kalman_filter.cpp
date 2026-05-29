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
constexpr int STATE_DIM   = 4;  /* x, y, vx, vy */
constexpr int MEAS_DIM    = 2;  /* x, y */
constexpr float DT        = 0.033f;  /* ~30fps loop period (was 0.1f @ 10Hz) */

/* Process noise diagonal */
static const float Q_DIAG[STATE_DIM] = {1.0f, 1.0f, 10.0f, 10.0f};

/* Measurement noise diagonal */
static const float R_DIAG[MEAS_DIM] = {100.0f, 100.0f};

struct RadarTarget {
    int16_t x, y;
    int16_t speed;
    bool    valid;
};

struct KalmanState {
    float x, y, vx, vy;
    bool  initialized;
};

/* Per-target persistent state (diagonal P) */
static float st[MAX_TARGETS][STATE_DIM];     /* state vector */
static float P[MAX_TARGETS][STATE_DIM];      /* diagonal covariance */
static bool  st_init[MAX_TARGETS];
static int   miss_count[MAX_TARGETS];


void kalman_filter(
    const RadarTarget targets_in[MAX_TARGETS],
    KalmanState       states_out[MAX_TARGETS],
    bool              reset)
{
#ifdef __SYNTHESIS__
/* 인터페이스 프라그마 */
#pragma HLS INTERFACE s_axilite port=targets_in
#pragma HLS INTERFACE s_axilite port=states_out
#pragma HLS INTERFACE s_axilite port=reset
#pragma HLS INTERFACE s_axilite port=return

/* 스토리지 바인딩 프라그마 (모두 함수 내부로 이동 완료) */
#pragma HLS BIND_STORAGE variable=Q_DIAG type=ROM_1P impl=LUTRAM
#pragma HLS BIND_STORAGE variable=R_DIAG type=ROM_1P impl=LUTRAM
#pragma HLS BIND_STORAGE variable=st         type=RAM_2P impl=LUTRAM
#pragma HLS BIND_STORAGE variable=P          type=RAM_2P impl=LUTRAM
#pragma HLS BIND_STORAGE variable=st_init    type=RAM_2P impl=LUTRAM
#pragma HLS BIND_STORAGE variable=miss_count type=RAM_2P impl=LUTRAM
#endif

    if (reset) {
        RESET_LOOP:
        for (int i = 0; i < MAX_TARGETS; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS LOOP_TRIPCOUNT min=3 max=3
#pragma HLS UNROLL
#endif
            for (int j = 0; j < STATE_DIM; ++j) {
#ifdef __SYNTHESIS__
#pragma HLS UNROLL
#endif
                st[i][j] = 0.0f;
                P[i][j]  = 1000.0f;
            }
            st_init[i]    = false;
            miss_count[i]  = 0;
        }
    }

    TARGET_LOOP:
    for (int t = 0; t < MAX_TARGETS; ++t) {
#ifdef __SYNTHESIS__
#pragma HLS LOOP_TRIPCOUNT min=3 max=3
#pragma HLS PIPELINE
#endif
        bool valid = targets_in[t].valid;

        if (!st_init[t] && valid) {
            /* First measurement: initialize */
            st[t][0] = (float)targets_in[t].x;
            st[t][1] = (float)targets_in[t].y;
            st[t][2] = 0.0f;
            st[t][3] = 0.0f;
            for (int j = 0; j < STATE_DIM; ++j) {
#ifdef __SYNTHESIS__
#pragma HLS UNROLL
#endif
                P[t][j] = 1000.0f;
            }
            st_init[t]   = true;
            miss_count[t] = 0;
        } else if (st_init[t]) {
            /* === PREDICT === */
            st[t][0] += DT * st[t][2];
            st[t][1] += DT * st[t][3];

            /* P predict (diagonal approx):
               p_x  += dt^2 * p_vx + q_x
               p_vx += q_vx          */
            float dt2 = DT * DT;
            P[t][0] += dt2 * P[t][2] + Q_DIAG[0];
            P[t][1] += dt2 * P[t][3] + Q_DIAG[1];
            P[t][2] += Q_DIAG[2];
            P[t][3] += Q_DIAG[3];

            if (valid) {
                /* === UPDATE === */
                float zx = (float)targets_in[t].x;
                float zy = (float)targets_in[t].y;

                /* Kalman gain (scalar, per axis) */
                float s0 = P[t][0] + R_DIAG[0];
                float s1 = P[t][1] + R_DIAG[1];
                float k0 = (s0 > 1e-6f) ? P[t][0] / s0 : 0.0f;
                float k1 = (s1 > 1e-6f) ? P[t][1] / s1 : 0.0f;

                /* Also update velocity via cross-covariance approx:
                   k_vx = dt * p_vx / s0 */
                float k_vx = (s0 > 1e-6f) ? DT * P[t][2] / s0 : 0.0f;
                float k_vy = (s1 > 1e-6f) ? DT * P[t][3] / s1 : 0.0f;

                float innov_x = zx - st[t][0];
                float innov_y = zy - st[t][1];

                st[t][0] += k0   * innov_x;
                st[t][1] += k1   * innov_y;
                st[t][2] += k_vx * innov_x;
                st[t][3] += k_vy * innov_y;

                P[t][0] *= (1.0f - k0);
                P[t][1] *= (1.0f - k1);
                /* Simplified P update for velocity */
                P[t][2] *= (1.0f - k_vx * DT);
                P[t][3] *= (1.0f - k_vy * DT);

                miss_count[t] = 0;
            } else {
                /* No measurement: prediction only */
                miss_count[t]++;
                if (miss_count[t] > 1) {
                    st_init[t] = false; /* invalidate after 1 frame coast */
                }
            }
        }

        /* Output */
        states_out[t].x   = st[t][0];
        states_out[t].y   = st[t][1];
        states_out[t].vx  = st[t][2];
        states_out[t].vy  = st[t][3];
        states_out[t].initialized = st_init[t];
    }
}

/* ===== Test Harness ===== */
#ifndef __SYNTHESIS__
static void print_states(const char* label, KalmanState s[3]) {
    printf("[%s]\n", label);
    for (int i = 0; i < 3; ++i) {
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

    for (int step = 0; step < 5; ++step) {
        kalman_filter(inp, out, false);
        char lbl[32]; snprintf(lbl, 32, "Step %d", step);
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
    inp[1] = {930,   935, 8,   true};
    inp[2] = {0, 0, 0, false};
    for (int step = 0; step < 5; ++step) {
        kalman_filter(inp, out, false);
    }
    print_states("After 5 steps", out);
    bool sep = (fabsf(out[0].x - out[1].x) > 500);
    printf("Separation: %s\n\n", sep ? "PASS" : "FAIL");

    /* --- Scenario 3: Target loss --- */
    printf("=== Scenario 3: Target Loss ===\n");
    kalman_filter(inp, out, true);
    inp[0] = {1713, -782, -16, true};
    inp[1] = {930,   935, 8,   true};
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
