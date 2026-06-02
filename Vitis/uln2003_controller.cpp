#include "ap_int.h"

void hw_delay(int delay_count) {
    #pragma HLS INLINE off
    volatile int dummy = 0;
    for (int i = 0; i < delay_count; i++) {
        dummy++;
    }
}

/**
 * 28BYJ-48 + ULN2003 Dual Stepper Controller IP
 * @param target_pan  Pan 모터 절대 스텝 위치  (AXI4-Lite 0x10)
 * @param target_tilt Tilt 모터 절대 스텝 위치 (AXI4-Lite 0x18)
 * @param speed_delay 스텝 딜레이 루프카운트   (AXI4-Lite 0x20) — pan/tilt 공통, 권장 300 (≈0.85ms/step)
 * @param pan_out     Pan 모터 IN1~IN4 출력 핀
 * @param tilt_out    Tilt 모터 IN1~IN4 출력 핀
 */
void uln2003_controller(
    int target_pan,
    int target_tilt,
    int speed_delay,
    ap_uint<4> &pan_out,
    ap_uint<4> &tilt_out
) {
    #pragma HLS INTERFACE s_axilite port=target_pan  bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=target_tilt bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=speed_delay bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=return      bundle=CTRL
    #pragma HLS INTERFACE ap_none   port=pan_out
    #pragma HLS INTERFACE ap_none   port=tilt_out

    // 절대 위치 및 위상 인덱스 — static: HLS가 PL 레지스터로 보존
    static int current_pan  = 0;
    static int current_tilt = 0;
    static int pan_idx  = 0;
    static int tilt_idx = 0;

    // 28BYJ-48 half-step 시퀀스 (1-2상 여자, IN1~IN4)
    const ap_uint<4> step_seq[8] = {
        0b0001, 0b0011, 0b0010, 0b0110,
        0b0100, 0b1100, 0b1000, 0b1001
    };

    // 두 축이 목표에 도달할 때까지 1스텝씩 동시 구동
    // pan/tilt 중력 부하 차이 없음 → speed_delay 공통 적용
    while (current_pan != target_pan || current_tilt != target_tilt) {

        if (current_pan < target_pan) {
            pan_idx = (pan_idx + 1) % 8;
            current_pan++;
        } else if (current_pan > target_pan) {
            pan_idx = (pan_idx - 1 + 8) % 8;
            current_pan--;
        }

        if (current_tilt < target_tilt) {
            tilt_idx = (tilt_idx + 1) % 8;
            current_tilt++;
        } else if (current_tilt > target_tilt) {
            tilt_idx = (tilt_idx - 1 + 8) % 8;
            current_tilt--;
        }

        pan_out  = step_seq[pan_idx];
        tilt_out = step_seq[tilt_idx];

        // while 조건 상 루프 내부에서는 반드시 한 축 이상 스텝했으므로 무조건 딜레이
        hw_delay(speed_delay);
    }
}
