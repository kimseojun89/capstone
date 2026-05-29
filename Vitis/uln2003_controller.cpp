#include "ap_int.h"

// 하드웨어 타이머 역할을 하는 딜레이 함수
void hw_delay(int delay_count) {
    #pragma HLS INLINE off
    volatile int dummy = 0;
    for (int i = 0; i < delay_count; i++) {
        dummy++;
    }
}

/**
 * 28BYJ-48 + ULN2003 Dual Stepper Controller IP
 * * @param target_pan  Pan 모터 목표 스텝 (AXI4-Lite)
 * @param target_tilt Tilt 모터 목표 스텝 (AXI4-Lite)
 * @param speed_delay 스텝 간 딜레이 속도 조절 (AXI4-Lite)
 * @param pan_out     Pan 모터 IN1~IN4 출력 핀 (외부 4핀)
 * @param tilt_out    Tilt 모터 IN1~IN4 출력 핀 (외부 4핀)
 */
void uln2003_controller(
    int target_pan,
    int target_tilt,
    int speed_delay,
    ap_uint<4> &pan_out,
    ap_uint<4> &tilt_out
) {
    // 1. AXI4-Lite 인터페이스 (PS에서 제어 명령을 내리는 레지스터)
    #pragma HLS INTERFACE s_axilite port=target_pan bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=target_tilt bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=speed_delay bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=return bundle=CTRL

    // 2. 외부 GPIO 핀 출력 (ap_none: 프로토콜 없는 순수 와이어 출력)
    #pragma HLS INTERFACE ap_none port=pan_out
    #pragma HLS INTERFACE ap_none port=tilt_out

    // 현재 위치 및 위상(Phase) 인덱스 메모리 (하드웨어 레지스터로 유지됨)
    static int current_pan = 0;
    static int current_tilt = 0;
    static int pan_idx = 0;
    static int tilt_idx = 0;

    // 28BYJ-48용 8박자 하프스텝(Half-step) 시퀀스 배열 (1-2상 여자)
    const ap_uint<4> step_seq[8] = {
        0b0001, 0b0011, 0b0010, 0b0110,
        0b0100, 0b1100, 0b1000, 0b1001
    };

    // 💡 핵심 로직: 두 모터 중 하나라도 목표에 도달하지 않았다면 계속 회전 (PS 오프로딩)
    while (current_pan != target_pan || current_tilt != target_tilt) {

        bool moved = false;

        // Pan 모터 1스텝 구동 로직
        if (current_pan < target_pan) {
            pan_idx = (pan_idx + 1) % 8;
            current_pan++;
            moved = true;
        } else if (current_pan > target_pan) {
            pan_idx = (pan_idx - 1 + 8) % 8;
            current_pan--;
            moved = true;
        }

        // Tilt 모터 1스텝 구동 로직
        if (current_tilt < target_tilt) {
            tilt_idx = (tilt_idx + 1) % 8;
            current_tilt++;
            moved = true;
        } else if (current_tilt > target_tilt) {
            tilt_idx = (tilt_idx - 1 + 8) % 8;
            current_tilt--;
            moved = true;
        }

        // 핀 상태 업데이트
        pan_out = step_seq[pan_idx];
        tilt_out = step_seq[tilt_idx];

        // 물리적 모터가 반응할 수 있도록 딜레이 부여
        if (moved) {
            hw_delay(speed_delay);
        }
    }
}
