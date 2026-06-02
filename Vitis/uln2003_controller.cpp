#include "ap_int.h"

/**
 * 28BYJ-48 + ULN2003 Dual Stepper Controller IP
 * @param target_pan  Pan 모터 절대 스텝 위치  (AXI4-Lite 0x10)
 * @param target_tilt Tilt 모터 절대 스텝 위치 (AXI4-Lite 0x18)
 * @param speed_delay 스텝 딜레이 계수         (AXI4-Lite 0x20)
 *        실제 지연 = speed_delay × 280 사이클 (PL 100MHz 기준):
 *          200 →  56,000 cycles ≈ 0.56ms/step ≈ 1786pps
 *          300 →  84,000 cycles ≈ 0.84ms/step ≈ 1190pps (권장)
 *          588 → 164,640 cycles ≈ 1.65ms/step ≈  606pps (공식 스펙 안전값)
 *
 * [수정 이력]
 * 기존 hw_delay(volatile int dummy++) 구현은 Vitis HLS 2023.2에서 완전 제거됨.
 * - 원인: 로컬 volatile 변수는 HLS에서 외부 포트 미연결 시 dead code 처리.
 * - 확인: csynth.rpt FSM 4상태, DELAY 루프 없음, Iter Latency=3 (30ns/step).
 * - 결과: speed_delay 값 무관 전 스텝이 수십 ns 내 완료 → 모터 탈조.
 * 수정: pan_out/tilt_out (ap_none 실제 포트) 반복 쓰기로 루프 보존.
 *       포트 쓰기는 HLS가 최적화 제거 불가 → 의도한 사이클 수 보장.
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

        // 스텝 간 지연: pan_out/tilt_out (실제 출력 포트)에 반복 쓰기
        //   → HLS 최적화 제거 불가. speed_delay × 280 클럭 사이클 소요.
        //   → 이 루프 동안 코일 여자 패턴이 유지되어 로터가 정렬됨.
        DELAY_LOOP: for (int d = 0; d < speed_delay * 280; d++) {
            #pragma HLS LOOP_TRIPCOUNT min=56000 max=165000
            pan_out  = step_seq[pan_idx];
            tilt_out = step_seq[tilt_idx];
        }
    }
}
