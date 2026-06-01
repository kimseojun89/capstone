/*
 * ============================================================
 *  legacy/mti_subsystem/ps_main_mti_excerpt.cpp
 *  — 2026-06-01 ps_main.cpp에서 분리된 MTI(영상 모션 감지) 코드 원본 보존
 *  — 컴파일 대상 아님. 부활 시 ps_main.cpp로 복원 + DISABLE_MTI 경로 복구.
 *  배경: FPGA 온보드 MTI는 PC YOLO 경로와 중복이며 Mock 상태(DISABLE_MTI=1)로만
 *        존재했음. 실제 드론 탐지는 PC가 수행 → 레거시 분리.
 * ============================================================
 */

// ── (1) MTI IP 레지스터 맵 ────────────────────────────────────
// 주: AP_START/AP_DONE/AP_IDLE 는 CORDIC/Kalman/Motor 공용이라 ps_main.cpp에 잔류시킴.
#ifndef XPAR_MTI_PROCESS_0_BASEADDR
    #define MTI_BASEADDR  0x40020000
#else
    #define MTI_BASEADDR  XPAR_MTI_PROCESS_0_BASEADDR
#endif

#define MTI_CTRL_OFF    0x00
#define MTI_PF1_OFF     0x10
#define MTI_PF2_OFF     0x14
#define MTI_CF1_OFF     0x1C
#define MTI_CF2_OFF     0x20
#define MTI_W_OFF       0x28
#define MTI_H_OFF       0x30
#define MTI_P1_OFF      0x38
#define MTI_P2_OFF      0x3C
#define MTI_ROI1_OFF    0x44
#define MTI_ROI2_OFF    0x48
#define MTI_CNT1_OFF    0x50
#define MTI_CNT2_OFF    0x54

#define MTI_WR(off, v)  Xil_Out32(MTI_BASEADDR + (off), (u32)(v))
#define MTI_RD(off)     Xil_In32 (MTI_BASEADDR + (off))

// ── (2) DDR 버퍼 영역 + MAX_ROIS ──────────────────────────────
#define PREV_BUF   0x00500000u
#define CURR_BUF   0x00520000u
#define ROI_BUF    0x00540000u
#define CNT_BUF    0x00540400u
#define MAX_ROIS     8

// ── (3) MTI 자료형 ────────────────────────────────────────────
typedef struct __attribute__((packed)) { uint16_t x,y,w,h; uint32_t area; float score; } BBox_t;
typedef struct __attribute__((packed)) { uint8_t dthr,blur; uint16_t mba,pad; } MtiParams_t;

// ── (4) 센서 퓨전 매칭 (레이더 방위각 ↔ MTI BBox) ─────────────
// angle_to_px()는 레이더 단독 모드에서도 쓰여 ps_main.cpp에 잔류. fuse()만 분리.
static int fuse(const BBox_t* bb, int n, int16_t at_10, int thr)
{
    if(n<=0) return -1;
    int rpx = angle_to_px(at_10);
    int best = -1, bd = thr + 1;
    for(int i=0;i<n;++i){
        int d = abs(bb[i].x + bb[i].w/2 - rpx);
        if(d < bd){bd=d;best=i;}
    }
    return best;
}

// ── (5) 듀얼축 PID 모터 업데이트 (퓨전 모드 전용이라 함께 분리) ─
// 레이더 단독 모드는 motor_update_hybrid()를 쓰므로 그것은 ps_main.cpp 잔류.
static void motor_update(int err_x, int err_y, bool has_target)
{
    float ramp_scale = (has_target && !g_had_target) ? MOTOR_ACQUIRE_RAMP : 1.0f;
    g_had_target = has_target;

    int dpan  = motor_pid_step((float)err_x, (float)(CAM_W_PX / 2),
                               MOTOR_PAN_DEADBAND, MOTOR_KP, MOTOR_KD,
                               MOTOR_PAN_MIN_STEP, MOTOR_PAN_MAX_STEP,
                               &g_pan_prev_err, &g_pan_cmd, ramp_scale);
    int dtilt = motor_pid_step((float)err_y, (float)(CAM_H_PX / 2),
                               MOTOR_TILT_DEADBAND, MOTOR_KP, MOTOR_KD,
                               MOTOR_TILT_MIN_STEP, MOTOR_TILT_MAX_STEP,
                               &g_tilt_prev_err, &g_tilt_cmd, ramp_scale);

    if (dpan == 0 && dtilt == 0) return;
    if (!(MOTOR_RD(0x00) & AP_IDLE)) return;   // IP 실행 중이면 누적 안 함

    g_motor_abs_pan  += dpan;
    g_motor_abs_tilt += dtilt;
    MOTOR_WR(0x10, (u32)g_motor_abs_pan);
    MOTOR_WR(0x18, (u32)g_motor_abs_tilt);
    MOTOR_WR(0x20, MOTOR_SPEED_DELAY);
    MOTOR_WR(0x00, AP_START);
}

// ── (6) MTI 초기화 / Mock 프레임 / 실행 ───────────────────────
static void mti_init(void)
{
    MtiParams_t p={25,1,200,10};
    uint32_t p1=0,p2=0;
    memcpy(&p1,&p,4); memcpy(&p2,(uint8_t*)&p+4,2);
    MTI_WR(MTI_W_OFF,  CAM_W_PX); MTI_WR(MTI_H_OFF,  CAM_H_PX);
    MTI_WR(MTI_P1_OFF, p1);       MTI_WR(MTI_P2_OFF, p2);
    MTI_WR(MTI_PF1_OFF,PREV_BUF); MTI_WR(MTI_PF2_OFF,0);
    MTI_WR(MTI_CF1_OFF,CURR_BUF); MTI_WR(MTI_CF2_OFF,0);
    MTI_WR(MTI_ROI1_OFF,ROI_BUF); MTI_WR(MTI_ROI2_OFF,0);
    MTI_WR(MTI_CNT1_OFF,CNT_BUF); MTI_WR(MTI_CNT2_OFF,0);
}

static void mock_frame(int cx, int cy, int r)
{
    uint8_t* pv=(uint8_t*)PREV_BUF;
    uint8_t* cu=(uint8_t*)CURR_BUF;
    for(int i=0;i<CAM_W_PX*CAM_H_PX;++i){
        pv[i]=(uint8_t)(100+(i*7+13)%30);
        cu[i]=(uint8_t)(100+(i*7+17)%30);
    }
    for(int row=0;row<CAM_H_PX;++row)
        for(int col=0;col<CAM_W_PX;++col){
            int dx=col-cx,dy=row-cy;
            if(dx*dx+dy*dy<=r*r) cu[row*CAM_W_PX+col]=200;
        }
}

static int mti_run(BBox_t* out)
{
#if DISABLE_MTI
    (void)out;
    return 0;
#else

#if USE_MOCK_MTI
    static int mcx=160;
    mcx=(mcx+1)%CAM_W_PX;
    mock_frame(mcx, 120, 15);
#endif

    Xil_DCacheFlushRange(PREV_BUF, CAM_W_PX*CAM_H_PX);
    Xil_DCacheFlushRange(CURR_BUF, CAM_W_PX*CAM_H_PX);
    MTI_WR(MTI_CTRL_OFF, AP_START);

    uint32_t to=10000000;
    while(!(MTI_RD(MTI_CTRL_OFF)&AP_DONE)){
        if ((to & 0x3FFu) == 0u) uart_accumulate();
        if(--to==0){ xil_printf("[WARN] MTI timeout\n"); return 0; }
    }

    Xil_DCacheInvalidateRange(ROI_BUF, sizeof(BBox_t)*MAX_ROIS);
    Xil_DCacheInvalidateRange(CNT_BUF, 4);

    int cnt=*(volatile int*)CNT_BUF;
    if(cnt<0||cnt>MAX_ROIS) cnt=0;
    BBox_t* raw=(BBox_t*)ROI_BUF;
    for(int i=0;i<cnt;++i) out[i]=raw[i];
    return cnt;
#endif
}

/* ── (7) 메인 루프에서의 사용 (참조용) ────────────────────────
   컴파일 스위치:  #define USE_MOCK_MTI 1   #define DISABLE_MTI 1
   부팅:          #if !DISABLE_MTI  mti_init();  #endif
   루프 상단:      static BBox_t bbox[MAX_ROIS];
                  int bc = mti_run(bbox);
                  int fi = fuse(bbox, bc, rang, FUSE_THRESHOLD_PX);
   모터 분기(퓨전): } else if (fi >= 0) {
                      int cy = bbox[fi].y + bbox[fi].h / 2;
                      motor_update(angle_to_px(rang) - CAM_W_PX/2, CAM_H_PX/2 - cy, true);
                  }
   로그:          [MTI] ROI=bc, [FUSE] radar->px vs bbox_cx 매칭 출력
   ───────────────────────────────────────────────────────────── */
