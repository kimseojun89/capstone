// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2023.2 (64-bit)
// Tool Version Limit: 2023.10
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
// control
// 0x00 : Control signals
//        bit 0  - ap_start (Read/Write/COH)
//        bit 1  - ap_done (Read/COR)
//        bit 2  - ap_idle (Read)
//        bit 3  - ap_ready (Read/COR)
//        bit 7  - auto_restart (Read/Write)
//        bit 9  - interrupt (Read)
//        others - reserved
// 0x04 : Global Interrupt Enable Register
//        bit 0  - Global Interrupt Enable (Read/Write)
//        others - reserved
// 0x08 : IP Interrupt Enable Register (Read/Write)
//        bit 0 - enable ap_done interrupt (Read/Write)
//        bit 1 - enable ap_ready interrupt (Read/Write)
//        others - reserved
// 0x0c : IP Interrupt Status Register (Read/TOW)
//        bit 0 - ap_done (Read/TOW)
//        bit 1 - ap_ready (Read/TOW)
//        others - reserved
// 0x10 : Data signal of prev_frame
//        bit 31~0 - prev_frame[31:0] (Read/Write)
// 0x14 : Data signal of prev_frame
//        bit 31~0 - prev_frame[63:32] (Read/Write)
// 0x18 : reserved
// 0x1c : Data signal of curr_frame
//        bit 31~0 - curr_frame[31:0] (Read/Write)
// 0x20 : Data signal of curr_frame
//        bit 31~0 - curr_frame[63:32] (Read/Write)
// 0x24 : reserved
// 0x28 : Data signal of width
//        bit 31~0 - width[31:0] (Read/Write)
// 0x2c : reserved
// 0x30 : Data signal of height
//        bit 31~0 - height[31:0] (Read/Write)
// 0x34 : reserved
// 0x38 : Data signal of params
//        bit 31~0 - params[31:0] (Read/Write)
// 0x3c : Data signal of params
//        bit 15~0 - params[47:32] (Read/Write)
//        others   - reserved
// 0x40 : reserved
// 0x44 : Data signal of roi_out
//        bit 31~0 - roi_out[31:0] (Read/Write)
// 0x48 : Data signal of roi_out
//        bit 31~0 - roi_out[63:32] (Read/Write)
// 0x4c : reserved
// 0x50 : Data signal of roi_count
//        bit 31~0 - roi_count[31:0] (Read/Write)
// 0x54 : Data signal of roi_count
//        bit 31~0 - roi_count[63:32] (Read/Write)
// 0x58 : reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XMTI_PROCESS_CONTROL_ADDR_AP_CTRL         0x00
#define XMTI_PROCESS_CONTROL_ADDR_GIE             0x04
#define XMTI_PROCESS_CONTROL_ADDR_IER             0x08
#define XMTI_PROCESS_CONTROL_ADDR_ISR             0x0c
#define XMTI_PROCESS_CONTROL_ADDR_PREV_FRAME_DATA 0x10
#define XMTI_PROCESS_CONTROL_BITS_PREV_FRAME_DATA 64
#define XMTI_PROCESS_CONTROL_ADDR_CURR_FRAME_DATA 0x1c
#define XMTI_PROCESS_CONTROL_BITS_CURR_FRAME_DATA 64
#define XMTI_PROCESS_CONTROL_ADDR_WIDTH_DATA      0x28
#define XMTI_PROCESS_CONTROL_BITS_WIDTH_DATA      32
#define XMTI_PROCESS_CONTROL_ADDR_HEIGHT_DATA     0x30
#define XMTI_PROCESS_CONTROL_BITS_HEIGHT_DATA     32
#define XMTI_PROCESS_CONTROL_ADDR_PARAMS_DATA     0x38
#define XMTI_PROCESS_CONTROL_BITS_PARAMS_DATA     48
#define XMTI_PROCESS_CONTROL_ADDR_ROI_OUT_DATA    0x44
#define XMTI_PROCESS_CONTROL_BITS_ROI_OUT_DATA    64
#define XMTI_PROCESS_CONTROL_ADDR_ROI_COUNT_DATA  0x50
#define XMTI_PROCESS_CONTROL_BITS_ROI_COUNT_DATA  64

