// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2023.2 (64-bit)
// Tool Version Limit: 2023.10
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
/***************************** Include Files *********************************/
#include "xmti_process.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XMti_process_CfgInitialize(XMti_process *InstancePtr, XMti_process_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Control_BaseAddress = ConfigPtr->Control_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XMti_process_Start(XMti_process *InstancePtr) {
    u32 Data;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMti_process_ReadReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_AP_CTRL) & 0x80;
    XMti_process_WriteReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_AP_CTRL, Data | 0x01);
}

u32 XMti_process_IsDone(XMti_process *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMti_process_ReadReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_AP_CTRL);
    return (Data >> 1) & 0x1;
}

u32 XMti_process_IsIdle(XMti_process *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMti_process_ReadReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_AP_CTRL);
    return (Data >> 2) & 0x1;
}

u32 XMti_process_IsReady(XMti_process *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMti_process_ReadReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_AP_CTRL);
    // check ap_start to see if the pcore is ready for next input
    return !(Data & 0x1);
}

void XMti_process_EnableAutoRestart(XMti_process *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMti_process_WriteReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_AP_CTRL, 0x80);
}

void XMti_process_DisableAutoRestart(XMti_process *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMti_process_WriteReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_AP_CTRL, 0);
}

void XMti_process_Set_prev_frame(XMti_process *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMti_process_WriteReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_PREV_FRAME_DATA, (u32)(Data));
    XMti_process_WriteReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_PREV_FRAME_DATA + 4, (u32)(Data >> 32));
}

u64 XMti_process_Get_prev_frame(XMti_process *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMti_process_ReadReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_PREV_FRAME_DATA);
    Data += (u64)XMti_process_ReadReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_PREV_FRAME_DATA + 4) << 32;
    return Data;
}

void XMti_process_Set_curr_frame(XMti_process *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMti_process_WriteReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_CURR_FRAME_DATA, (u32)(Data));
    XMti_process_WriteReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_CURR_FRAME_DATA + 4, (u32)(Data >> 32));
}

u64 XMti_process_Get_curr_frame(XMti_process *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMti_process_ReadReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_CURR_FRAME_DATA);
    Data += (u64)XMti_process_ReadReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_CURR_FRAME_DATA + 4) << 32;
    return Data;
}

void XMti_process_Set_width(XMti_process *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMti_process_WriteReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_WIDTH_DATA, Data);
}

u32 XMti_process_Get_width(XMti_process *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMti_process_ReadReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_WIDTH_DATA);
    return Data;
}

void XMti_process_Set_height(XMti_process *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMti_process_WriteReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_HEIGHT_DATA, Data);
}

u32 XMti_process_Get_height(XMti_process *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMti_process_ReadReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_HEIGHT_DATA);
    return Data;
}

void XMti_process_Set_params(XMti_process *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMti_process_WriteReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_PARAMS_DATA, (u32)(Data));
    XMti_process_WriteReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_PARAMS_DATA + 4, (u32)(Data >> 32));
}

u64 XMti_process_Get_params(XMti_process *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMti_process_ReadReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_PARAMS_DATA);
    Data += (u64)XMti_process_ReadReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_PARAMS_DATA + 4) << 32;
    return Data;
}

void XMti_process_Set_roi_out(XMti_process *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMti_process_WriteReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_ROI_OUT_DATA, (u32)(Data));
    XMti_process_WriteReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_ROI_OUT_DATA + 4, (u32)(Data >> 32));
}

u64 XMti_process_Get_roi_out(XMti_process *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMti_process_ReadReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_ROI_OUT_DATA);
    Data += (u64)XMti_process_ReadReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_ROI_OUT_DATA + 4) << 32;
    return Data;
}

void XMti_process_Set_roi_count(XMti_process *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMti_process_WriteReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_ROI_COUNT_DATA, (u32)(Data));
    XMti_process_WriteReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_ROI_COUNT_DATA + 4, (u32)(Data >> 32));
}

u64 XMti_process_Get_roi_count(XMti_process *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMti_process_ReadReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_ROI_COUNT_DATA);
    Data += (u64)XMti_process_ReadReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_ROI_COUNT_DATA + 4) << 32;
    return Data;
}

void XMti_process_InterruptGlobalEnable(XMti_process *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMti_process_WriteReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_GIE, 1);
}

void XMti_process_InterruptGlobalDisable(XMti_process *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMti_process_WriteReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_GIE, 0);
}

void XMti_process_InterruptEnable(XMti_process *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XMti_process_ReadReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_IER);
    XMti_process_WriteReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_IER, Register | Mask);
}

void XMti_process_InterruptDisable(XMti_process *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XMti_process_ReadReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_IER);
    XMti_process_WriteReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_IER, Register & (~Mask));
}

void XMti_process_InterruptClear(XMti_process *InstancePtr, u32 Mask) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMti_process_WriteReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_ISR, Mask);
}

u32 XMti_process_InterruptGetEnabled(XMti_process *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XMti_process_ReadReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_IER);
}

u32 XMti_process_InterruptGetStatus(XMti_process *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XMti_process_ReadReg(InstancePtr->Control_BaseAddress, XMTI_PROCESS_CONTROL_ADDR_ISR);
}

