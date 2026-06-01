// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2023.2 (64-bit)
// Tool Version Limit: 2023.10
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
#ifndef XMTI_PROCESS_H
#define XMTI_PROCESS_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#ifndef __linux__
#include "xil_types.h"
#include "xil_assert.h"
#include "xstatus.h"
#include "xil_io.h"
#else
#include <stdint.h>
#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#endif
#include "xmti_process_hw.h"

/**************************** Type Definitions ******************************/
#ifdef __linux__
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#else
typedef struct {
#ifdef SDT
    char *Name;
#else
    u16 DeviceId;
#endif
    u64 Control_BaseAddress;
} XMti_process_Config;
#endif

typedef struct {
    u64 Control_BaseAddress;
    u32 IsReady;
} XMti_process;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XMti_process_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XMti_process_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XMti_process_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XMti_process_ReadReg(BaseAddress, RegOffset) \
    *(volatile u32*)((BaseAddress) + (RegOffset))

#define Xil_AssertVoid(expr)    assert(expr)
#define Xil_AssertNonvoid(expr) assert(expr)

#define XST_SUCCESS             0
#define XST_DEVICE_NOT_FOUND    2
#define XST_OPEN_DEVICE_FAILED  3
#define XIL_COMPONENT_IS_READY  1
#endif

/************************** Function Prototypes *****************************/
#ifndef __linux__
#ifdef SDT
int XMti_process_Initialize(XMti_process *InstancePtr, UINTPTR BaseAddress);
XMti_process_Config* XMti_process_LookupConfig(UINTPTR BaseAddress);
#else
int XMti_process_Initialize(XMti_process *InstancePtr, u16 DeviceId);
XMti_process_Config* XMti_process_LookupConfig(u16 DeviceId);
#endif
int XMti_process_CfgInitialize(XMti_process *InstancePtr, XMti_process_Config *ConfigPtr);
#else
int XMti_process_Initialize(XMti_process *InstancePtr, const char* InstanceName);
int XMti_process_Release(XMti_process *InstancePtr);
#endif

void XMti_process_Start(XMti_process *InstancePtr);
u32 XMti_process_IsDone(XMti_process *InstancePtr);
u32 XMti_process_IsIdle(XMti_process *InstancePtr);
u32 XMti_process_IsReady(XMti_process *InstancePtr);
void XMti_process_EnableAutoRestart(XMti_process *InstancePtr);
void XMti_process_DisableAutoRestart(XMti_process *InstancePtr);

void XMti_process_Set_prev_frame(XMti_process *InstancePtr, u64 Data);
u64 XMti_process_Get_prev_frame(XMti_process *InstancePtr);
void XMti_process_Set_curr_frame(XMti_process *InstancePtr, u64 Data);
u64 XMti_process_Get_curr_frame(XMti_process *InstancePtr);
void XMti_process_Set_width(XMti_process *InstancePtr, u32 Data);
u32 XMti_process_Get_width(XMti_process *InstancePtr);
void XMti_process_Set_height(XMti_process *InstancePtr, u32 Data);
u32 XMti_process_Get_height(XMti_process *InstancePtr);
void XMti_process_Set_params(XMti_process *InstancePtr, u64 Data);
u64 XMti_process_Get_params(XMti_process *InstancePtr);
void XMti_process_Set_roi_out(XMti_process *InstancePtr, u64 Data);
u64 XMti_process_Get_roi_out(XMti_process *InstancePtr);
void XMti_process_Set_roi_count(XMti_process *InstancePtr, u64 Data);
u64 XMti_process_Get_roi_count(XMti_process *InstancePtr);

void XMti_process_InterruptGlobalEnable(XMti_process *InstancePtr);
void XMti_process_InterruptGlobalDisable(XMti_process *InstancePtr);
void XMti_process_InterruptEnable(XMti_process *InstancePtr, u32 Mask);
void XMti_process_InterruptDisable(XMti_process *InstancePtr, u32 Mask);
void XMti_process_InterruptClear(XMti_process *InstancePtr, u32 Mask);
u32 XMti_process_InterruptGetEnabled(XMti_process *InstancePtr);
u32 XMti_process_InterruptGetStatus(XMti_process *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
