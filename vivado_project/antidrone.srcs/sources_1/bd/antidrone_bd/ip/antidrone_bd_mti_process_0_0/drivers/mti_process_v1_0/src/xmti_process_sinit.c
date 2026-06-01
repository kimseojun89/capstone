// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2023.2 (64-bit)
// Tool Version Limit: 2023.10
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
#ifndef __linux__

#include "xstatus.h"
#ifdef SDT
#include "xparameters.h"
#endif
#include "xmti_process.h"

extern XMti_process_Config XMti_process_ConfigTable[];

#ifdef SDT
XMti_process_Config *XMti_process_LookupConfig(UINTPTR BaseAddress) {
	XMti_process_Config *ConfigPtr = NULL;

	int Index;

	for (Index = (u32)0x0; XMti_process_ConfigTable[Index].Name != NULL; Index++) {
		if (!BaseAddress || XMti_process_ConfigTable[Index].Control_BaseAddress == BaseAddress) {
			ConfigPtr = &XMti_process_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XMti_process_Initialize(XMti_process *InstancePtr, UINTPTR BaseAddress) {
	XMti_process_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XMti_process_LookupConfig(BaseAddress);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XMti_process_CfgInitialize(InstancePtr, ConfigPtr);
}
#else
XMti_process_Config *XMti_process_LookupConfig(u16 DeviceId) {
	XMti_process_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XMTI_PROCESS_NUM_INSTANCES; Index++) {
		if (XMti_process_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XMti_process_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XMti_process_Initialize(XMti_process *InstancePtr, u16 DeviceId) {
	XMti_process_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XMti_process_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XMti_process_CfgInitialize(InstancePtr, ConfigPtr);
}
#endif

#endif

