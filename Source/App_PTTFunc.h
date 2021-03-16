/*
 * App_PTTFunc.h : PTT application Function Model header file
 * Written by Chenm
 */

#ifndef APP_PTTFUNC_H
#define APP_PTTFUNC_H

#include "hal_types.h"

extern void PTTFunc_Init(uint8 taskID); //init
extern void PTTFunc_SetPttSampling(bool start); // start or stop the PTT sampling
extern void PTTFunc_SendPttPacket(uint16 connHandle); // send PTT data packet with connHandle

#endif