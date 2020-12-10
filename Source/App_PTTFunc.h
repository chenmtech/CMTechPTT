/*
 * App_PTTFunc.h : PTT application Function Model header file
 * Written by Chenm
 */

#ifndef APP_PTTFUNC_H
#define APP_PTTFUNC_H

#include "hal_types.h"

extern void PTTFunc_Init(uint8 taskID, uint16 sampleRate); //init
extern void PTTFunc_SetPttSampling(bool start); // set up the PTT sampling started or stopped
extern void PTTFunc_SendPttPacket(uint16 connHandle); // send PTT data packet

#endif