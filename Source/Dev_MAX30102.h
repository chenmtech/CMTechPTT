/*
 * MAX30102: 脉搏和心率传感器MAX30102驱动
 * written by chenm 2020-12-07
*/

#ifndef DEV_MAX30102_H
#define DEV_MAX30102_H

extern void MAX30102_Setup();

extern void MAX30102_WakeUp();

extern void MAX30102_Shutdown();

extern void MAX30102_Start();

extern void MAX30102_Stop();

extern float MAX30102_ReadTemperature();

extern bool MAX30102_ReadPpgSample(uint16* pData);


#endif

