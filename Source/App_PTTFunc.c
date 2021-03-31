/*
 * App_PTTFunc.h : PTT application Function Model source file
 * Written by Chenm
 */

#include "hal_mcu.h"
#include "App_PTTFunc.h"
#include "CMUtil.h"
#include "service_PTT.h"
#include "CMTechPTT.h"
#include "Dev_MAX30102.h"
#include "Dev_ADS1x9x.h"

#define PTT_PACK_BYTE_NUM 17 // byte number per PTT packet, 1+4*4
#define PTT_MAX_PACK_NUM 255 // max packet num

// taskId of application
static uint8 taskId; 

// the number of the current ptt data packet, from 0 to PTT_MAX_PACK_NUM
static uint8 pckNum = 0;

// ptt packet buffer
static uint8 pckBuff[PTT_PACK_BYTE_NUM] = {0};

// pointer to the current position of the ptt buff
static uint8* pBuffPos;

// ptt packet structure sent out
static attHandleValueNoti_t pttNoti;

static bool ecgOk = false;
static bool ppgOk = false;
static int16 ecg = 0;
static uint16 ppg = 0;

// the callback function to process the PTT data read from MAX30102
static void processPttSignal(int16 ecg, uint16 ppg);

extern void PTTFunc_Init(uint8 taskID)
{ 
  taskId = taskID;  
  
  // 配置MAX30102
  MAX30102_Setup();
  // 进入低功耗模式
  MAX30102_Shutdown();  
  delayus(2000);  
  
  // 初始化ADS1x9x
  ADS1x9x_Init(); 
  delayus(2000);
  // 进入Power-down模式
  ADS1x9x_PowerDown(); 
}

extern void PTTFunc_SetPttSampling(bool start)
{
  if(start)
  {
    ecgOk = false;
    ecg = 0;
    ppgOk = false;
    ppg = 0;
    
    // 唤醒两个终端
    //MAX30102_WakeUp();
    //ADS1x9x_WakeUp(); 
    
    // 启动采集
    MAX30102_Start();
    ADS1x9x_StartConvert();
  } 
  else
  {    
    // 停止采集
    P0IE = 0;
    
    ADS1x9x_StopConvert();
    MAX30102_Stop();  
    
    delayus(10000);
    
    P0IE = 1;
    
    // 进入低功耗模式
    //ADS1x9x_StandBy();
    //MAX30102_Shutdown();
  }
  
  pckNum = 0;
  pBuffPos = pckBuff;
  osal_clear_event(taskId, PTT_PACKET_NOTI_EVT);
}

extern void PTTFunc_SendPttPacket(uint16 connHandle)
{
  PTT_PacketNotify( connHandle, &pttNoti );
}

#pragma vector = P0INT_VECTOR
__interrupt void PORT0_ISR(void)
{ 
  HAL_ENTER_ISR();  // Hold off interrupts.  
  
  // P0_1中断, 即ADS1191数据中断
  //if(P0IFG & 0x02)
  //{
    // 读ECG数据
    ecgOk = ADS1x9x_ReadEcgSample(&ecg);
    // 读PPG数据
    ppgOk = MAX30102_ReadPpgSample(&ppg);
    // 处理数据
    processPttSignal(ecg, ppg);
    
    P0IFG = 0;   //clear P0_1 IFG 
  //}  
  
//  // P0_2中断, 即MAX30102中断  
//  if(P0IFG & 0x04)
//  {
//    ppgOk = MAX30102_ReadPpgSample(&ppg);
//    P0IFG &= 0xFB;   // clear P0_2 IFG
//  }
//  
//  // PPG和ECG数据都有了，则处理数据
//  if(ecgOk && ppgOk)
//  {
//    processPttSignal(ecg, ppg);
//    ecgOk = false;
//    ppgOk = false;
//  }

  P0IF = 0;           //clear P0 interrupt flag
  
  HAL_EXIT_ISR();   // Re-enable interrupts.  
}

static void processPttSignal(int16 ecg, uint16 ppg)
{
  // 添加包序号
  if(pBuffPos == pckBuff)
  {
    *pBuffPos++ = pckNum;
    pckNum = (pckNum == PTT_MAX_PACK_NUM) ? 0 : pckNum+1;
  }
  
  // 添加数据
  *pBuffPos++ = LO_UINT16(ecg);
  *pBuffPos++ = HI_UINT16(ecg);
  *pBuffPos++ = LO_UINT16(ppg);
  *pBuffPos++ = HI_UINT16(ppg);
  
  // 包填满后，触发发送事件
  if(pBuffPos - pckBuff >= PTT_PACK_BYTE_NUM)
  {
    osal_memcpy(pttNoti.value, pckBuff, PTT_PACK_BYTE_NUM);
    pttNoti.len = PTT_PACK_BYTE_NUM;
    osal_set_event(taskId, PTT_PACKET_NOTI_EVT);
    pBuffPos = pckBuff;
  }
}