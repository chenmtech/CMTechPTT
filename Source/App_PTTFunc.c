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

static uint8 taskId; // taskId of application

// the number of the current ptt data packet, from 0 to PTT_MAX_PACK_NUM
static uint8 pckNum = 0;
// ptt packet buffer
static uint8 pttBuff[PTT_PACK_BYTE_NUM] = {0};
// pointer to the current position of the ptt buff
static uint8* pPttBuff;
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
  
  // 判断MAX30102是否上电
  while(!MAX30102_IsPowerOn());
  // 配置MAX30102
  MAX30102_Setup();
  // 进入低功耗模式
  MAX30102_Shutdown();
  
  // initilize the ADS1x9x
  ADS1x9x_Init(); 
  // 进入Power-down模式
  ADS1x9x_PowerDown(); 
}

extern void PTTFunc_SetPttSampling(bool start)
{
  pckNum = 0;
  pPttBuff = pttBuff;
  osal_clear_event(taskId, PTT_PACKET_NOTI_EVT);
  if(start)
  {
    ecgOk = false;
    ecg = 0;
    ppgOk = false;
    ppg = 0;
    
    MAX30102_WakeUp();
    ADS1x9x_WakeUp(); 
    // 这里一定要延时，否则容易死机
    //delayus(1000);
    
    ADS1x9x_StartConvert();
    MAX30102_Start();
    //delayus(1000);
  } 
  else
  {    
    ADS1x9x_StopConvert();
    MAX30102_Stop();    
    //delayus(1000);
    
    ADS1x9x_StandBy();
    MAX30102_Shutdown();
    //delayus(1000);
  }
}

extern void PTTFunc_SendPttPacket(uint16 connHandle)
{
  PTT_PacketNotify( connHandle, &pttNoti );
}

#pragma vector = P0INT_VECTOR
__interrupt void PORT0_ISR(void)
{ 
  HAL_ENTER_ISR();  // Hold off interrupts.
  
  // P0_2中断, 即MAX30102数据中断  
  if(P0IFG & 0x04)
  {
    //if(MAX30102_IsDATARDY()) // MAX30102数据中断 
    //{
      ppgOk = MAX30102_ReadPpgSample(&ppg);
    //}
    P0IFG &= ~(1<<2);   // clear P0_2 IFG
  }
  
  // P0_1中断, 即ADS1191数据中断
  if(P0IFG & 0x02)
  {
    ecgOk = ADS1x9x_ReadEcgSample(&ecg);
    P0IFG &= ~(1<<1);   //clear P0_1 IFG 
  }  
  
  if(ecgOk && ppgOk)
  {
    processPttSignal(ecg, ppg);
    ecgOk = false;
    ppgOk = false;
  }
  
  P0IF = 0;           //clear P0 interrupt flag
  
  HAL_EXIT_ISR();   // Re-enable interrupts.  
}

static void processPttSignal(int16 ecg, uint16 ppg)
{
  if(pPttBuff == pttBuff)
  {
    *pPttBuff++ = pckNum;
    pckNum = (pckNum == PTT_MAX_PACK_NUM) ? 0 : pckNum+1;
  }
  *pPttBuff++ = LO_UINT16(ecg);  
  *pPttBuff++ = HI_UINT16(ecg);
  *pPttBuff++ = LO_UINT16(ppg);
  *pPttBuff++ = HI_UINT16(ppg);
  
  if(pPttBuff-pttBuff >= PTT_PACK_BYTE_NUM)
  {
    osal_memcpy(pttNoti.value, pttBuff, PTT_PACK_BYTE_NUM);
    pttNoti.len = PTT_PACK_BYTE_NUM;
    osal_set_event(taskId, PTT_PACKET_NOTI_EVT);
    pPttBuff = pttBuff;
  }
}