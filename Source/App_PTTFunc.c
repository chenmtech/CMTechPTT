/*
 * App_PTTFunc.h : PTT application Function Model source file
 * Written by Chenm
 */

#include "App_PTTFunc.h"
#include "CMUtil.h"
#include "service_PTT.h"
#include "CMTechPTT.h"
#include "Dev_MAX30102.h"

#define PTT_PACK_BYTE_NUM 19 // byte number per PTT packet, 1+9*2
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

// the callback function to process the PTT data from MAX30102
static void processPttSignal(uint16 ppg, int16 ecg);

extern void PTTFunc_Init(uint8 taskID, uint16 sampleRate)
{ 
  taskId = taskID;
  
  // initilize the MAX30102 and set the data process callback function
  MAX30102_Init(processPttSignal);
  MAX30102_Setup(HR_MODE, sampleRate);
  MAX30102_Shutdown();
  delayus(1000);
}

extern void PTTFunc_SetPttSampling(bool start)
{
  pckNum = 0;
  pPttBuff = pttBuff;
  osal_clear_event(taskId, PTT_PACKET_NOTI_EVT);
  if(start)
  {
    MAX30102_WakeUp();
    delayus(1000);
  } 
  else
  {
    MAX30102_Shutdown();
    delayus(2000);
  }
}

extern void PTTFunc_SendPttPacket(uint16 connHandle)
{
  PTT_PacketNotify( connHandle, &pttNoti );
}

static void processPttSignal(uint16 ppg, int16 ecg)
{
  if(pPttBuff == pttBuff)
  {
    *pPttBuff++ = pckNum;
    pckNum = (pckNum == PTT_MAX_PACK_NUM) ? 0 : pckNum+1;
  }
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