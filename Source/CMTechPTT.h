/**************************************************************************************************
* CMTechPTT.h : PTT application header file
**************************************************************************************************/

#ifndef CMTECHPTT_H
#define CMTECHPTT_H


#define PTT_START_DEVICE_EVT 0x0001      // device start event
#define PTT_PACKET_NOTI_EVT 0x0002       // PTT data packet notification event

/*
 * Task Initialization for the BLE Application
 */
extern void PTT_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 PTT_ProcessEvent( uint8 task_id, uint16 events );


#endif 
