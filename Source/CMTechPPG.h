/**************************************************************************************************
* CMTechPPG.h : PPG application header file
**************************************************************************************************/

#ifndef CMTECHPPG_H
#define CMTECHPPG_H


#define PPG_START_DEVICE_EVT 0x0001      // device start event
#define PPG_PACKET_NOTI_EVT 0x0002       // PPG data packet notification event

/*
 * Task Initialization for the BLE Application
 */
extern void PPG_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 PPG_ProcessEvent( uint8 task_id, uint16 events );


#endif 
