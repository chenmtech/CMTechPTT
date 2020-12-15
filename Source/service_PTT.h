/**
* PTT service header file: providing the PTT-related info and sending the PTT data packet
*/

#ifndef SERVICE_PTT_H
#define SERVICE_PTT_H

// PTT Service Parameters
#define PTT_PACK                      0  // PTT data packet
#define PTT_PACK_CHAR_CFG             1  // PTT packet CCC
#define PTT_SAMPLE_RATE               2  // sample rate

// PTT Service UUIDs
#define PTT_SERV_UUID                 0xAAC0
#define PTT_PACK_UUID                 0xAAC1
#define PTT_SAMPLE_RATE_UUID          0xAAC2

// PTT Service bit fields
#define PTT_SERVICE                   0x00000001

// Callback events
#define PTT_PACK_NOTI_ENABLED         0 // PTT data packet notification enabled
#define PTT_PACK_NOTI_DISABLED        1 // PTT data packet notification disabled

// PTT Service callback function
typedef void (*pttServiceCB_t)(uint8 event);

typedef struct
{
  pttServiceCB_t    pfnPttServiceCB;  
} PTTServiceCBs_t;


extern bStatus_t PTT_AddService( uint32 services );
extern void PTT_RegisterAppCBs( PTTServiceCBs_t* pfnServiceCBs );
extern bStatus_t PTT_SetParameter( uint8 param, uint8 len, void *value );
extern bStatus_t PTT_GetParameter( uint8 param, void *value );
extern bStatus_t PTT_PacketNotify( uint16 connHandle, attHandleValueNoti_t *pNoti );// notify the PTT data packet



#endif /* PTTSERVICE_H */
