/**
* PPG service header file: providing the PPG-related info and sending the PPG data packet
*/

#ifndef SERVICE_PPG_H
#define SERVICE_PPG_H

// PPG Service Parameters
#define PPG_PACK                      0  // PPG data packet
#define PPG_PACK_CHAR_CFG             1  // PPG packet CCC
#define PPG_SAMPLE_RATE               2  // sample rate

// PPG Service UUIDs
#define PPG_SERV_UUID                 0xAAB0
#define PPG_PACK_UUID                 0xAAB1
#define PPG_SAMPLE_RATE_UUID          0xAAB2

// PPG Service bit fields
#define PPG_SERVICE                   0x00000001

// Callback events
#define PPG_PACK_NOTI_ENABLED         0 // PPG data packet notification enabled
#define PPG_PACK_NOTI_DISABLED        1 // PPG data packet notification disabled

// PPG Service callback function
typedef void (*ppgServiceCB_t)(uint8 event);

typedef struct
{
  ppgServiceCB_t    pfnPpgServiceCB;  
} PPGServiceCBs_t;


extern bStatus_t PPG_AddService( uint32 services );
extern void PPG_RegisterAppCBs( PPGServiceCBs_t* pfnServiceCBs );
extern bStatus_t PPG_SetParameter( uint8 param, uint8 len, void *value );
extern bStatus_t PPG_GetParameter( uint8 param, void *value );
extern bStatus_t PPG_PacketNotify( uint16 connHandle, attHandleValueNoti_t *pNoti );// notify the PPG data packet



#endif /* PPGSERVICE_H */
