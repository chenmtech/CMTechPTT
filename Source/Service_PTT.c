/**
* PTT service source file: providing the PTT-related info and sending the PTT data packet
*/

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "CMUtil.h"
#include "Service_PTT.h"

// Position of PTT data packet in attribute array
#define PTT_PACK_VALUE_POS            2

// PTT service
CONST uint8 PTTServUUID[ATT_UUID_SIZE] =
{ 
  CM_UUID(PTT_SERV_UUID)
};

// PTT Data Packet characteristic
CONST uint8 PTTPackUUID[ATT_UUID_SIZE] =
{ 
  CM_UUID(PTT_PACK_UUID)
};

// Sample rate characteristic
CONST uint8 PTTSampleRateUUID[ATT_UUID_SIZE] =
{ 
  CM_UUID(PTT_SAMPLE_RATE_UUID)
};

static PTTServiceCBs_t* pttServiceCBs;

// PTT Service attribute
static CONST gattAttrType_t pttService = { ATT_UUID_SIZE, PTTServUUID };

// PTT Data Packet Characteristic
// Note: the characteristic value is not stored here
static uint8 pttPackProps = GATT_PROP_NOTIFY;
static uint8 pttPack = 0;
static gattCharCfg_t pttPackClientCharCfg[GATT_MAX_NUM_CONN];

// Sample Rate Characteristic
static uint8 pttSampleRateProps = GATT_PROP_READ;
static uint16 pttSampleRate = 125;


/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t PTTAttrTbl[] = 
{
  // PTT Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&pttService                      /* pValue */
  },

    // 1. PTT Data Packet Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &pttPackProps 
    },

      // PTT Data Packet Value
      { 
        { ATT_UUID_SIZE, PTTPackUUID },
        0, 
        0, 
        &pttPack 
      },

      // PTT Data Packet Client Characteristic Configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *) &pttPackClientCharCfg 
      },   

    // 2. Sample Rate Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &pttSampleRateProps 
    },

      // Sample Rate Value
      { 
        { ATT_UUID_SIZE, PTTSampleRateUUID },
        GATT_PERMIT_READ, 
        0, 
        (uint8*)&pttSampleRate 
      }
};

static uint8 readAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t writeAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );
static void handleConnStatusCB( uint16 connHandle, uint8 changeType );

// PTT Service Callbacks
CONST gattServiceCBs_t pttCBs =
{
  readAttrCB,  // Read callback function pointer
  writeAttrCB, // Write callback function pointer
  NULL         // Authorization callback function pointer
};

bStatus_t PTT_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, pttPackClientCharCfg );
  
  VOID linkDB_Register(handleConnStatusCB);

  if ( services & PTT_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( PTTAttrTbl, 
                                          GATT_NUM_ATTRS( PTTAttrTbl ),
                                          &pttCBs );
  }

  return ( status );
}

extern void PTT_RegisterAppCBs( PTTServiceCBs_t* pfnServiceCBs )
{
  pttServiceCBs = pfnServiceCBs;
    
  return;
}

extern bStatus_t PTT_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
     case PTT_PACK_CHAR_CFG:
      // Need connection handle
      //PTTMeasClientCharCfg.value = *((uint16*)value);
      break;   
      
    case PTT_SAMPLE_RATE:
      osal_memcpy((uint8*)&pttSampleRate, value, len);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
   
  return ( ret );
}

extern bStatus_t PTT_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case PTT_PACK_CHAR_CFG:
      // Need connection handle
      //*((uint16*)value) = PTTMeasClientCharCfg.value;
      break; 
      
    case PTT_SAMPLE_RATE:
      osal_memcpy(value, (uint8*)&pttSampleRate, 2);
      break;   

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

extern bStatus_t PTT_PacketNotify( uint16 connHandle, attHandleValueNoti_t *pNoti )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, pttPackClientCharCfg );

  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    // Set the handle
    pNoti->handle = PTTAttrTbl[PTT_PACK_VALUE_POS].handle;
  
    // Send the notification
    return GATT_Notification( connHandle, pNoti, FALSE );
  }

  return bleIncorrectMode;
}
                               
static uint8 readAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  uint16 uuid = 0;
  if (utilExtractUuid16(pAttr, &uuid) == FAILURE) {
    // Invalid handle
    *pLen = 0;
    return ATT_ERR_INVALID_HANDLE;
  }

  switch(uuid)
  {
    case PTT_SAMPLE_RATE_UUID:
      *pLen = 2;
       VOID osal_memcpy( pValue, pAttr->pValue, 2 );
       break;
      
    default:
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  return ( status );
}

static bStatus_t writeAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
 
  uint16 uuid = 0;
  if (utilExtractUuid16(pAttr,&uuid) == FAILURE) {
    // Invalid handle
    return ATT_ERR_INVALID_HANDLE;
  }
  
  switch ( uuid )
  {
    case GATT_CLIENT_CHAR_CFG_UUID:
      status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                               offset, GATT_CLIENT_CFG_NOTIFY );
      if ( status == SUCCESS )
      {
        uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );

        (pttServiceCBs->pfnPttServiceCB)( (charCfg == GATT_CFG_NO_OPERATION) ?
                                PTT_PACK_NOTI_DISABLED :
                                PTT_PACK_NOTI_ENABLED );
      }
      break;
 
    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  return ( status );
}

static void handleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    { 
      GATTServApp_InitCharCfg( connHandle, pttPackClientCharCfg );
    }
  }
}
