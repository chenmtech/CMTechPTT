/**
* PPG service source file: providing the PPG-related info and sending the PPG data packet
*/

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "CMUtil.h"
#include "Service_PPG.h"

// Position of PPG data packet in attribute array
#define PPG_PACK_VALUE_POS            2

// PPG service
CONST uint8 PPGServUUID[ATT_UUID_SIZE] =
{ 
  CM_UUID(PPG_SERV_UUID)
};

// PPG Data Packet characteristic
CONST uint8 PPGPackUUID[ATT_UUID_SIZE] =
{ 
  CM_UUID(PPG_PACK_UUID)
};

// Sample rate characteristic
CONST uint8 PPGSampleRateUUID[ATT_UUID_SIZE] =
{ 
  CM_UUID(PPG_SAMPLE_RATE_UUID)
};

static PPGServiceCBs_t* ppgServiceCBs;

// PPG Service attribute
static CONST gattAttrType_t ppgService = { ATT_UUID_SIZE, PPGServUUID };

// PPG Data Packet Characteristic
// Note: the characteristic value is not stored here
static uint8 ppgPackProps = GATT_PROP_NOTIFY;
static uint8 ppgPack = 0;
static gattCharCfg_t ppgPackClientCharCfg[GATT_MAX_NUM_CONN];

// Sample Rate Characteristic
static uint8 ppgSampleRateProps = GATT_PROP_READ;
static uint16 ppgSampleRate = 125;


/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t PPGAttrTbl[] = 
{
  // PPG Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&ppgService                      /* pValue */
  },

    // 1. PPG Data Packet Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &ppgPackProps 
    },

      // PPG Data Packet Value
      { 
        { ATT_UUID_SIZE, PPGPackUUID },
        0, 
        0, 
        &ppgPack 
      },

      // PPG Data Packet Client Characteristic Configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *) &ppgPackClientCharCfg 
      },   

    // 2. Sample Rate Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &ppgSampleRateProps 
    },

      // Sample Rate Value
      { 
        { ATT_UUID_SIZE, PPGSampleRateUUID },
        GATT_PERMIT_READ, 
        0, 
        (uint8*)&ppgSampleRate 
      }
};

static uint8 readAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t writeAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );
static void handleConnStatusCB( uint16 connHandle, uint8 changeType );

// PPG Service Callbacks
CONST gattServiceCBs_t ppgCBs =
{
  readAttrCB,  // Read callback function pointer
  writeAttrCB, // Write callback function pointer
  NULL         // Authorization callback function pointer
};

bStatus_t PPG_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, ppgPackClientCharCfg );
  
  VOID linkDB_Register(handleConnStatusCB);

  if ( services & PPG_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( PPGAttrTbl, 
                                          GATT_NUM_ATTRS( PPGAttrTbl ),
                                          &ppgCBs );
  }

  return ( status );
}

extern void PPG_RegisterAppCBs( PPGServiceCBs_t* pfnServiceCBs )
{
  ppgServiceCBs = pfnServiceCBs;
    
  return;
}

extern bStatus_t PPG_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
     case PPG_PACK_CHAR_CFG:
      // Need connection handle
      //PPGMeasClientCharCfg.value = *((uint16*)value);
      break;   
      
    case PPG_SAMPLE_RATE:
      osal_memcpy((uint8*)&ppgSampleRate, value, len);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
   
  return ( ret );
}

extern bStatus_t PPG_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case PPG_PACK_CHAR_CFG:
      // Need connection handle
      //*((uint16*)value) = PPGMeasClientCharCfg.value;
      break; 
      
    case PPG_SAMPLE_RATE:
      osal_memcpy(value, (uint8*)&ppgSampleRate, 2);
      break;   

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

extern bStatus_t PPG_PacketNotify( uint16 connHandle, attHandleValueNoti_t *pNoti )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, ppgPackClientCharCfg );

  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    // Set the handle
    pNoti->handle = PPGAttrTbl[PPG_PACK_VALUE_POS].handle;
  
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
    case PPG_SAMPLE_RATE_UUID:
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

        (ppgServiceCBs->pfnPpgServiceCB)( (charCfg == GATT_CFG_NO_OPERATION) ?
                                PPG_PACK_NOTI_DISABLED :
                                PPG_PACK_NOTI_ENABLED );
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
      GATTServApp_InitCharCfg( connHandle, ppgPackClientCharCfg );
    }
  }
}
