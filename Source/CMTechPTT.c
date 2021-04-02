/**************************************************************************************************
* CMTechPTT.c: main application source file
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "osal_snv.h"
#include "linkdb.h"
#include "OnBoard.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "cmutil.h"

#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif
#include "gapbondmgr.h"
#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

#include "CMTechPTT.h"
#include "Service_DevInfo.h"
#include "Service_PTT.h"
#include "App_PTTFunc.h"
#include "Dev_ADS1x9x.h"
#include "Dev_MAX30102.h"

#define ADVERTISING_INTERVAL 320 // ad interval, units of 0.625ms
#define ADVERTISING_DURATION 2000 // ad duration, units of ms
#define ADVERTISING_OFFTIME 8000 // ad offtime to wait for a next ad, units of ms

// connection parameter in PTT mode
#define PTT_MODE_MIN_INTERVAL 16  // unit: 1.25ms
#define PTT_MODE_MAX_INTERVAL 32  // unit: 1.25ms
#define PTT_MODE_SLAVE_LATENCY 4
#define PTT_MODE_CONNECT_TIMEOUT 100 // unit: 10ms, If no connection event occurred during this timeout, the connect will be shut down.

#define CONN_PAUSE_PERIPHERAL 4  // the pause time from the connection establishment to the update of the connection parameters

#define INVALID_CONNHANDLE 0xFFFF // invalid connection handle
#define STATUS_PTT_STOP 0x00     // PTT sampling stopped status
#define STATUS_PTT_START 0x01    // PTT sampling started status


static uint8 taskID;   
static uint16 gapConnHandle = INVALID_CONNHANDLE;
static gaprole_States_t gapProfileState = GAPROLE_INIT;
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "KM PTT"; // GGS device name
static uint8 status = STATUS_PTT_STOP; // PTT sampling status
static uint16 pttSampleRate = 250; // PTT real sample rate

// advertise data
static uint8 advertData[] = 
{ 
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,
  LO_UINT16( PTT_SERV_UUID ),
  HI_UINT16( PTT_SERV_UUID ),

};

// scan response data
static uint8 scanResponseData[] =
{
  0x06,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_SHORT,   
  'K',
  'M',
  'P',
  'T',
  'T'
};

static void gapStateCB( gaprole_States_t newState ); // gap state callback function
static void pttServiceCB( uint8 event ); // PTT service callback function

// GAP Role callback struct
static gapRolesCBs_t gapStateCBs =
{
  gapStateCB,         // Profile State Change Callbacks
  NULL                // When a valid RSSI is read from controller (not used by application)
};

static gapBondCBs_t bondCBs =
{
  NULL,                   // Passcode callback
  NULL                    // Pairing state callback
};

// PTT service callback struct
static PTTServiceCBs_t pttServCBs =
{
  pttServiceCB    
};

static void processOSALMsg( osal_event_hdr_t *pMsg ); // OSAL message process function
static void initIOPin(); // initialize IO pins
static void initInterrupt(); // initialize interrupt
static void startPttSampling( void ); // start PTT sampling
static void stopPttSampling( void ); // stop PTT sampling

extern void PTT_Init( uint8 task_id )
{ 
  taskID = task_id;
  
  HCI_EXT_SetTxPowerCmd (LL_EXT_TX_POWER_0_DBM);
  
  // Setup the GAP Peripheral Role Profile
  {
    // set the advertising data and scan response data
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanResponseData ), scanResponseData );
    
    // set the advertising parameters
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, ADVERTISING_INTERVAL ); // units of 0.625ms
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, ADVERTISING_INTERVAL ); // units of 0.625ms
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, ADVERTISING_DURATION ); // advertising duration
    uint16 gapRole_AdvertOffTime = ADVERTISING_OFFTIME;
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    
    // enable advertising
    uint8 advertising = TRUE;
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advertising );

    GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, CONN_PAUSE_PERIPHERAL ); 
    
    // set the connection parameter
    uint16 desired_min_interval = PTT_MODE_MIN_INTERVAL;
    uint16 desired_max_interval = PTT_MODE_MAX_INTERVAL;
    uint16 desired_slave_latency = PTT_MODE_SLAVE_LATENCY;
    uint16 desired_conn_timeout = PTT_MODE_CONNECT_TIMEOUT; 
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );  
    
    uint8 enable_update_request = TRUE;
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
  }
  
  // set GGS device name
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }  

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes
  DevInfo_AddService( ); // device information service  
  PTT_AddService(GATT_ALL_SERVICES); // ptt service
  
  // set sample rate in ptt service
  {
    PTT_SetParameter( PTT_SAMPLE_RATE, sizeof ( uint16 ), &pttSampleRate ); 
  }
  
  //初始化IO管脚
  initIOPin();    
  
  // 初始化中断
  initInterrupt(); 
  
  // PTT应用初始化
  PTTFunc_Init(taskID); 
  
  PTT_RegisterAppCBs( &pttServCBs );  
  
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );  

  // 启动设备
  osal_set_event( taskID, PTT_START_DEVICE_EVT );
}

// 初始化IO管脚
static void initIOPin()
{
  // 全部设为GPIO
  P0SEL = 0; 
  P1SEL = 0; 
  P2SEL = 0; 

  // P0.1设为输入，其他设为输出
  P0DIR = 0xFD; 
  P1DIR = 0xFF; 
  P2DIR = 0x1F; 

  // 低电平
  P0 = 0; 
  P1 = 0;   
  P2 = 0; 
}

static void initInterrupt()
{  
  // 关P0.1中断
  P0IEN &= 0xFD;
  P0IFG &= 0xFD;  
  P0IF = 0;   
  
  PICTL |= (1<<0);  //所有P0管脚都是下降沿触发
  
  //开P0.1 INT中断
  P0IEN |= 0x02;  
  P0IE = 0; // 关P0总中断  
}

extern uint16 PTT_ProcessEvent( uint8 task_id, uint16 events )
{
  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( taskID )) != NULL )
    {
      processOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & PTT_START_DEVICE_EVT )
  {    
    // Start the Device
    VOID GAPRole_StartDevice( &gapStateCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &bondCBs );
    
  
    //startPttSampling();

    return ( events ^ PTT_START_DEVICE_EVT );
  }
  
  if ( events & PTT_PACKET_NOTI_EVT )
  {
    //if (gapProfileState == GAPROLE_CONNECTED)
    //{
      PTTFunc_SendPttPacket(gapConnHandle);
    //}

    return (events ^ PTT_PACKET_NOTI_EVT);
  } 
  
  // Discard unknown events
  return 0;
}

static void processOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    default:
      // do nothing
      break;
  }
}

static void gapStateCB( gaprole_States_t newState )
{
  // 已连接
  if( newState == GAPROLE_CONNECTED)
  {
    // Get connection handle
    GAPRole_GetParameter( GAPROLE_CONNHANDLE, &gapConnHandle );
    
    // 让ADS1x9x从power-down模式退出
    ADS1x9x_PowerUp(); 
    
    // 让MAX30102从shutdown模式退出
    MAX30102_WakeUp();
    
    delayus(2000);
  }
  
  // 断开连接
  else if(gapProfileState == GAPROLE_CONNECTED && 
            newState != GAPROLE_CONNECTED)
  {
    // 停止采样
    stopPttSampling();
    
    // ADS1x9x进入Power-down模式
    ADS1x9x_PowerDown();
    
    // MAX30102进入shutdown模式
    MAX30102_Shutdown();
  }
  // if started
  else if (newState == GAPROLE_STARTED)
  {
    // Set the system ID from the bd addr
    uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
    GAPRole_GetParameter(GAPROLE_BD_ADDR, systemId);
    
    // shift three bytes up
    systemId[7] = systemId[5];
    systemId[6] = systemId[4];
    systemId[5] = systemId[3];
    
    // set middle bytes to zero
    systemId[4] = 0;
    systemId[3] = 0;
    
    DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
  }
  
  gapProfileState = newState;
}

// start PTT Sampling
static void startPttSampling( void )
{  
  if(status == STATUS_PTT_STOP) 
  {
    status = STATUS_PTT_START;
    PTTFunc_SetPttSampling(true);
  }
}

// stop PTT Sampling
static void stopPttSampling( void )
{  
  if(status == STATUS_PTT_START)
  {
    status = STATUS_PTT_STOP;
    PTTFunc_SetPttSampling(false);
  }
}

static void pttServiceCB( uint8 event )
{
  switch (event)
  {
    case PTT_PACK_NOTI_ENABLED:
      startPttSampling();
      break;
        
    case PTT_PACK_NOTI_DISABLED:
      stopPttSampling();
      break;
      
    default:
      // Should not get here
      break;
  }
}