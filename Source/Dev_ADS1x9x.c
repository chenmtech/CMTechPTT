
#include "Dev_ADS1x9x.H"
#include "hal_mcu.h"
#include "CMUtil.h"
#include "CMTechPTT.h"
    
// all registers for outputing the test signal
const static uint8 ECGRegs125[12] = {  
  //DEVID
  0x52,
  //CONFIG1
  0x00,                     //continous sample,125sps
  //CONFIG2
  0xA0,                     //
  //LOFF
  0x10,                     //
  //CH1SET 
  0x60,                     //PGA=12，and ECG input
  //CH2SET
  0x80,                     //close CH2
  //RLD_SENS     
  0x23,                     //
  //LOFF_SENS (default)
  0x00,                     //disable channel 1 lead-off detect
  //LOFF_STAT
  0x00,                     //default
  //RESP1
  0x02,                     //
  //RESP2
  0x07,
  //GPIO
  0x0C                      //
};	

// all registers for outputing the normal ECG signal with 250 sample rate
const static uint8 ECGRegs250[12] = {  
  //DEVID
  0x52,
  //CONFIG1
  0x01,                     //continous sample,250sps
  //CONFIG2
  0xA0,                     //
  //LOFF
  0x10,                     //
  //CH1SET 
  0x60,                     //PGA=12，and ECG input
  //CH2SET
  0x80,                     //close CH2
  //RLD_SENS     
  0x23,                     //
  //LOFF_SENS (default)
  0x00,                     //disable channel 1 lead-off detect
  //LOFF_STAT
  0x00,                     //default
  //RESP1
  0x02,                     //
  //RESP2
  0x07,
  //GPIO
  0x0C                      //
};

//static uint8 data[2];
//static int16 * pEcg = (int16*)data;
static int ecgData;

static void execute(uint8 cmd); // execute command
static void setRegsAsNormalECGSignal(uint16 sampleRate); // set registers as outputing normal ECG signal
//static void readOneSampleUsingADS1291(void); // read one data with ADS1291
static void readOneSampleUsingADS1191(void); // read one data with ADS1191

// ADS init
extern void ADS1x9x_Init()
{
  // init ADS1x9x chip
  SPI_ADS_Init();
  
  ADS1x9x_PowerDown(); 
}

extern void ADS1x9x_PowerDown()
{
  ADS_RST_LOW();     //PWDN/RESET 低电平
  delayus(10000);
}

// wakeup
extern void ADS1x9x_WakeUp(void)
{
  execute(WAKEUP);
}

// enter in standby mode
extern void ADS1x9x_StandBy(void)
{
  execute(STANDBY);
}

// reset chip
extern void ADS1x9x_PowerUp(void)
{  
  ADS_RST_LOW();     //PWDN/RESET 低电平
  delayus(50);
  ADS_RST_HIGH();    //PWDN/RESET 高电平
  delayus(50);
  
  setRegsAsNormalECGSignal(125);
}

// start continuous sampling
extern void ADS1x9x_StartConvert(void)
{
  //设置连续采样模式
  ADS_CS_LOW();  
  delayus(100);
  SPI_ADS_SendByte(SDATAC);
  delayus(100);
  SPI_ADS_SendByte(RDATAC);  
  delayus(100);
  //ADS_CS_HIGH();  
  
  delayus(100);   
  
  //START 高电平
  ADS_START_HIGH();    
  delayus(32000); 
}

// stop continuous sampling
extern void ADS1x9x_StopConvert(void)
{
  //ADS_CS_LOW();  
  delayus(100);
  SPI_ADS_SendByte(SDATAC);
  delayus(100);
  ADS_CS_HIGH(); 

  delayus(100);  
  
  //START 低电平
  ADS_START_LOW();
  delayus(160); 
}

// read len registers into pRegs from the address beginaddr
extern void ADS1x9x_ReadMultipleRegister(uint8 beginaddr, uint8 * pRegs, uint8 len)
{
  ADS_CS_LOW();
  delayus(100);
  SPI_ADS_SendByte(SDATAC);
  delayus(100);
  SPI_ADS_SendByte(beginaddr | 0x20);               //发送RREG命令
  SPI_ADS_SendByte(len-1);                          //长度-1
  
  for(uint8 i = 0; i < len; i++)
    *(pRegs+i) = SPI_ADS_SendByte(ADS_DUMMY_CHAR);

  delayus(100);
  ADS_CS_HIGH();
}

// read a register with address
extern uint8 ADS1x9x_ReadRegister(uint8 address)
{
  uint8 result = 0;

  ADS_CS_LOW();
  delayus(100);
  SPI_ADS_SendByte(SDATAC);  
  delayus(100);
  SPI_ADS_SendByte(address | 0x20);         //发送RREG命令
  SPI_ADS_SendByte(0);                      //长度为1
  result = SPI_ADS_SendByte(ADS_DUMMY_CHAR);   //读寄存器
  
  delayus(100);
  ADS_CS_HIGH();
  return result;
}

// write all 12 registers
extern void ADS1x9x_WriteAllRegister(const uint8 * pRegs)
{
  ADS1x9x_WriteMultipleRegister(0x00, pRegs, 12);
}


// write multiple registers
extern void ADS1x9x_WriteMultipleRegister(uint8 beginaddr, const uint8 * pRegs, uint8 len)
{
  ADS_CS_LOW();
  delayus(100);
  SPI_ADS_SendByte(SDATAC);  
  delayus(100);
  SPI_ADS_SendByte(beginaddr | 0x40);
  SPI_ADS_SendByte(len-1);
  
  for(uint8 i = 0; i < len; i++)
    SPI_ADS_SendByte( *(pRegs+i) );
     
  delayus(100); 
  ADS_CS_HIGH();
} 

// write one register
extern void ADS1x9x_WriteRegister(uint8 address, uint8 onebyte)
{
  ADS_CS_LOW();
  delayus(100);
  SPI_ADS_SendByte(SDATAC);  
  delayus(100);
  SPI_ADS_SendByte(address | 0x40);
  SPI_ADS_SendByte(0);  
  SPI_ADS_SendByte(onebyte);
  
  delayus(100);
  ADS_CS_HIGH();
}  

// set registers as normal ecg mode
static void setRegsAsNormalECGSignal(uint16 sampleRate)
{
  if(sampleRate == 125)
    ADS1x9x_WriteAllRegister(ECGRegs125);
  else 
    ADS1x9x_WriteAllRegister(ECGRegs250);
}

//execute command
static void execute(uint8 cmd)
{
  ADS_CS_LOW();
  delayus(100);
  // 发送停止采样命令
  SPI_ADS_SendByte(SDATAC);
  // 发送当前命令
  SPI_ADS_SendByte(cmd);
  delayus(100);
  ADS_CS_HIGH();
}

extern bool ADS1x9x_ReadEcgSample(int16* pData)
{
  SPI_SEND(ADS_DUMMY_CHAR); 
  while (!U1TX_BYTE);
  U1TX_BYTE = 0;
  
  SPI_SEND(ADS_DUMMY_CHAR); 
  while (!U1TX_BYTE);
  U1TX_BYTE = 0;
  
  SPI_SEND(ADS_DUMMY_CHAR); 
  while (!U1TX_BYTE);
  U1TX_BYTE = 0;
  *((uint8*)&ecgData+1) = U1DBUF;
  
  SPI_SEND(ADS_DUMMY_CHAR); 
  while (!U1TX_BYTE);
  U1TX_BYTE = 0;  
  *((uint8*)&ecgData) = U1DBUF;  
  
}
