/*
* MAX30102: ���������ʴ�����MAX30102����
* Written by Chenm 2020-12-07
*
* �����Ż� by chenm 2021-03-16
*/


#include "hal_i2c.h"
#include "Dev_MAX30102.h"
#include "hal_mcu.h"
#include "CMUtil.h"

/*
* ����
*/
//MAX30102��I2C��ַ
static const uint8 I2C_ADDR = 0x57;

// ����ID������ȷ���ǲ���MAX30102оƬ
static const uint8 MAX30102_EXPECTEDPARTID = 0x15;

// ����Ϊ�Ĵ�����ַ����pg 10���
// �ж���ؼĴ���
static const uint8 MAX30102_INTSTAT1 =		0x00; // �жϼĴ���1
static const uint8 MAX30102_INTSTAT2 =		0x01; // �жϼĴ���2
static const uint8 MAX30102_INTENABLE1 =	0x02; // �ж�ʹ��1
static const uint8 MAX30102_INTENABLE2 =	0x03; // �ж�ʹ��2

// FIFO���ݶ�д��ؼĴ���
static const uint8 MAX30102_FIFOWRITEPTR = 	0x04; // FIFOдָ��
static const uint8 MAX30102_FIFOOVERFLOW = 	0x05; // �������������
static const uint8 MAX30102_FIFOREADPTR = 	0x06; // FIFO��ָ��
static const uint8 MAX30102_FIFODATA =		0x07; // FIFO���ݼĴ���

// ������Config�Ĵ���
static const uint8 MAX30102_FIFOCONFIG = 	0x08; // FIFO��������
static const uint8 MAX30102_MODECONFIG = 	0x09; // ����ģʽ����
static const uint8 MAX30102_SPO2CONFIG = 	0x0A; // SPO2���ã����������źŵĶ�̬�����Χ��
static const uint8 MAX30102_LED1_PULSEAMP = 	0x0C; // ��ɫLED�Ĺ�����������ã������������������С
static const uint8 MAX30102_LED2_PULSEAMP = 	0x0D; // ����LED�Ĺ�����������ã������������������С
static const uint8 MAX30102_MULTILEDCONFIG1 =   0x11; // ��LEDģʽ�µ�ʱ�������1
static const uint8 MAX30102_MULTILEDCONFIG2 =   0x12; // ��LEDģʽ�µ�ʱ�������2

// оƬ�¶���ؼĴ���
static const uint8 MAX30102_DIETEMPINT = 	0x1F; // �¶����ݵ���������
static const uint8 MAX30102_DIETEMPFRAC = 	0x20; // �¶����ݵ�С������
static const uint8 MAX30102_DIETEMPCONFIG = 	0x21; // �¶����ã�����ʹ���¶Ȳɼ�

// ����ID�Ĵ���
static const uint8 MAX30102_REVISIONID = 	0xFE;
static const uint8 MAX30102_PARTID = 		0xFF; // ���ݱ�����MAX30102_EXPECTEDPARTID����0x15.



// ����ĳ�������ʵ�ֶ�����ĳ���Ĵ���������Ӧ���ã��Դﵽ����оƬ���ܵ�Ŀ��
// MASK����ָ������Ĵ�������Щλ��������MASK��ֵ����ָ��Ҫ���üĴ�������Щλ
// ������ͨ������bitMask���������õģ����Ķ�����bitMask��˵��

// MAX30102_INTENABLE1��MAX30102_INTENABLE2�Ĵ���������� (��pg 12, 13)
// FIFO���桰�Ϳ����ˡ�almost-full�жϵ�ʹ��
static const uint8 MAX30102_INT_A_FULL_MASK =		(uint8)~0x80;
static const uint8 MAX30102_INT_A_FULL_ENABLE = 	0x80; // enable
static const uint8 MAX30102_INT_A_FULL_DISABLE = 	0x00; // disable

// ����������׼�����жϵ�ʹ��
static const uint8 MAX30102_INT_DATA_RDY_MASK =         (uint8)~0x40;
static const uint8 MAX30102_INT_DATA_RDY_ENABLE =	0x40; // enable
static const uint8 MAX30102_INT_DATA_RDY_DISABLE =      0x00; // disable

// ���������������ܴﵽ����ޡ��жϵ�ʹ�ܣ��������жϴ�����˵����������ź����������Ӱ�죬����˵���������Ѿ����ܺõ�������
static const uint8 MAX30102_INT_ALC_OVF_MASK =          (uint8)~0x20;
static const uint8 MAX30102_INT_ALC_OVF_ENABLE = 	0x20; // enable
static const uint8 MAX30102_INT_ALC_OVF_DISABLE =       0x00; // disable

// "�¶�ת����ɣ��¶�����׼����"�жϵ�ʹ��
static const uint8 MAX30102_INT_DIE_TEMP_RDY_MASK =     (uint8)~0x02;
static const uint8 MAX30102_INT_DIE_TEMP_RDY_ENABLE =   0x02; // enable
static const uint8 MAX30102_INT_DIE_TEMP_RDY_DISABLE =  0x00; // disable

// MAX30102_FIFOCONFIG�Ĵ���������ã���pg 17��
// ����ƽ�����ã����������ƽ��������������8������ȡƽ������ô�����ɼ�8�����ݺ󣬻�ȡ���ǵ�ƽ��ֵ���뵽FIFO��
// ����ƽ���ĺô��ǣ��˳���һ���ĸ�Ƶ������ͬʱ��С��Ҫ�����������
static const uint8 MAX30102_SAMPLEAVG_MASK =	(uint8)~0xE0;
static const uint8 MAX30102_SAMPLEAVG_1 = 	0x00; // ��ƽ��
static const uint8 MAX30102_SAMPLEAVG_2 = 	0x20; // 2������ȡƽ��
static const uint8 MAX30102_SAMPLEAVG_4 = 	0x40; // 4������ȡƽ��
static const uint8 MAX30102_SAMPLEAVG_8 = 	0x60; // 8������ȡƽ��
static const uint8 MAX30102_SAMPLEAVG_16 = 	0x80; // 16������ȡƽ��
static const uint8 MAX30102_SAMPLEAVG_32 = 	0xA0; // 32������ȡƽ��

// FIFO�����Ƿ��ܷ���
static const uint8 MAX30102_ROLLOVER_MASK = 	0xEF;
static const uint8 MAX30102_ROLLOVER_ENABLE =   0x10;
static const uint8 MAX30102_ROLLOVER_DISABLE =  0x00;

// FIFO���桰�Ϳ����ˡ�almost-full��ʣ�����ݸ�������Ϊ�����õ������Բ��ṩ���õ�ֵ
static const uint8 MAX30102_A_FULL_MASK = 	0xF0;

// MAX30102_MODECONFIG�Ĵ���������� (page 18)
// �ضϻ���оƬ
static const uint8 MAX30102_SHUTDOWN_MASK = 	0x7F;
static const uint8 MAX30102_SHUTDOWN = 		0x80; // �ض�оƬ�����䴦�ڵ͹���ģʽ
static const uint8 MAX30102_WAKEUP = 		0x00; // ����оƬ

// ����оƬ
static const uint8 MAX30102_RESET_MASK = 	0xBF;
static const uint8 MAX30102_RESET = 		0x40;

// оƬ����ģʽ����
static const uint8 MAX30102_MODE_MASK = 	0xF8;
static const uint8 MAX30102_MODE_REDONLY = 	0x02; // ֻ����ɫLED���������ʼ��ģʽ
static const uint8 MAX30102_MODE_REDIRONLY = 	0x03; // ����ɫ�ͺ���LED������Ѫ�����ģʽ
static const uint8 MAX30102_MODE_MULTILED = 	0x07; // ��LEDģʽ���������������LED�Ŀ���

// MAX30102_SPO2CONFIG�Ĵ���������� (page 18)��ע������Ĵ��������û�Ӱ��PPG�źŵ�����

// ADC�������̷�Χ�����������ܰѹ�ǿת��Ϊ�������ADCҪ����������������Ϊ�����ź�
// ���̷�Χ��ָADC�ܽ��յ����������ֵ���������̷�Χָ��Ϊ2.048uA����ô�������Ĺ�����������ֵ����ADC����źŻᱥ��
// ������̷�Χָ��̫�󣬱���16.384uA����ô���ᱥ�͡������������Ĺ�����仯ԶԶС�����ֵ������ADC����źű仯�ͺ�С��
static const uint8 MAX30102_ADCRANGE_MASK = 	0x9F;
static const uint8 MAX30102_ADCRANGE_2048 = 	0x00; // 2.048uA
static const uint8 MAX30102_ADCRANGE_4096 = 	0x20; // 4.096uA
static const uint8 MAX30102_ADCRANGE_8192 = 	0x40; // 8.192uA
static const uint8 MAX30102_ADCRANGE_16384 = 	0x60; // 16.384uA

// ���������ã�ע��������������������PW��������أ�������������ò�����Ҫ�󣬿���ʵ�ʲ����ʲ��������õġ����߹�ϵ��pg23 ��11,12
static const uint8 MAX30102_SAMPLERATE_MASK =   0xE3;
static const uint8 MAX30102_SAMPLERATE_50 = 	0x00; // 50Hz
static const uint8 MAX30102_SAMPLERATE_100 = 	0x04; // 100Hz
static const uint8 MAX30102_SAMPLERATE_200 = 	0x08; // 200Hz
static const uint8 MAX30102_SAMPLERATE_400 = 	0x0C; // 400Hz
static const uint8 MAX30102_SAMPLERATE_800 = 	0x10; // 800Hz
static const uint8 MAX30102_SAMPLERATE_1000 =   0x14; // 1kHz
static const uint8 MAX30102_SAMPLERATE_1600 =   0x18; // 1600Hz
static const uint8 MAX30102_SAMPLERATE_3200 =   0x1C; // 3200Hz

// LED��������PW������ָLEDҪ�����ͨ��ʱ�䳤��
// PWԽ��ADC����źŲɼ�������ʱ���Խ���㣬������ݵ���Чλ���ͻ�Խ��
// ע�⣬PWԽ�󣬹���Ҳ��Խ��
static const uint8 MAX30102_PULSEWIDTH_MASK =   0xFC;
static const uint8 MAX30102_PULSEWIDTH_15 = 	0x00; // 15λ
static const uint8 MAX30102_PULSEWIDTH_16 =     0x01; // 16λ
static const uint8 MAX30102_PULSEWIDTH_17 =     0x02; // 17λ
static const uint8 MAX30102_PULSEWIDTH_18 =     0x03; // 18λ

// MAX30102_MULTILEDCONFIG1��MAX30102_MULTILEDCONFIG2�Ĵ��������ã�pg 21��
static const uint8 MAX30102_SLOT1_MASK = 	0xF8;
static const uint8 MAX30102_SLOT2_MASK = 	0x8F;
static const uint8 MAX30102_SLOT3_MASK = 	0xF8;
static const uint8 MAX30102_SLOT4_MASK = 	0x8F;
static const uint8 SLOT_NONE = 			0x00; // ��ʱ���û��LED����
static const uint8 SLOT_RED_LED = 		0x01; // ��ʱ��۵�����ɫLED
static const uint8 SLOT_IR_LED = 		0x02; // ��ʱ��۵�������LED
static const uint8 SLOT_GREEN_LED = 		0x03; // ����Ĳ��ù�
static const uint8 SLOT_NONE_PILOT = 		0x04;
static const uint8 SLOT_RED_PILOT =		0x05;
static const uint8 SLOT_IR_PILOT = 		0x06;
static const uint8 SLOT_GREEN_PILOT = 		0x07;

/*
 * �ֲ��������������ܼ�����ĺ�������
*/
static void writeOneByte(uint8 reg, uint8 data);
static uint8 readOneByte(uint8 reg);
static void readMultipleBytes(uint8 reg, uint8 len, uint8* pBuff);
static void bitMask(uint8 reg, uint8 mask, uint8 thing);
static uint8 getINT1(void);
static uint8 getINT2(void);
static void softReset();
static void setLEDMode(uint8 mode);
static void clearFIFO(void);
static void shutDown(void);
static void wakeUp(void);
static void enableDATARDY(void);
static void disableDATARDY(void);
static void setFIFOAverage(uint8 numberOfSamples);
static void setADCRange(uint8 adcRange);
static void setSampleRate(uint8 sampleRate);
static void setPulseWidth(uint8 pulseWidth);
static void setPulseAmplitudeRed(uint8 amplitude);
static void setPulseAmplitudeIR(uint8 amplitude);
static void setSLOT1(uint8 SLOT1);
//static void setINTPin();

/*
* ��������
*/

// �ж��Ƿ��ϵ�
extern bool MAX30102_IsPowerOn()
{
  IIC_Enable(I2C_ADDR, i2cClock_267KHZ);
  uint8 intStatus1 = getINT1();
  return (intStatus1 & 0x01);
}

// �ж��Ƿ񴥷�DATA RDY �ж�
extern bool MAX30102_IsDATARDY()
{
  IIC_Enable(I2C_ADDR, i2cClock_267KHZ);
  uint8 intStatus1 = getINT1();
  return (intStatus1 & 0x40);
}

// ����MAX30102
extern void MAX30102_Setup()
{
  IIC_Enable(I2C_ADDR, i2cClock_267KHZ);
  
  // ������оƬ
  softReset();
  
  // ���ý�������ɫLED��������ģʽ
  setLEDMode(MAX30102_MODE_REDONLY);
  // ����ʱ���
  //setSLOT1(SLOT_RED_LED);
  // ���ò�����Ϊ1kHz
  setSampleRate(MAX30102_SAMPLERATE_1000);
  // ��������ƽ������Ϊ8������ʵ��������Ϊ125Hz
  setFIFOAverage(MAX30102_SAMPLEAVG_8);
  
  // ����LED�����ȣ��ı�ADC�����Чλ��
  setPulseWidth(MAX30102_PULSEWIDTH_16);
  
  // ����ADC����������̷�Χ, ��λnA
  setADCRange(MAX30102_ADCRANGE_2048);
  
  // ����LED������ȣ���ֵ����0.2���ǹ������ֵmA
  setPulseAmplitudeRed(0x0A); 
  setPulseAmplitudeIR(0x0A);
  
  // ����FIFO  
  clearFIFO();
}

// ����MAX30102
extern void MAX30102_WakeUp()
{
  IIC_Enable(I2C_ADDR, i2cClock_267KHZ);
  wakeUp();
}

// ��ͣMAX30102��ʹ�䴦�ڵ͹���ģʽ
extern void MAX30102_Shutdown()
{
  IIC_Enable(I2C_ADDR, i2cClock_267KHZ);
  shutDown();
}

// �������ݲɼ�
extern void MAX30102_Start()
{
  IIC_Enable(I2C_ADDR, i2cClock_267KHZ);
  enableDATARDY();
}

// ֹͣ���ݲɼ�
extern void MAX30102_Stop()
{
  IIC_Enable(I2C_ADDR, i2cClock_267KHZ);
  disableDATARDY();
  clearFIFO();
}

// ��ȡоƬ�¶�
extern float MAX30102_ReadTemperature() {
  // Step 1: Config die temperature register to take 1 temperature sample
  writeOneByte(MAX30102_DIETEMPCONFIG, 0x01);

  // Poll for bit to clear, reading is then complete
  while (true)
  {
    uint8 response = readOneByte(MAX30102_DIETEMPCONFIG);
    if ((response & 0x01) == 0) break; //We're done!
    delayus(1000); //Let's not over burden the I2C bus
  }

  // Step 2: Read die temperature register (integer)
  int8 tempInt = readOneByte(MAX30102_DIETEMPINT);
  uint8 tempFrac = readOneByte(MAX30102_DIETEMPFRAC);

  // Step 3: Calculate temperature (datasheet pg. 23)
  return (float)tempInt + ((float)tempFrac * 0.0625);
}


/*
* �ֲ���������
*/
// ��Ĵ���д��һ���ֽڵ�ֵ
// reg: �Ĵ�����ַ
// data: Ҫд���ֵ
static void writeOneByte(uint8 reg, uint8 data)
{
  uint8 buff[2] = {reg, data};
  IIC_Write(2, buff);
}

// �ӼĴ�����ȡһ���ֽڵ�ֵ
// reg: �Ĵ����ĵ�ַ
// ���ض�ȡ��ֵ
static uint8 readOneByte(uint8 reg)
{
  uint8 data = 0;
  IIC_Write(1, &reg);
  IIC_Read(1, &data);
  return data;
}

// �ӼĴ�����ȡ����ֽڵ�ֵ
// reg: �Ĵ�����ַ
// len: ��ȡ���ֽ���
// pBuff: �洢��ȡ�����ݻ���ָ��
static void readMultipleBytes(uint8 reg, uint8 len, uint8* pBuff)
{
  IIC_Write(1, &reg);
  IIC_Read(len, pBuff);
}

// �ı�Ĵ�����ĳЩλ��ֵ
// ��Ϊ�������ȶ�ȡ�Ĵ�����ֵ������mask���������ֵ��ĳЩλ��Ȼ����thing���������ֵĳЩλ�����д��Ĵ���
// reg: Ҫ���õļĴ�����ַ
// mask: ���Ĵ����ж�ӦmaskΪ0��λ����
// thing: ���Ĵ����ж�ӦthingΪ1��λ ��1
static void bitMask(uint8 reg, uint8 mask, uint8 thing)
{
  // Grab current register context
  uint8 originalContents = readOneByte(reg);

  // Zero-out the portions of the register we're interested in
  originalContents = originalContents & mask;

  // Change contents
  writeOneByte(reg, originalContents | thing);
}

// ��ȡMAX30102_INTSTAT1�Ĵ���ֵ
static uint8 getINT1(void) {
  return (readOneByte(MAX30102_INTSTAT1));
}

// ��ȡMAX30102_INTSTAT2�Ĵ���ֵ
static uint8 getINT2(void) {
  return (readOneByte(MAX30102_INTSTAT2));
}

// enable almost-full �ж�
static void enableAFULL(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_ENABLE);
}

// disable almost-full �ж�
static void disableAFULL(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_DISABLE);
}

// enable ��������׼�� �ж�
static void enableDATARDY(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_ENABLE);
}

// disable ��������׼�� �ж�
static void disableDATARDY(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_DISABLE);
}

// enable ������������� �ж�
static void enableALCOVF(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_ENABLE);
}

// diaable ������������� �ж�
static void disableALCOVF(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_DISABLE);
}

// enable �¶�������׼�� �ж�
static void enableDIETEMPRDY(void) {
  bitMask(MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_ENABLE);
}

// disable �¶�������׼�� �ж�
static void disableDIETEMPRDY(void) {
  bitMask(MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_DISABLE);
}

// ������MAX30102
static void softReset() {
  bitMask(MAX30102_MODECONFIG, MAX30102_RESET_MASK, MAX30102_RESET);

  while (true)
  {
    uint8 response = readOneByte(MAX30102_MODECONFIG);
    if ((response & MAX30102_RESET) == 0) break; //We're done!
    delayus(1000); //Let's not over burden the I2C bus
  }
}

// �ض�MAX30102�����䴦�ڵ͹���ģʽ
static void shutDown(void) {
  // Put IC into low power mode (datasheet pg. 19)
  // During shutdown the IC will continue to respond to I2C commands but will
  // not update with or take new readings (such as temperature)
  bitMask(MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_SHUTDOWN);
}

// ����MAX30102, ���ٴ��ڵ͹���ģʽ
static void wakeUp(void) {
  // Pull IC out of low power mode (datasheet pg. 19)
  bitMask(MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_WAKEUP);
}

// ����LEDģʽ
static void setLEDMode(uint8 mode) {
  // Set which LEDs are used for sampling -- Red only, RED+IR only, or multi-LED.
  // See datasheet, page 18
  bitMask(MAX30102_MODECONFIG, MAX30102_MODE_MASK, mode);
}
// ����ʱ���
static void setSLOT1(uint8 SLOT1){
  //SLOT_NONE, SLOT_RED_LED, SLOT_IR_LED 
  bitMask(MAX30102_MULTILEDCONFIG1, MAX30102_SLOT1_MASK, SLOT1);
}
// ����ADC���̷�Χ
static void setADCRange(uint8 adcRange) {
  // adcRange: one of MAX30102_ADCRANGE_2048, _4096, _8192, _16384
  bitMask(MAX30102_SPO2CONFIG, MAX30102_ADCRANGE_MASK, adcRange);
}

// ����ADC������
static void setSampleRate(uint8 sampleRate) {
  // sampleRate: one of MAX30102_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  bitMask(MAX30102_SPO2CONFIG, MAX30102_SAMPLERATE_MASK, sampleRate);
}

// ����LED�����ȣ���ADC����Чλ��
static void setPulseWidth(uint8 pulseWidth) {
  bitMask(MAX30102_SPO2CONFIG, MAX30102_PULSEWIDTH_MASK, pulseWidth);
}

// ����LED������ȣ���ͨ�����ù������������LED������ǿ��
// NOTE: Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
// See datasheet, page 20, Table 8
// ���LED�������
static void setPulseAmplitudeRed(uint8 amplitude) {
  writeOneByte(MAX30102_LED1_PULSEAMP, amplitude);
}

// ����LED�������
static void setPulseAmplitudeIR(uint8 amplitude) {
  writeOneByte(MAX30102_LED2_PULSEAMP, amplitude);
}

//
// FIFO Configuration
//

// ��������ƽ������
static void setFIFOAverage(uint8 numberOfSamples) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_SAMPLEAVG_MASK, numberOfSamples);
}

// ���FIFO���ݻ���
static void clearFIFO(void) {
  writeOneByte(MAX30102_FIFOWRITEPTR, 0);
  writeOneByte(MAX30102_FIFOOVERFLOW, 0);
  writeOneByte(MAX30102_FIFOREADPTR, 0);
}

//Enable FIFO ����
static void enableFIFORollover(void) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_ENABLE);
}

//Disble FIFO ����
static void disableFIFORollover(void) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_DISABLE);
}

//Set number of samples to trigger the almost full interrupt (Page 18)
//Power on default is 32 samples
//Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples
static void setFIFOAlmostFull(uint8 numberOfSamples) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_A_FULL_MASK, numberOfSamples);
}

// ��ȡMAX30102_FIFOWRITEPTRֵ
static uint8 getWritePointer() 
{
  return readOneByte(MAX30102_FIFOWRITEPTR);
}

// ��ȡMAX30102_FIFOREADPTRֵ
static uint8 getReadPointer() 
{
  return readOneByte(MAX30102_FIFOREADPTR);
}

static uint8 buff[3] = {0};

// ��ȡ���µ�һ��PPG���ݣ�����ֻ��һ��ͨ������
extern bool MAX30102_ReadPpgSample(uint16* pData)
{
  IIC_Enable(I2C_ADDR, i2cClock_267KHZ);
  
  /*
  uint8 ptRead = getReadPointer();
  uint8 ptWrite = getWritePointer();
  int8 num = ptWrite-ptRead;
  if(num == 0) return false;
  if(num < 0) num += 32; // �����������µ�ptWriteС��ptRead
  // ����ж�������ݣ���������
  while(num > 1)
  {
    readMultipleBytes(MAX30102_FIFODATA, 3, buff);
  }
  */
  
  // ��ȡ����
  readMultipleBytes(MAX30102_FIFODATA, 3, buff);  
  uint32 data32 = BUILD_UINT32(buff[2], buff[1], buff[0], 0x00);
  *pData = (uint16)(data32>>2);
  return true;
}