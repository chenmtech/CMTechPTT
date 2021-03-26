/*
* MAX30102: 脉搏和心率传感器MAX30102驱动
* Written by Chenm 2020-12-07
*
* 代码优化 by chenm 2021-03-16
*/


#include "hal_i2c.h"
#include "Dev_MAX30102.h"
#include "hal_mcu.h"
#include "CMUtil.h"

/*
* 常量
*/
//MAX30102的I2C地址
static const uint8 I2C_ADDR = 0x57;

// 部件ID，用来确认是不是MAX30102芯片
static const uint8 MAX30102_EXPECTEDPARTID = 0x15;

// 下面为寄存器地址，见pg 10表格
// 中断相关寄存器
static const uint8 MAX30102_INTSTAT1 =		0x00; // 中断寄存器1
static const uint8 MAX30102_INTSTAT2 =		0x01; // 中断寄存器2
static const uint8 MAX30102_INTENABLE1 =	0x02; // 中断使能1
static const uint8 MAX30102_INTENABLE2 =	0x03; // 中断使能2

// FIFO数据读写相关寄存器
static const uint8 MAX30102_FIFOWRITEPTR = 	0x04; // FIFO写指针
static const uint8 MAX30102_FIFOOVERFLOW = 	0x05; // 数据溢出计数器
static const uint8 MAX30102_FIFOREADPTR = 	0x06; // FIFO读指针
static const uint8 MAX30102_FIFODATA =		0x07; // FIFO数据寄存器

// 下面是Config寄存器
static const uint8 MAX30102_FIFOCONFIG = 	0x08; // FIFO缓存配置
static const uint8 MAX30102_MODECONFIG = 	0x09; // 工作模式配置
static const uint8 MAX30102_SPO2CONFIG = 	0x0A; // SPO2配置，用来设置信号的动态输出范围等
static const uint8 MAX30102_LED1_PULSEAMP = 	0x0C; // 红色LED的光脉冲幅度配置，用来调整供电电流大小
static const uint8 MAX30102_LED2_PULSEAMP = 	0x0D; // 红外LED的光脉冲幅度配置，用来调整供电电流大小
static const uint8 MAX30102_MULTILEDCONFIG1 =   0x11; // 多LED模式下的时间槽配置1
static const uint8 MAX30102_MULTILEDCONFIG2 =   0x12; // 多LED模式下的时间槽配置2

// 芯片温度相关寄存器
static const uint8 MAX30102_DIETEMPINT = 	0x1F; // 温度数据的整数部分
static const uint8 MAX30102_DIETEMPFRAC = 	0x20; // 温度数据的小数部分
static const uint8 MAX30102_DIETEMPCONFIG = 	0x21; // 温度配置，用来使能温度采集

// 部件ID寄存器
static const uint8 MAX30102_REVISIONID = 	0xFE;
static const uint8 MAX30102_PARTID = 		0xFF; // 内容必须是MAX30102_EXPECTEDPARTID，即0x15.



// 下面的常数用于实现对上述某个寄存器进行相应设置，以达到设置芯片功能的目的
// MASK用于指定清零寄存器的哪些位，紧接着MASK的值用于指定要设置寄存器的哪些位
// 具体是通过调用bitMask函数来设置的，请阅读函数bitMask的说明

// MAX30102_INTENABLE1和MAX30102_INTENABLE2寄存器相关设置 (见pg 12, 13)
// FIFO缓存“就快满了”almost-full中断的使能
static const uint8 MAX30102_INT_A_FULL_MASK =		(uint8)~0x80;
static const uint8 MAX30102_INT_A_FULL_ENABLE = 	0x80; // enable
static const uint8 MAX30102_INT_A_FULL_DISABLE = 	0x00; // disable

// “新数据已准备”中断的使能
static const uint8 MAX30102_INT_DATA_RDY_MASK =         (uint8)~0x40;
static const uint8 MAX30102_INT_DATA_RDY_ENABLE =	0x40; // enable
static const uint8 MAX30102_INT_DATA_RDY_DISABLE =      0x00; // disable

// “环境光消除功能达到最大极限“中断的使能，如果这个中断触发，说明环境光对信号输出产生了影响，否则，说明环境光已经被很好的消除了
static const uint8 MAX30102_INT_ALC_OVF_MASK =          (uint8)~0x20;
static const uint8 MAX30102_INT_ALC_OVF_ENABLE = 	0x20; // enable
static const uint8 MAX30102_INT_ALC_OVF_DISABLE =       0x00; // disable

// "温度转换完成，温度数据准备好"中断的使能
static const uint8 MAX30102_INT_DIE_TEMP_RDY_MASK =     (uint8)~0x02;
static const uint8 MAX30102_INT_DIE_TEMP_RDY_ENABLE =   0x02; // enable
static const uint8 MAX30102_INT_DIE_TEMP_RDY_DISABLE =  0x00; // disable

// MAX30102_FIFOCONFIG寄存器相关设置（见pg 17）
// 样本平均设置，如果设置了平均，比如设置了8个样本取平均，那么连续采集8个数据后，会取它们的平均值放入到FIFO中
// 样本平均的好处是：滤除了一定的高频噪声，同时减小了要传输的数据量
static const uint8 MAX30102_SAMPLEAVG_MASK =	(uint8)~0xE0;
static const uint8 MAX30102_SAMPLEAVG_1 = 	0x00; // 无平均
static const uint8 MAX30102_SAMPLEAVG_2 = 	0x20; // 2个样本取平均
static const uint8 MAX30102_SAMPLEAVG_4 = 	0x40; // 4个样本取平均
static const uint8 MAX30102_SAMPLEAVG_8 = 	0x60; // 8个样本取平均
static const uint8 MAX30102_SAMPLEAVG_16 = 	0x80; // 16个样本取平均
static const uint8 MAX30102_SAMPLEAVG_32 = 	0xA0; // 32个样本取平均

// FIFO缓存是否能翻滚
static const uint8 MAX30102_ROLLOVER_MASK = 	0xEF;
static const uint8 MAX30102_ROLLOVER_ENABLE =   0x10;
static const uint8 MAX30102_ROLLOVER_DISABLE =  0x00;

// FIFO缓存“就快满了”almost-full的剩余数据个数。因为不会用到，所以不提供设置的值
static const uint8 MAX30102_A_FULL_MASK = 	0xF0;

// MAX30102_MODECONFIG寄存器相关设置 (page 18)
// 关断或唤醒芯片
static const uint8 MAX30102_SHUTDOWN_MASK = 	0x7F;
static const uint8 MAX30102_SHUTDOWN = 		0x80; // 关断芯片，让其处于低功耗模式
static const uint8 MAX30102_WAKEUP = 		0x00; // 唤醒芯片

// 重启芯片
static const uint8 MAX30102_RESET_MASK = 	0xBF;
static const uint8 MAX30102_RESET = 		0x40;

// 芯片工作模式设置
static const uint8 MAX30102_MODE_MASK = 	0xF8;
static const uint8 MAX30102_MODE_REDONLY = 	0x02; // 只开红色LED，用于心率监测模式
static const uint8 MAX30102_MODE_REDIRONLY = 	0x03; // 开红色和红外LED，用于血氧监测模式
static const uint8 MAX30102_MODE_MULTILED = 	0x07; // 多LED模式，可灵活配置两个LED的开关

// MAX30102_SPO2CONFIG寄存器相关设置 (page 18)，注意这个寄存器的设置会影响PPG信号的质量

// ADC量化量程范围。光敏二极管把光强转换为光电流，ADC要采样量化这个光电流为数字信号
// 量程范围是指ADC能接收的最大光电流的值，比如量程范围指定为2.048uA，那么如果输入的光电流大于这个值，则ADC输出信号会饱和
// 如果量程范围指的太大，比如16.384uA，那么不会饱和。可是如果输入的光电流变化远远小于这个值，则导致ADC输出信号变化就很小。
static const uint8 MAX30102_ADCRANGE_MASK = 	0x9F;
static const uint8 MAX30102_ADCRANGE_2048 = 	0x00; // 2.048uA
static const uint8 MAX30102_ADCRANGE_4096 = 	0x20; // 4.096uA
static const uint8 MAX30102_ADCRANGE_8192 = 	0x40; // 8.192uA
static const uint8 MAX30102_ADCRANGE_16384 = 	0x60; // 16.384uA

// 采样率设置，注意采样率与下面的脉冲宽度PW的设置相关，如果采样率设置不满足要求，可能实际采样率不是你设置的。两者关系见pg23 表11,12
static const uint8 MAX30102_SAMPLERATE_MASK =   0xE3;
static const uint8 MAX30102_SAMPLERATE_50 = 	0x00; // 50Hz
static const uint8 MAX30102_SAMPLERATE_100 = 	0x04; // 100Hz
static const uint8 MAX30102_SAMPLERATE_200 = 	0x08; // 200Hz
static const uint8 MAX30102_SAMPLERATE_400 = 	0x0C; // 400Hz
static const uint8 MAX30102_SAMPLERATE_800 = 	0x10; // 800Hz
static const uint8 MAX30102_SAMPLERATE_1000 =   0x14; // 1kHz
static const uint8 MAX30102_SAMPLERATE_1600 =   0x18; // 1600Hz
static const uint8 MAX30102_SAMPLERATE_3200 =   0x1C; // 3200Hz

// LED的脉冲宽度PW，就是指LED要发光的通电时间长度
// PW越大，ADC完成信号采集量化的时间就越充足，输出数据的有效位数就会越多
// 注意，PW越大，功耗也会越大
static const uint8 MAX30102_PULSEWIDTH_MASK =   0xFC;
static const uint8 MAX30102_PULSEWIDTH_15 = 	0x00; // 15位
static const uint8 MAX30102_PULSEWIDTH_16 =     0x01; // 16位
static const uint8 MAX30102_PULSEWIDTH_17 =     0x02; // 17位
static const uint8 MAX30102_PULSEWIDTH_18 =     0x03; // 18位

// MAX30102_MULTILEDCONFIG1和MAX30102_MULTILEDCONFIG2寄存器的设置（pg 21）
static const uint8 MAX30102_SLOT1_MASK = 	0xF8;
static const uint8 MAX30102_SLOT2_MASK = 	0x8F;
static const uint8 MAX30102_SLOT3_MASK = 	0xF8;
static const uint8 MAX30102_SLOT4_MASK = 	0x8F;
static const uint8 SLOT_NONE = 			0x00; // 该时间槽没有LED点亮
static const uint8 SLOT_RED_LED = 		0x01; // 该时间槽点亮红色LED
static const uint8 SLOT_IR_LED = 		0x02; // 该时间槽点亮红外LED
static const uint8 SLOT_GREEN_LED = 		0x03; // 下面的不用管
static const uint8 SLOT_NONE_PILOT = 		0x04;
static const uint8 SLOT_RED_PILOT =		0x05;
static const uint8 SLOT_IR_PILOT = 		0x06;
static const uint8 SLOT_GREEN_PILOT = 		0x07;

/*
 * 局部函数，函数功能见后面的函数定义
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
* 公共函数
*/

// 判断是否上电
extern bool MAX30102_IsPowerOn()
{
  IIC_Enable(I2C_ADDR, i2cClock_267KHZ);
  uint8 intStatus1 = getINT1();
  return (intStatus1 & 0x01);
}

// 判断是否触发DATA RDY 中断
extern bool MAX30102_IsDATARDY()
{
  IIC_Enable(I2C_ADDR, i2cClock_267KHZ);
  uint8 intStatus1 = getINT1();
  return (intStatus1 & 0x40);
}

// 配置MAX30102
extern void MAX30102_Setup()
{
  IIC_Enable(I2C_ADDR, i2cClock_267KHZ);
  
  // 软重启芯片
  softReset();
  
  // 设置仅开启红色LED，即心率模式
  setLEDMode(MAX30102_MODE_REDONLY);
  // 设置时间槽
  //setSLOT1(SLOT_RED_LED);
  // 设置采样率为1kHz
  setSampleRate(MAX30102_SAMPLERATE_1000);
  // 设置样本平均个数为8，所以实际数据率为125Hz
  setFIFOAverage(MAX30102_SAMPLEAVG_8);
  
  // 设置LED脉冲宽度，改变ADC输出有效位数
  setPulseWidth(MAX30102_PULSEWIDTH_16);
  
  // 设置ADC光电流的量程范围, 单位nA
  setADCRange(MAX30102_ADCRANGE_2048);
  
  // 设置LED脉冲幅度，数值乘以0.2就是供电电流值mA
  setPulseAmplitudeRed(0x0A); 
  setPulseAmplitudeIR(0x0A);
  
  // 重置FIFO  
  clearFIFO();
}

// 唤醒MAX30102
extern void MAX30102_WakeUp()
{
  IIC_Enable(I2C_ADDR, i2cClock_267KHZ);
  wakeUp();
}

// 关停MAX30102，使其处于低功耗模式
extern void MAX30102_Shutdown()
{
  IIC_Enable(I2C_ADDR, i2cClock_267KHZ);
  shutDown();
}

// 启动数据采集
extern void MAX30102_Start()
{
  IIC_Enable(I2C_ADDR, i2cClock_267KHZ);
  enableDATARDY();
}

// 停止数据采集
extern void MAX30102_Stop()
{
  IIC_Enable(I2C_ADDR, i2cClock_267KHZ);
  disableDATARDY();
  clearFIFO();
}

// 读取芯片温度
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
* 局部函数定义
*/
// 向寄存器写入一个字节的值
// reg: 寄存器地址
// data: 要写入的值
static void writeOneByte(uint8 reg, uint8 data)
{
  uint8 buff[2] = {reg, data};
  IIC_Write(2, buff);
}

// 从寄存器读取一个字节的值
// reg: 寄存器的地址
// 返回读取的值
static uint8 readOneByte(uint8 reg)
{
  uint8 data = 0;
  IIC_Write(1, &reg);
  IIC_Read(1, &data);
  return data;
}

// 从寄存器读取多个字节的值
// reg: 寄存器地址
// len: 读取的字节数
// pBuff: 存储读取的数据缓存指针
static void readMultipleBytes(uint8 reg, uint8 len, uint8* pBuff)
{
  IIC_Write(1, &reg);
  IIC_Read(len, pBuff);
}

// 改变寄存器的某些位的值
// 分为三步：先读取寄存器的值，再用mask来清零这个值的某些位，然后用thing来设置这个值某些位，最后写入寄存器
// reg: 要设置的寄存器地址
// mask: 将寄存器中对应mask为0的位清零
// thing: 将寄存器中对应thing为1的位 置1
static void bitMask(uint8 reg, uint8 mask, uint8 thing)
{
  // Grab current register context
  uint8 originalContents = readOneByte(reg);

  // Zero-out the portions of the register we're interested in
  originalContents = originalContents & mask;

  // Change contents
  writeOneByte(reg, originalContents | thing);
}

// 获取MAX30102_INTSTAT1寄存器值
static uint8 getINT1(void) {
  return (readOneByte(MAX30102_INTSTAT1));
}

// 获取MAX30102_INTSTAT2寄存器值
static uint8 getINT2(void) {
  return (readOneByte(MAX30102_INTSTAT2));
}

// enable almost-full 中断
static void enableAFULL(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_ENABLE);
}

// disable almost-full 中断
static void disableAFULL(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_DISABLE);
}

// enable 新数据已准备 中断
static void enableDATARDY(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_ENABLE);
}

// disable 新数据已准备 中断
static void disableDATARDY(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_DISABLE);
}

// enable 环境光消除溢出 中断
static void enableALCOVF(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_ENABLE);
}

// diaable 环境光消除溢出 中断
static void disableALCOVF(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_DISABLE);
}

// enable 温度数据已准备 中断
static void enableDIETEMPRDY(void) {
  bitMask(MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_ENABLE);
}

// disable 温度数据已准备 中断
static void disableDIETEMPRDY(void) {
  bitMask(MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_DISABLE);
}

// 软重启MAX30102
static void softReset() {
  bitMask(MAX30102_MODECONFIG, MAX30102_RESET_MASK, MAX30102_RESET);

  while (true)
  {
    uint8 response = readOneByte(MAX30102_MODECONFIG);
    if ((response & MAX30102_RESET) == 0) break; //We're done!
    delayus(1000); //Let's not over burden the I2C bus
  }
}

// 关断MAX30102，让其处于低功耗模式
static void shutDown(void) {
  // Put IC into low power mode (datasheet pg. 19)
  // During shutdown the IC will continue to respond to I2C commands but will
  // not update with or take new readings (such as temperature)
  bitMask(MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_SHUTDOWN);
}

// 唤醒MAX30102, 不再处于低功耗模式
static void wakeUp(void) {
  // Pull IC out of low power mode (datasheet pg. 19)
  bitMask(MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_WAKEUP);
}

// 设置LED模式
static void setLEDMode(uint8 mode) {
  // Set which LEDs are used for sampling -- Red only, RED+IR only, or multi-LED.
  // See datasheet, page 18
  bitMask(MAX30102_MODECONFIG, MAX30102_MODE_MASK, mode);
}
// 设置时间槽
static void setSLOT1(uint8 SLOT1){
  //SLOT_NONE, SLOT_RED_LED, SLOT_IR_LED 
  bitMask(MAX30102_MULTILEDCONFIG1, MAX30102_SLOT1_MASK, SLOT1);
}
// 设置ADC量程范围
static void setADCRange(uint8 adcRange) {
  // adcRange: one of MAX30102_ADCRANGE_2048, _4096, _8192, _16384
  bitMask(MAX30102_SPO2CONFIG, MAX30102_ADCRANGE_MASK, adcRange);
}

// 设置ADC采样率
static void setSampleRate(uint8 sampleRate) {
  // sampleRate: one of MAX30102_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  bitMask(MAX30102_SPO2CONFIG, MAX30102_SAMPLERATE_MASK, sampleRate);
}

// 设置LED脉冲宽度，即ADC的有效位数
static void setPulseWidth(uint8 pulseWidth) {
  bitMask(MAX30102_SPO2CONFIG, MAX30102_PULSEWIDTH_MASK, pulseWidth);
}

// 设置LED脉冲幅度，即通过设置供电电流来设置LED发射光的强度
// NOTE: Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
// See datasheet, page 20, Table 8
// 红光LED脉冲幅度
static void setPulseAmplitudeRed(uint8 amplitude) {
  writeOneByte(MAX30102_LED1_PULSEAMP, amplitude);
}

// 红外LED脉冲幅度
static void setPulseAmplitudeIR(uint8 amplitude) {
  writeOneByte(MAX30102_LED2_PULSEAMP, amplitude);
}

//
// FIFO Configuration
//

// 设置样本平均个数
static void setFIFOAverage(uint8 numberOfSamples) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_SAMPLEAVG_MASK, numberOfSamples);
}

// 清除FIFO数据缓存
static void clearFIFO(void) {
  writeOneByte(MAX30102_FIFOWRITEPTR, 0);
  writeOneByte(MAX30102_FIFOOVERFLOW, 0);
  writeOneByte(MAX30102_FIFOREADPTR, 0);
}

//Enable FIFO 翻滚
static void enableFIFORollover(void) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_ENABLE);
}

//Disble FIFO 翻滚
static void disableFIFORollover(void) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_DISABLE);
}

//Set number of samples to trigger the almost full interrupt (Page 18)
//Power on default is 32 samples
//Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples
static void setFIFOAlmostFull(uint8 numberOfSamples) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_A_FULL_MASK, numberOfSamples);
}

// 读取MAX30102_FIFOWRITEPTR值
static uint8 getWritePointer() 
{
  return readOneByte(MAX30102_FIFOWRITEPTR);
}

// 读取MAX30102_FIFOREADPTR值
static uint8 getReadPointer() 
{
  return readOneByte(MAX30102_FIFOREADPTR);
}

static uint8 buff[3] = {0};

// 读取最新的一个PPG数据，假设只有一个通道数据
extern bool MAX30102_ReadPpgSample(uint16* pData)
{
  IIC_Enable(I2C_ADDR, i2cClock_267KHZ);
  
  /*
  uint8 ptRead = getReadPointer();
  uint8 ptWrite = getWritePointer();
  int8 num = ptWrite-ptRead;
  if(num == 0) return false;
  if(num < 0) num += 32; // 消除翻滚导致的ptWrite小于ptRead
  // 如果有多余的数据，读出丢弃
  while(num > 1)
  {
    readMultipleBytes(MAX30102_FIFODATA, 3, buff);
  }
  */
  
  // 读取数据
  readMultipleBytes(MAX30102_FIFODATA, 3, buff);  
  uint32 data32 = BUILD_UINT32(buff[2], buff[1], buff[0], 0x00);
  *pData = (uint16)(data32>>2);
  return true;
}