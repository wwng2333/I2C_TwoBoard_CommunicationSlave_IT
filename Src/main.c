/**
 ******************************************************************************
 * @file    main.c
 * @author  MCU Application Team
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
 * All rights reserved.</center></h2>
 *
 * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "py32f002xx_ll_Start_Kit.h"
/* Private define ------------------------------------------------------------*/
#define I2C_ADDRESS 0xA0      /* 本机\从机地址 */
#define I2C_SPEEDCLOCK 100000 /* 通讯速度100K */
#define I2C_STATE_READY 0     /* 就绪状态 */
#define I2C_STATE_BUSY_TX 1   /* 发送状态 */
#define I2C_STATE_BUSY_RX 2   /* 接收状态 */
// ADC
uint8_t ADC_Result[10] = {0};
uint8_t ADC_Count = 0;
uint32_t ADC_CRC = 0;

/* Private variables ---------------------------------------------------------*/
uint8_t aTxBuffer[15] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
uint8_t aRxBuffer[100] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const static uint16_t table[256] = { 0x0000, 0xa001, 0xe003, 0x4002, 0x6007, 0xc006, 0x8004, 0x2005, 0xc00e, 0x600f, 0x200d, 0x800c, 0xa009, 0x8, 0x400a, 0xe00b, 0x201d, 0x801c, 0xc01e, 0x601f,
                                     0x401a, 0xe01b, 0xa019, 0x0018, 0xe013, 0x4012, 0x0010, 0xa011, 0x8014, 0x2015, 0x6017, 0xc016, 0x403a, 0xe03b, 0xa039, 0x38, 0x203d, 0x803c, 0xc03e, 0x603f,
                                     0x8034, 0x2035, 0x6037, 0xc036, 0xe033, 0x4032, 0x0030, 0xa031, 0x6027, 0xc026, 0x8024, 0x2025, 0x20, 0xa021, 0xe023, 0x4022, 0xa029, 0x28, 0x402a, 0xe02b,
                                     0xc02e, 0x602f, 0x202d, 0x802c, 0x8074, 0x2075, 0x6077, 0xc076, 0xe073, 0x4072, 0x70, 0xa071, 0x407a, 0xe07b, 0xa079, 0x78, 0x207d, 0x807c, 0xc07e, 0x607f,
                                     0xa069, 0x0068, 0x406a, 0xe06b, 0xc06e, 0x606f, 0x206d, 0x806c, 0x6067, 0xc066, 0x8064, 0x2065, 0x60, 0xa061, 0xe063, 0x4062, 0xc04e, 0x604f, 0x204d, 0x804c,
                                     0xa049, 0x0048, 0x404a, 0xe04b, 0x0040, 0xa041, 0xe043, 0x4042, 0x6047, 0xc046, 0x8044, 0x2045, 0xe053, 0x4052, 0x50, 0xa051, 0x8054, 0x2055, 0x6057, 0xc056,
                                     0x205d, 0x805c, 0xc05e, 0x605f, 0x405a, 0xe05b, 0xa059, 0x58, 0xa0e9, 0xe8, 0x40ea, 0xe0eb, 0xc0ee, 0x60ef, 0x20ed, 0x80ec, 0x60e7, 0xc0e6, 0x80e4, 0x20e5,
                                     0x00e0, 0xa0e1, 0xe0e3, 0x40e2, 0x80f4, 0x20f5, 0x60f7, 0xc0f6, 0xe0f3, 0x40f2, 0xf0, 0xa0f1, 0x40fa, 0xe0fb, 0xa0f9, 0xf8, 0x20fd, 0x80fc, 0xc0fe, 0x60ff,
                                     0xe0d3, 0x40d2, 0x00d0, 0xa0d1, 0x80d4, 0x20d5, 0x60d7, 0xc0d6, 0x20dd, 0x80dc, 0xc0de, 0x60df, 0x40da, 0xe0db, 0xa0d9, 0xd8, 0xc0ce, 0x60cf, 0x20cd, 0x80cc, 0xa0c9, 0xc8,
                                     0x40ca, 0xe0cb, 0x00c0, 0xa0c1, 0xe0c3, 0x40c2, 0x60c7, 0xc0c6, 0x80c4, 0x20c5, 0x209d, 0x809c, 0xc09e, 0x609f, 0x409a, 0xe09b, 0xa099, 0x98, 0xe093, 0x4092, 0x90, 0xa091,
                                     0x8094, 0x2095, 0x6097, 0xc096, 0x0080, 0xa081, 0xe083, 0x4082, 0x6087, 0xc086, 0x8084, 0x2085, 0xc08e, 0x608f, 0x208d, 0x808c, 0xa089, 0x88, 0x408a, 0xe08b, 0x60a7, 0xc0a6,
                                     0x80a4, 0x20a5, 0x00a0, 0xa0a1, 0xe0a3, 0x40a2, 0xa0a9, 0xa8, 0x40aa, 0xe0ab, 0xc0ae, 0x60af, 0x20ad, 0x80ac, 0x40ba, 0xe0bb, 0xa0b9, 0xb8, 0x20bd, 0x80bc, 0xc0be, 0x60bf,
                                     0x80b4, 0x20b5, 0x60b7, 0xc0b6, 0xe0b3, 0x40b2, 0x00b0, 0xa0b1 };

__IO uint8_t counter = 0;
__IO uint8_t I2CSendPointer = 0x00, rcv_counter = 0;
uint8_t aADCRefresh = 0;
uint32_t aADCCounter = 0;

uint8_t *pBuffPtr = NULL;
__IO uint16_t XferCount = 0;
__IO uint32_t State = I2C_STATE_READY;
/* Private function prototypes -----------------------------------------------*/
void APP_SystemClockConfig(void);
static void APP_ConfigI2cSlave(void);
static void APP_SlaveTransmit_IT(uint8_t *pData, uint16_t Size);
static void APP_SlaveReceive_IT(uint8_t *pData, uint16_t Size);
static uint32_t APP_CalculateCRC(uint32_t pBuffer[],uint32_t BufferLength);
static void APP_AdcConfig(void);
static void APP_AdcEnable(void);
static void APP_AdcCalibrate(void);
static void APP_TimerInit(void);

/**
 * @brief  应用程序入口函数.
 * @param  无
 * @retval int
 */
int main(void)
{
  /* 配置系统时钟 */
  APP_SystemClockConfig();

  /*ADC复位*/
  LL_ADC_Reset(ADC1);

  /* ADC模块时钟使能 */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

	//使能CRC模块时钟
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);
	
  /* ADC校准 */
  APP_AdcCalibrate();

  /* 配置ADC相关参数 */
  APP_AdcConfig();

  /* ADC使能 */
  APP_AdcEnable();

	
  /* 配置I2C */
  APP_ConfigI2cSlave();
  SEGGER_RTT_printf(0, "Init ok.\r\n");

  LL_ADC_REG_StartConversion(ADC1);
  //  /* 从机发送数据 */
  APP_SlaveTransmit_IT((uint8_t *)aTxBuffer, sizeof(aTxBuffer));
  //
  //  /* 等待从机发送数据完成 */
  //  while (State != I2C_STATE_READY);
  // APP_SlaveReceive_IT((uint8_t *)aRxBuffer, sizeof(aRxBuffer));
	APP_TimerInit();
  while (1)
  {
		//if(LL_ADC_IsActiveFlag_EOS(ADC1)) LL_ADC_ClearFlag_EOS(ADC1);
		//LL_ADC_REG_StartConversion(ADC1);
		//LL_mDelay(100);
		
//    if (aADCRefresh == 0x04)
//    {
//      LL_ADC_REG_StartConversion(ADC1);
//      aADCRefresh = 0x00;
//    }
    //  APP_SlaveReceive_IT((uint8_t *)aRxBuffer, sizeof(aRxBuffer));
    //
    //  /* 等待从机接收数据完成 */
    //  APP_SlaveTransmit_IT((uint8_t *)aTxBuffer, sizeof(aTxBuffer));
  }
}

/**
  * @brief  CRC值计算函数.
  * @param  CRC数组,CRC数组长度
  * @retval CRC值
  */
uint16_t crc16(uint8_t* data, uint8_t len, uint16_t* table) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc = (crc << 8) ^ table[((crc >> 8) ^ data[i]) & 0xFF];
  }
  return crc;
}

/**
  * @brief  TIM配置函数
  * @param  无
  * @retval 无
  */
void APP_TimerInit()
{
  /* 使能TIM1时钟 */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

  /* 设置TIM1预分频器 */
  LL_TIM_SetPrescaler(TIM1,4000);

  /* 设置TIM1自动重装载值 */
  LL_TIM_SetAutoReload(TIM1, 80000);

  /* 设置TIM1更新触发 */
  LL_TIM_SetTriggerOutput(TIM1,LL_TIM_TRGO_UPDATE);

  /* 使能TIM1 */
  LL_TIM_EnableCounter(TIM1);
}

void uint32_to_uint8(uint32_t input, uint8_t output[4]) {
    for (int i = 0; i < 4; i++) {
        output[i] = (input >> (i * 8)) & 0xFF;
    }
}

void APP_AdcGrpRegularUnitaryConvCompleteCallback()
{
  uint16_t temp = 0;
	uint8_t i=0, j=8;
  float tmp = 0;
  if (ADC_Count == 4)
  {
    tmp = __LL_ADC_CALC_TEMPERATURE(((int32_t)3000), LL_ADC_REG_ReadConversionData12(ADC1), LL_ADC_RESOLUTION_12B);
    tmp *= 100;
    temp = (uint16_t)tmp;
  }
  else
  {
    temp = LL_ADC_REG_ReadConversionData12(ADC1);
  }
  ADC_Result[ADC_Count++] = (uint8_t)temp;
  ADC_Result[ADC_Count++] = temp >> 8;
  //ADC_Count += 2;  
	if (ADC_Count == 8)
  {
		ADC_CRC = crc16((uint8_t*)ADC_Result, 8, (uint16_t*)table);
		ADC_Result[8] = ADC_CRC;
		ADC_Result[9] = ADC_CRC >> 8;
		SEGGER_RTT_printf(0, "crc=0x%x\r\n", ADC_CRC);
    ADC_Count = 0;
  }
	aADCCounter++;
}

/**
 * @brief  ADC配置函数.
 * @param  无
 * @retval 无
 */
static void APP_AdcConfig(void)
{

  __IO uint32_t wait_loop_index = 0;

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1 | LL_GPIO_PIN_6, LL_GPIO_MODE_ANALOG);

  LL_ADC_InitTypeDef ADC_Init;
  LL_ADC_REG_InitTypeDef LL_ADC_REG_InitType;
  ADC_Common_TypeDef ADC_Common_Type;

  /*ADC通道和时钟源需在ADEN=0时配置，其余的需在ADSTART=0时配置*/
  /*ADC部分功能初始化*/
  ADC_Init.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
  ADC_Init.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_Init.LowPowerMode = LL_ADC_LP_MODE_NONE;
  ADC_Init.Resolution = LL_ADC_RESOLUTION_12B;
  LL_ADC_Init(ADC1, &ADC_Init);
  /* 设置通道转换时间 */
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_239CYCLES_5);

  /* ADC结构功能初始化 */
  LL_ADC_REG_InitType.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  LL_ADC_REG_InitType.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_InitType.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  //LL_ADC_REG_InitType.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  LL_ADC_REG_InitType.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM1_TRGO;
  LL_ADC_REG_Init(ADC1, &LL_ADC_REG_InitType);
  /* ADC共用参数设置 */
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_VREFINT | LL_ADC_PATH_INTERNAL_TEMPSENSOR);
  /* ADC TempSensor 等待稳定 */
  wait_loop_index = ((LL_ADC_DELAY_TEMPSENSOR_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while (wait_loop_index != 0)
  {
    wait_loop_index--;
  }
  /* 设置转换通道 */
  // LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_TEMPSENSOR | LL_ADC_CHANNEL_VREFINT | LL_ADC_CHANNEL_1 | LL_ADC_CHANNEL_6);
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_TEMPSENSOR | LL_ADC_CHANNEL_VREFINT | LL_ADC_CHANNEL_1 | LL_ADC_CHANNEL_6);
  LL_ADC_EnableIT_EOC(ADC1);
  NVIC_SetPriority(ADC_COMP_IRQn, 0);
  NVIC_EnableIRQ(ADC_COMP_IRQn);
}
/**
 * @brief  ADC校准函数.
 * @param  无
 * @retval 无
 */
static void APP_AdcCalibrate(void)
{
  __IO uint32_t wait_loop_index = 0;
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0; /* Variable used for timeout management */
#endif                  /* USE_TIMEOUT */

  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* 使能校准 */
    LL_ADC_StartCalibration(ADC1);

#if (USE_TIMEOUT == 1)
    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
#endif /* USE_TIMEOUT */

    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
    {
#if (USE_TIMEOUT == 1)
      /* 检测校准是否超时 */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if (Timeout-- == 0)
        {
        }
      }
#endif /* USE_TIMEOUT */
    }

    /* ADC校准结束和使能ADC之间的延时最低4个ADC Clock */
    LL_mDelay(1);
  }
}

/**
 * @brief  ADC使能函数
 * @param  无
 * @retval 无
 */
static void APP_AdcEnable(void)
{
  /* 使能ADC */
  LL_ADC_Enable(ADC1);

  /* 使能ADC 稳定时间，最低8个ADC Clock */
  LL_mDelay(1);
}

/**
 * @brief  系统时钟配置函数
 * @param  无
 * @retval 无
 */
void APP_SystemClockConfig(void)
{
  /* 使能HSI */
  LL_RCC_HSI_Enable();
  while (LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* 设置 AHB 分频*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* 配置HSISYS作为系统时钟源 */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
  {
  }

  /* 设置 APB1 分频*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(8000000);

  /* 更新系统时钟全局变量SystemCoreClock(也可以通过调用SystemCoreClockUpdate函数更新) */
  LL_SetSystemCoreClock(8000000);
}
/**
 * @brief  I2C配置函数
 * @param  无
 * @retval 无
 */
static void APP_ConfigI2cSlave(void)
{
  /* 使能 GPIOA 的外设时钟 */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);

  /* 启用 I2C1 的外设时钟 */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* 将 SCL 引脚配置为：可选功能、高速、开漏、上拉 */
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* 将 SDA 引脚配置为：可选功能、高速、开漏、上拉 */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* 复位I2C */
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C1);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C1);

  /* 使能NVIC中断 */
  NVIC_SetPriority(I2C1_IRQn, 0);
  NVIC_EnableIRQ(I2C1_IRQn);

  /* I2C初始化 */
  LL_I2C_InitTypeDef I2C_InitStruct;
  I2C_InitStruct.ClockSpeed = I2C_SPEEDCLOCK;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_16_9;
  I2C_InitStruct.OwnAddress1 = I2C_ADDRESS;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_NACK;
  LL_I2C_Init(I2C1, &I2C_InitStruct);

  /* 启用时钟拉伸 */
  /* 复位值是启用时钟延长 */
  /*LL_I2C_EnableClockStretching(I2C1);*/

  /* 启用广播呼叫 */
  /* 复位值为禁用广播呼叫 */
  /*LL_I2C_EnableGeneralCall(I2C1);*/
}

/**
 * @brief  I2C发送函数
 * @param  DevAddress：从机地址；
 **          pData：要发送数据指针；
 **          Size：要发送数据大小
 * @retval 无
 */
static void APP_SlaveTransmit_IT(uint8_t *pData, uint16_t Size)
{
  /* 清pos */
  LL_I2C_DisableBitPOS(I2C1);

  /*要发送数据、数据大小、状态赋给全局变量 */
  pBuffPtr = pData;
  XferCount = Size;
  State = I2C_STATE_BUSY_TX;

  /* 使能应答 */
  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);

  /* 使能中断 */
  LL_I2C_EnableIT_EVT(I2C1);
  LL_I2C_EnableIT_BUF(I2C1);
  LL_I2C_EnableIT_ERR(I2C1);
}
/**
 * @brief  I2C接收函数
 * @param  DevAddress：从机地址；
 **          pData：要发送数据指针；
 **          Size：要发送数据大小
 * @retval 无
 */
static void APP_SlaveReceive_IT(uint8_t *pData, uint16_t Size)
{
  /* 清pos */
  LL_I2C_DisableBitPOS(I2C1);

  /* 要发送数据、数据大小赋给全局变量 */
  pBuffPtr = pData;
  XferCount = Size;
  State = I2C_STATE_BUSY_RX;

  /* 使能应答 */
  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);

  /* 使能中断 */
  LL_I2C_EnableIT_EVT(I2C1);
  LL_I2C_EnableIT_BUF(I2C1);
  LL_I2C_EnableIT_ERR(I2C1);
}

/**
 * @brief  I2C中断回调函数
 * @param  无
 * @retval 无
 */
void APP_SlaveIRQCallback(void)
{
  /* ADDR标志位置位 */
  //   从机地址已经匹配                            中断已使能
  if ((LL_I2C_IsActiveFlag_ADDR(I2C1) == 1) && (LL_I2C_IsEnabledIT_EVT(I2C1) == 1))
  {
    // SEGGER_RTT_printf(0, "I2C IRQCallback:");
    //  清 地址匹配中断
    LL_I2C_ClearFlag_ADDR(I2C1);
    return;
  }
  /* STOPF 标志位置位 */
  else if (LL_I2C_IsActiveFlag_STOP(I2C1) == 1)
  {
    // 失能中断
    LL_I2C_DisableIT_EVT(I2C1);
    // 失能 缓冲区中断
    LL_I2C_DisableIT_BUF(I2C1);
    // 失能 错误中断
    LL_I2C_DisableIT_ERR(I2C1);
    // 应答下一个数据（地址或者数据）
    LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);

    // 检测字节传输结束标志位（BTF）
    if ((LL_I2C_IsActiveFlag_BTF(I2C1) == 1))
    {
      *pBuffPtr = LL_I2C_ReceiveData8(I2C1);
      XferCount;
      LL_I2C_ClearFlag_STOP(I2C1);
    }
    // 数据寄存器非空 （RXNE）
    if ((LL_I2C_IsActiveFlag_RXNE(I2C1) == 1))
    {
      counter++;
      *pBuffPtr = LL_I2C_ReceiveData8(I2C1);
      XferCount;
    }
    if (XferCount != 0U)
    {
      /* 数据接收失败处理 */
    }
    LL_I2C_EnableIT_EVT(I2C1);
    LL_I2C_EnableIT_BUF(I2C1);
    LL_I2C_EnableIT_ERR(I2C1);
  }
  /* 从机发送 */

  /* TXE标志位置位，BTF标志位未置位 */
  else if ((LL_I2C_IsActiveFlag_TXE(I2C1) == 1) && (LL_I2C_IsEnabledIT_BUF(I2C1) == 1) && (LL_I2C_IsActiveFlag_BTF(I2C1) == 0))
  {
    if (XferCount != 0)
    {
      // LL_I2C_TransmitData8(I2C1, 0x68);
      LL_I2C_TransmitData8(I2C1, ADC_Result[I2CSendPointer]);
      // SEGGER_RTT_printf(0, "IIC sent 0x%x\r\n", ADC_Result[I2CSendPointer]);
      I2CSendPointer++;
    }
  }
  /* BTF标志位置位 */
  else if ((LL_I2C_IsActiveFlag_BTF(I2C1) == 1) && (LL_I2C_IsEnabledIT_EVT(I2C1) == 1))
  {
    if (XferCount != 0)
    {
      LL_I2C_TransmitData8(I2C1, 0x66);
      XferCount--;
    }
  }
  /* 从机接收 */
  else
  {
    /* RXNE标志位置位，BTF标志位未置位 */
    if ((LL_I2C_IsActiveFlag_RXNE(I2C1) == 1) && (LL_I2C_IsEnabledIT_BUF(I2C1) == 1) && (LL_I2C_IsActiveFlag_BTF(I2C1) == 0))
    {
      if (XferCount != 0U)
      {
        aRxBuffer[0] = LL_I2C_ReceiveData8(I2C1);
        // SEGGER_RTT_printf(0, "IIC recv 0x%x\r\n", aRxBuffer[0]);
        switch (aRxBuffer[0])
        {
          aADCRefresh++;
        case 0: // BAT, CH1
          I2CSendPointer = 0;
          break;
        case 1: // Vin, CH6
          I2CSendPointer = 2; 
          break;
        case 2: // Temp, CH11
          I2CSendPointer = 4;
          // SEGGER_RTT_printf(0, "Gonna refresh adc\r\n", aRxBuffer[0]);
          break;
        case 3: // Vref, CH12
          I2CSendPointer = 6;
          break;
        case 9: // CRC
          I2CSendPointer = 8;
          break;
        default:
          I2CSendPointer = 0xFF;
        }
        //        if (rcv_counter >= 2)
        //        {
        //          rcv_counter = 0;
        //        }
      }
    }
    /* BTF标志位置位 */
    else if ((LL_I2C_IsActiveFlag_BTF(I2C1) == 1) && (LL_I2C_IsEnabledIT_EVT(I2C1) == 1))
    {
      if (XferCount != 0U)
      {
        *pBuffPtr = LL_I2C_ReceiveData8(I2C1);
      }
    }
  }
}
/**
 * @brief  I2C主机接收完最后一字节后，向从机发送NACK，从机NACK中断回调函数
 * @param  无
 * @retval 无
 */
void APP_SlaveIRQCallback_NACK(void)
{
  if ((LL_I2C_IsActiveFlag_AF(I2C1) == 1) && (LL_I2C_IsEnabledIT_ERR(I2C1) == 1))
  {
    if ((State == I2C_STATE_BUSY_TX))
    {
      LL_I2C_DisableIT_EVT(I2C1);
      LL_I2C_DisableIT_BUF(I2C1);
      LL_I2C_DisableIT_ERR(I2C1);

      LL_I2C_ClearFlag_AF(I2C1);

      LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);

      State = I2C_STATE_READY;
    }
  }
}

/**
 * @brief  错误执行函数
 * @param  无
 * @retval 无
 */
void Error_Handler(void)
{
  /* 无限循环 */
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  输出产生断言错误的源文件名及行号
 * @param  file：源文件名指针
 * @param  line：发生断言错误的行号
 * @retval 无
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* 用户可以根据需要添加自己的打印信息,
     例如: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* 无限循环 */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE****/
