/**
  ******************************************************************************
  * @file    py32f0xx_it.c
  * @author  MCU Application Team
  * @brief   Interrupt Service Routines.
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
#include "py32f0xx_it.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers         */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
	SEGGER_RTT_printf(0, "HardFault_Handler.\r\n");
  while (1)
  {
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/* PY32F0xx Peripheral Interrupt Handlers                                     */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_py32f003xx.s).                   */
/******************************************************************************/
void I2C1_IRQHandler(void)
{
  APP_SlaveIRQCallback();
  //APP_SlaveIRQCallback_NACK();
}

void ADC_COMP_IRQHandler(void)
{
  /* 检测是不是转换结束触发的中断 */
  if(LL_ADC_IsActiveFlag_EOC(ADC1) != 0)
  {
    /* 清空ADC EOC 中断 */
    LL_ADC_ClearFlag_EOC(ADC1);

    /* 调用中断处理函数 */
    APP_AdcGrpRegularUnitaryConvCompleteCallback();
  }
}


/************************ (C) COPYRIGHT Puya *****END OF FILE****/
