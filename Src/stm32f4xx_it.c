/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
char rxByte[1];
char rxByte_buffer;
char rxComparer[50];
uint8_t rxResponsePosition = 0;
uint8_t rxSequencePosition = 0;
uint8_t rxPosition;
uint8_t checkCounter = 0;
uint8_t checkCounts = 3;
boolean isInSequence = False;
boolean isSignalWithoutError = True;
boolean isOkaySignal = False;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim2;

/* USER CODE BEGIN EV */
extern char response[10];
extern char rxBuffer[300];
extern char FC_Order[50];
extern UART_HandleTypeDef huart1;
extern uint8_t UART_ReceicedByte[1];
extern boolean UART_RecFlag;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */

	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
		/* USER CODE END W1_MemoryManagement_IRQn 0 */
	}
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void) {
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_BusFault_IRQn 0 */
		/* USER CODE END W1_BusFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_UsageFault_IRQn 0 */
		/* USER CODE END W1_UsageFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles CAN1 RX0 interrupt.
 */
void CAN1_RX0_IRQHandler(void) {
	/* USER CODE BEGIN CAN1_RX0_IRQn 0 */

	/* USER CODE END CAN1_RX0_IRQn 0 */
	HAL_CAN_IRQHandler(&hcan1);
	/* USER CODE BEGIN CAN1_RX0_IRQn 1 */

	/* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
 * @brief This function handles TIM2 global interrupt.
 */
void TIM2_IRQHandler(void) {
	/* USER CODE BEGIN TIM2_IRQn 0 */

	/* USER CODE END TIM2_IRQn 0 */
	HAL_TIM_IRQHandler(&htim2);
	/* USER CODE BEGIN TIM2_IRQn 1 */

	/* USER CODE END TIM2_IRQn 1 */
}

/**
 * @brief This function handles USART1 global interrupt.
 */
void USART1_IRQHandler(void) {
	/* USER CODE BEGIN USART1_IRQn 0 */

	/* USER CODE END USART1_IRQn 0 */
	HAL_UART_IRQHandler(&huart1);
	/* USER CODE BEGIN USART1_IRQn 1 */
	if (HAL_UART_GetState(&huart1) != RESET) {
		/* Read one byte from the receive data register */
		HAL_UART_Receive_IT(&huart1, rxByte, sizeof(rxByte));
		rxByte_buffer = rxByte[0];

		response[rxResponsePosition] = rxByte_buffer;//response ist important for XBee Init.

		if (rxByte_buffer != 0) {
			rxResponsePosition++;
		}

		if (rxByte_buffer == '\r') {
			rxResponsePosition = 0;
		}

		if (rxByte_buffer == '#')// anchor char  for the begin of the sequence
		{
			isInSequence = True;
			rxSequencePosition = 0;
		}

		if (isInSequence)
		{
			rxBuffer[rxSequencePosition] = rxByte_buffer;//putting incoming data in array

			if (rxByte_buffer == '!')// anchor char for the end of the sequence
			{
				rxResponsePosition = 0;
				rxSequencePosition = 0;
				uint8_t crcCounter = 0;
				isInSequence = False;

				for (int i = 0; i < sizeof(FC_Order); i++)
				{
					FC_Order[i] = rxBuffer[i];
				}
				FC_Controller();
			}
		}

		if (rxSequencePosition >= 300) {
			rxSequencePosition = 0;
		} else {
			rxSequencePosition++;
		}

	} else
		HAL_UART_Receive_IT(&huart1, rxByte, sizeof(rxByte));

	/* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
