/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* application defines */
#define Timer_Max 7   				/*maximum available timers at moment are 4. in order to have more timers, arrays should be adjusted.
									in order to use TON[3], 4 timer should be activated(the index starts from 0)
									the reason of using a define and not a variable is that c compiler does not accept array index as variable for definition
 	 	 	 	 	 	 	 	 	 */
#define Size_DataBuffer   300
#define Size_AICPHeaderLength 10

#define Forward 1
#define Backward 0

#define MotorPosition_Right 0
#define MotorPosition_Left 1


#define KEY_PRESSED							0x00
#define KEY_NOT_PRESSED						0x01

#define CANDeviceID_Main					0x2AA	//Main Module
#define CANDeviceID_BarrierSensor_Head		0x3BB	//RGB & Barrier Sensor Module
#define CANDeviceID_BarrierSensor_Tail		0x3BC	//RGB & Barrier Sensor Module
#define CANDeviceID_MotorDriver_Left		0x111	//Left Motor Driver & Encoder Module
#define CANDeviceID_MotorDriver_Right		0x112	//Right Motor Driver & Encoder Module
#define CANDeviceID_LineScanner				0x4CC	//Line Follower Module
#define CANDeviceID_RFID					0x5DD	//RFID Module
#define CANDeviceID_LightController			0x6EE	//Light Controller Module

#define CANDeviceID_ThisModule 				CANDeviceID_Main

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
//toDo: move this type to PID.h
/*PID Controller*/
/**
 * PIDState:
 * this struct save the current state of PID Controller
 * PrevError_C : last error for Deferential part
 * iState: the current state of Integrator. anti wind-up would be implemented on this parameter
 * IntTerm_C: the resulted Integrator Term after anti wind-up controll and multiplication by Ki
 */
typedef struct
{	double PrevError_C;
	double IntTerm_C ;
	double iState;
}PIDState;

/**
 * PIDParameters:
 * Kp : proportional coefficient
 * Ki : integrator coefficient
 * Kd : differentionator coefficient
 * Imin : integrator minimum wind-up
 * Imax : integrator maximum wind-up
 */
typedef struct
{
	double Kp, Ki, Kd;
	double Imin;
	double Imax;
}PIDParameters;


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_BT_Pin GPIO_PIN_13
#define USER_BT_GPIO_Port GPIOC
#define LED_MCU_Pin GPIO_PIN_0
#define LED_MCU_GPIO_Port GPIOC
#define LED_ERROR_Pin GPIO_PIN_1
#define LED_ERROR_GPIO_Port GPIOC
#define LED_STATUS_Pin GPIO_PIN_2
#define LED_STATUS_GPIO_Port GPIOC
#define LED_AICP_RX_Pin GPIO_PIN_4
#define LED_AICP_RX_GPIO_Port GPIOC
#define LED_AICP_TX_Pin GPIO_PIN_5
#define LED_AICP_TX_GPIO_Port GPIOC
#define R_SW1_Pin GPIO_PIN_12
#define R_SW1_GPIO_Port GPIOB
#define R_SW_2_Pin GPIO_PIN_13
#define R_SW_2_GPIO_Port GPIOB
#define R_SW_3_Pin GPIO_PIN_14
#define R_SW_3_GPIO_Port GPIOB
#define R_SW_4_Pin GPIO_PIN_15
#define R_SW_4_GPIO_Port GPIOB
#define CAN_RS_Pin GPIO_PIN_8
#define CAN_RS_GPIO_Port GPIOC
#define CAN_EN_Pin GPIO_PIN_9
#define CAN_EN_GPIO_Port GPIOC
#define XBEE_RESET_Pin GPIO_PIN_8
#define XBEE_RESET_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define RS485_EN_Pin GPIO_PIN_12
#define RS485_EN_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/**
 * Boolean
 */
typedef enum {
	False = 0,//!< False
	True      //!< True
}boolean;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
