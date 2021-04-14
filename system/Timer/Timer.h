/**
  ******************************************************************************
  * @file           : Timer.h
  * @author  		: Mohsen Asadipour
  * @version 		: V1.0
  * @date    		: 23-November-2019
  * @brief          : Timer controlling functions
  ******************************************************************************
  */



#ifndef Timer_H_
#define Timer_H_

/*add your code heir*/
#include "main.h"

extern boolean TON_Start[Timer_Max];
extern boolean TON_Out[Timer_Max];
extern boolean mTON_Start[Timer_Max];
extern boolean TON_StartCounter[Timer_Max];
extern uint32_t TON_Counter[Timer_Max];
extern uint32_t TON_PT[Timer_Max];
extern const uint32_t TON_PT_ms[Timer_Max];
extern const uint32_t Timer_Coefficient;

void Timer(uint16_t Index);
void Timer_CalculatePTs();

#endif /* Timer_H_ */



