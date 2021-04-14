/**
  ******************************************************************************
  * @file           : Timer.c
  * @author  		: Mohsen Asadipour
  * @version 		: V1.0
  * @date    		: 23-November-2019
  * @brief          : Timer controlling functions
  ******************************************************************************
  */

#include <system.h>

/*add your code heir*/

/**
 * run timer related to this index
 * @param Index
 */
void Timer(uint16_t Index)
{

	//capturing the edge of input signal
	if (TON_Start[Index]  && ! mTON_Start[Index] )	//by rising edge start counting
	{
		TON_StartCounter[Index] = True;
		TON_Counter[Index] = 0 ;	//reset the counter of the timer
	}

	if(!TON_Start[Index] &&  mTON_Start[Index])	//by falling edge stop counting
	{
		TON_StartCounter[Index] = False;
		TON_Counter[Index] = 0 ;
		TON_Out[Index] = False ;
	}

	mTON_Start[Index] = TON_Start[Index];



	//count til PT
	if(TON_StartCounter[Index] & TON_Start[Index] & !TON_Out[Index])
	{
		TON_Counter[Index] ++;
	}

	//set the output
	if (TON_Counter[Index] >= TON_PT_ms[Index])
	{
		TON_Out[Index] = True;
	}
}

/**
 * calculate in the first cycle the needed ticks for each ms
 */
void Timer_CalculatePTs()
{
	for(int i = 0 ; i < Timer_Max ; i++)
	{
		TON_PT[i] = TON_PT_ms[i] * Timer_Coefficient;
	}
}

