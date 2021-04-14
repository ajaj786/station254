/**
  ******************************************************************************
  * @file           : LineScanner.c
  * @author  		: Mohsen Asadipour
  * @version 		: V1.0
  * @date    		: 22-November-2019
  * @brief          : scanning line
  ******************************************************************************
  */

#include "system.h"

/**
 * Read the Lines Data from a CAN Packet. Line Data = found line , lines coordinates up to 3 Lines
 * @param CANData
 * @return
 */
void LineScanner_ReadLines(uint8_t CANData[], boolean MirrorLines , lines *Lines )
{
	Lines->NumberFoundLines = CANData[0];

	for(int i = 0 ; i < 3; i++)
	{
		if(! MirrorLines)
			Lines->Line[i] = CANData[i + 1];
		else
			Lines->Line[i] = LineScanner_MirrorLines(CANData[i + 1]);
	}

	return Lines;
}

/**
 * in order to reflect the line if the sensor was mounted in 180 degree
 * its better to reflect the line instead of changing lots of signs in code
 * a line can have a value 0 to 127 and this function mirror the value from middle of the array also (64)
 * @param Line
 * @return
 */
uint8_t LineScanner_MirrorLines(uint8_t Line)
{
	/*
	 * mirroring values
	 * input -> output
	 * 0		127
	 * 63		64
	 * 64		63
	 * 127		0
	 * */
	return 127 - Line;
}
