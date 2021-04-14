/**
  ******************************************************************************
  * @file           : LineScanner.h
  * @author  		: Mohsen Asadipour
  * @version 		: V1.0
  * @date    		: 22-November-2019
  * @brief          : scanning line
  ******************************************************************************
  */

#include "main.h"


/*Lines*/
typedef struct
{
	uint8_t NumberFoundLines;
	uint8_t Line[3];
}lines;

/***Line Scanner***/
void LineScanner_ReadLines(uint8_t CANData[], boolean MirrorLines, lines *Lines);
uint8_t LineScanner_MirrorLines(uint8_t Line);




