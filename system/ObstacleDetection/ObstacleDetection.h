/**
  ******************************************************************************
  * @file           : LineScanner.h
  * @author  		: Mohsen Asadipour
  * @version 		: V1.0
  * @date    		: 23-November-2019
  * @brief          : detecting obstacles in from of robot
  ******************************************************************************
  */

#ifndef ObstacleDetection_H_
#define ObstacleDetection_H_

/*add your code heir*/
#include "main.h"

extern boolean ObstacleDetection_SensorState[5] ;
extern boolean mObstacleDetected ;
extern boolean ObstacleDetection_ResumeDelay_Output;
extern boolean TON_Start[Timer_Max];
extern boolean TON_Out[Timer_Max];
extern boolean ObstacleDetection_SensorActive[5];

void ObstacleDetection_GetSensorsState(uint8_t CANData[]);
boolean ObstacleDetection_ProcessSensorSignalsSignals();
boolean ObstacleDetection_ResumeDelay(boolean ObstacleDetected);

#endif /* ObstacleDetection_H_ */




