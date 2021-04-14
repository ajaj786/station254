/**
  ******************************************************************************
  * @file           : LineScanner.c
  * @author  		: Mohsen Asadipour
  * @version 		: V1.0
  * @date    		: 23-November-2019
  * @brief          : detecting obstacles in from of robot
  ******************************************************************************
  */

#include <system.h>

/**
 * extract the obstacle detection sensor states form can data
 * @param CANData
 */
void ObstacleDetection_GetSensorsState(uint8_t CANData[])
{
	if (CANData[0] & 0x01)
		ObstacleDetection_SensorState[0] = True;
	else
		ObstacleDetection_SensorState[0] = False ;


	if (CANData[0] & 0x02)
		ObstacleDetection_SensorState[1] = True;
	else
		ObstacleDetection_SensorState[1] = False ;


	if (CANData[0] & 0x04)
		ObstacleDetection_SensorState[2] = True;
	else
		ObstacleDetection_SensorState[2] = False ;


	if (CANData[0] & 0x08)
		ObstacleDetection_SensorState[3] = True;
	else
		ObstacleDetection_SensorState[3] = False ;


	if (CANData[0] & 0x10)
		ObstacleDetection_SensorState[4] = True;
	else
		ObstacleDetection_SensorState[4] = False ;
}

/**
 * Process the sensor signals and set the output if obstacle was detected
 *
 * @return
 */
boolean ObstacleDetection_ProcessSensorSignalsSignals()
{
	boolean obstacleDetected = False;

		obstacleDetected = 	 	( ObstacleDetection_SensorState[0] & ObstacleDetection_SensorActive[0] ) |
								( ObstacleDetection_SensorState[1] & ObstacleDetection_SensorActive[1] ) |
								( ObstacleDetection_SensorState[2] & ObstacleDetection_SensorActive[2] ) |
								( ObstacleDetection_SensorState[3] & ObstacleDetection_SensorActive[3] ) |
								( ObstacleDetection_SensorState[4] & ObstacleDetection_SensorActive[4] );

	return obstacleDetected;
}

/**
 * stop the AGV for a defined time after detecting obstacle
 * this function act on falling edge of Obstacle detected signal and stretch the time by Timer 4 elapse time
 * @param ObstacleDetected
 * @return
 */
boolean ObstacleDetection_ResumeDelay(boolean ObstacleDetected )
{
	//getting the rising edge: by detecting rising edge, set the output
	if (!ObstacleDetected & mObstacleDetected) /*falling edge*/
	{
		ObstacleDetection_ResumeDelay_Output = True;
		TON_Start[4] = True;
	}


	mObstacleDetected = ObstacleDetected;	/*saving the edge state*/

	//reset the output after timer elapsed
	if(TON_Out[4]  )
	{
		ObstacleDetection_ResumeDelay_Output = False ;
		TON_Start[4] = False ;
	}

	return ObstacleDetection_ResumeDelay_Output | ObstacleDetected;	//set the output with delay or when the input signal is set
}

