/**
  ******************************************************************************
  * @file           : Motor.c
  * @author  		: Mohsen Asadipour
  * @version 		: V1.0
  * @date    		: 23-November-2019
  * @brief          : controlling the motor
  ******************************************************************************
  */

#include <system.h>

/*add your code heir*/
extern uint8_t motorConsistencyCounter;
/**************Motor speed Control**************************/
/**
 * control the direction and power of motor
 * @param CANDeviceID: the CAN ID of Motor driver => CANDeviceID_MotorDriver_Left , CANDeviceID_MotorDriver_Right
 * @param Direction: Forward or Backward
 * @param speedMPS: 0 - 2.3 m/s
 */
void Motor_SpeedCommand( int16_t CANDeviceID , boolean  direction , float speedMPS)
{
	/*control the limits of speedMPS*/
	speedMPS = Check_Boundaries(speedMPS, cMotorMinSpeedMPS, cMotorMaxSpeedMPS);

	TxHeader.DLC = 7;		//setting data length
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = CANDeviceID;	//setting the device can id

	MotorMovementData[0] = 0xF1;

	writeOnCAN_Direction(direction); 	//setting direction
	writeOnCAN_Speed(speedMPS);	//setting speed

	MotorMovementData[6] = motorConsistencyCounter;

	CAN_Transmit(&TxHeader, MotorMovementData);	//sending the can packet
}

/**
 * convert and write the float speed parameter into CAN Packet
 * @param speed : 0 - 1
 */
void writeOnCAN_Speed(float speed)
{
	uint8_t  bufferBytes[4];

	//initialising the array
	for(int i = 0 ; i<4 ; i++)
		bufferBytes[i] = 0 ;


	*(float*)(bufferBytes) = speed ;// convert float to bytes

	MotorMovementData[1] = bufferBytes[0];
	MotorMovementData[2] = bufferBytes[1];
	MotorMovementData[3] = bufferBytes[2];
	MotorMovementData[4] = bufferBytes[3];
}

/**
 * wirte direction parameter into CAN Packet
 * @param direction: Forward , Backward
 */
void writeOnCAN_Direction(boolean direction)
{
	MotorMovementData[5] = direction ;
}

void Motor_ReadRightMotorDistance(uint8_t CANData[])
{

	RightMotorDistance = GetDistanceFromCANData(CANData);

	 Motor_ReadRightEncoderCounts( CANData);
}

void Motor_ReadLeftMotorDistance(uint8_t CANData[])
{

	LeftMotorDistance = GetDistanceFromCANData(CANData);
	Motor_ReadLeftEncoderCounts(CANData);

}

void Motor_ReadRightEncoderCounts(uint8_t CANData[])
{

	RightEncoderCounts= uint8_To_uint32(CANData);
	RightTimeNow = HAL_GetTick() - Move_S6CalledTime;

	if (ResetRightSegmentProgessData== True){
		ResetRightSegmentProgessData = False;
		RightSegmentProgessCount = RightEncoderCounts;
		RightMotorArray[SegmentProgessData][1] = 0;
	}

	if (ResetEncoderRight == True ) {
		ResetEncoderRight = False;
		RightEncoderReset = RightEncoderCounts;
		RightTimeReset = RightTimeNow;
		RightMotorArray[TICK_Arr][1] = 0;
	}
	RightMotorArray[TICK_Arr][0] = RightEncoderCounts - RightEncoderReset;  // stores the value in (1,1) Encoder count data
	RightMotorArray[Time_Arr][0] =RightTimeNow - RightTimeReset;
	RightMotorArray[SegmentProgessData][0]= RightEncoderCounts - RightSegmentProgessCount;


	if (RightMotorArray[TICK_Arr][0] != RightMotorArray[TICK_Arr][1]){    //Checks the EncoderCount Data(1,1) against the second element of the array(1,2)
		RightAtRest = False;
	}
	else {
		//RightMotorArray[0][1] = RightMotorArray[0][2];
		RightAtRest = True;

		RightMotorArray[TICK_Arr][2] =RightMotorArray[TICK_Arr][0] - RightMotorArray[TICK_Arr][1];  //store the encoder data vlaue in (1,3) always 0 as this condition is true only when motor is at rest.
		RightMotorArray[Time_Arr][2] = RightMotorArray[Time_Arr][0] - RightMotorArray[Time_Arr][1];
		RightMotorArray[SegmentProgessData][2] =RightMotorArray[SegmentProgessData][0] -RightMotorArray[SegmentProgessData][1];


		//RightMotorArray [Dist_Arr][0] =  DistanceMoved(RightMotorArray [TICK_Arr][2] , RightEncoderResolution); // Stores the distance Value in (2,1) and always is 0
		RightMotorArray [Dist_Arr][2] = RightMotorArray[Dist_Arr][1]; //Store the Previous distance Covered in (2,3);
	}

	if (RightAtRest == False){
		RightMotorArray [TICK_Arr][2] = RightMotorArray[TICK_Arr][0] - RightMotorArray[TICK_Arr][1]; //store Difference of the encoder data in (1,3)
		RightMotorArray[SegmentProgessData][2] =RightMotorArray[SegmentProgessData][0] -RightMotorArray[SegmentProgessData][1];
		RightMotorArray [Time_Arr][2] = RightMotorArray[Time_Arr][0] - RightMotorArray[Time_Arr][1];

		RightMotorArray [Dist_Arr][0] =  DistanceMoved(RightMotorArray [SegmentProgessData][0] , RightEncoderResolution); // calculate the total distance moved by the Right motor
		RightMotorArray [Dist_Arr][1] = DistanceMoved(RightMotorArray [SegmentProgessData][1] , RightEncoderResolution); // calculate the distance moved betweeen the last transmit
		RightMotorArray[Dist_Arr][2] = RightMotorArray [Dist_Arr][0] - RightMotorArray [Dist_Arr][1];


		RightMotorArray[TICK_Arr][1] = RightMotorArray[TICK_Arr][0]; // update the data in (1,2) to check the if loop above
		RightMotorArray[Time_Arr][1] = RightMotorArray[Time_Arr][0];
		RightMotorArray[SegmentProgessData][1] =RightMotorArray[SegmentProgessData][0];

		if (RightMotorArray [Time_Arr][2] > EncoderDataRecieveFrequency){
			int Corrector = 0;
			Corrector = RightMotorArray [Time_Arr][2]/EncoderDataRecieveFrequency;
			RightMotorArray[TICK_Arr][2] = RightMotorArray[TICK_Arr][2]/Corrector;
		}

	}

}

void Motor_ReadLeftEncoderCounts(uint8_t CANData[]){

	LeftEncoderCounts = (uint8_To_uint32(CANData))/2; //adjusting the encoder pulse difference.
	LeftTimeNow = HAL_GetTick() - Move_S6CalledTime;

	if (ResetLeftSegmentProgessData == True) {
		ResetLeftSegmentProgessData = False;
		LeftSegmentProgessCount = LeftEncoderCounts;
		LeftMotorArray[SegmentProgessData][1] = 0;
	}

	if (ResetEncoderLeft == True) {
		ResetEncoderLeft = False;
		LeftEncoderReset = LeftEncoderCounts;
		LeftTimeReset = LeftTimeNow;
		LeftMotorArray[TICK_Arr][1] = 0;
	}
	LeftMotorArray[TICK_Arr][0] = LeftEncoderCounts - LeftEncoderReset; // stores the value in (1,1) Encoder count data
	LeftMotorArray[Time_Arr][0] = LeftTimeNow - LeftTimeReset;
	LeftMotorArray[SegmentProgessData][0] = LeftEncoderCounts - LeftSegmentProgessCount;

	if (LeftMotorArray[TICK_Arr][0] != LeftMotorArray[TICK_Arr][1]) { //Checks the EncoderCount Data(1,1) against the second element of the array(1,2)
		LeftAtRest = False;

	}
	else {
		LeftAtRest = True;

		LeftMotorArray[TICK_Arr][2] =LeftMotorArray[TICK_Arr][0] - LeftMotorArray[TICK_Arr][1];  //store the encoder data vlaue in (1,3) always 0 as this condition is true only when motor is at rest.
		LeftMotorArray[SegmentProgessData][2] =LeftMotorArray[SegmentProgessData][0] -LeftMotorArray[SegmentProgessData][1];
		LeftMotorArray[Time_Arr][2] = LeftMotorArray[Time_Arr][0] - LeftMotorArray[Time_Arr][1];
		//LeftMotorArray [Dist_Arr][0] =  DistanceMoved(LeftMotorArray [TICK_Arr][2] , LeftEncoderResolution); // Stores the distance Value in (2,1) and always is 0

		LeftMotorArray [Dist_Arr][2] = LeftMotorArray[Dist_Arr][1]; //Store the Previous distance Covered in (2,3);
		//LeftMotorArray [Vel_Arr][3] = VelocityPerCANMessageRecieved(LeftMotorArray [Dist_Arr][0]);
	}

	if (LeftAtRest == False){
		LeftMotorArray [TICK_Arr][2] = LeftMotorArray[TICK_Arr][0] - LeftMotorArray[TICK_Arr][1]; //store Difference of the encoder data in (1,3)
		LeftMotorArray[SegmentProgessData][2] =LeftMotorArray[SegmentProgessData][0] -LeftMotorArray[SegmentProgessData][1];
		LeftMotorArray [Time_Arr][2] = LeftMotorArray[Time_Arr][0] - LeftMotorArray[Time_Arr][1];

		LeftMotorArray [Dist_Arr][0] =  DistanceMoved(LeftMotorArray [SegmentProgessData][0] , LeftEncoderResolution); // calculate the total distance moved by left motor
		LeftMotorArray [Dist_Arr][1] =DistanceMoved(LeftMotorArray [SegmentProgessData][1] , LeftEncoderResolution); // calculate the distance moved in the last transmit
		LeftMotorArray[Dist_Arr][2] = LeftMotorArray [Dist_Arr][0] - LeftMotorArray [Dist_Arr][1];


		LeftMotorArray[TICK_Arr][1] = LeftMotorArray[TICK_Arr][0]; // update the data in (1,2) to check the if loop above

		LeftMotorArray[SegmentProgessData][1] =LeftMotorArray[SegmentProgessData][0];
		LeftMotorArray[Time_Arr][1] = LeftMotorArray[Time_Arr][0];

		if (LeftMotorArray [Time_Arr][2] > (EncoderDataRecieveFrequency-1)){
			int Corrector = 0;
			Corrector = LeftMotorArray [Time_Arr][2]/(EncoderDataRecieveFrequency-1);
			LeftMotorArray[TICK_Arr][2] = LeftMotorArray[TICK_Arr][2]/Corrector;
		}
		//LeftMotorArray[Dist_Arr][0];  // Moving the distance to (2,2)

//		LeftMotorArray [Vel_Arr][0] = VelocityPerCANMessageRecieved(LeftMotorArray [Dist_Arr][1]);
//		LeftMotorArray [Vel_Arr][1] = LeftMotorArray [Vel_Arr][2];

	}

	//GetMotorDistanceDiff (RightTempDistance, LeftTempDistance);
}

float GetDistanceFromCANData( uint8_t Data[]){
	float Output = 0;
		uint8_t bytes[4] ;

		bytes[0] = Data[0];
		bytes[1] = Data[1];
		bytes[2] = Data[2];
		bytes[3] = Data[3];

		Output = *(float*)(bytes); 	// convert bytes back to float

		return Output;
}

uint32_t (uint8_To_uint32)(uint8_t Data[]){
	uint32_t Output = 0;
	uint8_t bytes[4];
	bytes[0] = Data[0];
	bytes[1] = Data[1];
	bytes[2] = Data[2];
	bytes[3] = Data[3];
//*(uint32_t*)(bytes); 	//  (bytes[0] <<24) + (bytes[1] <<16) + (bytes[2] <<8) + bytes[3];//*// convert bytes back to float
	  for (int i=0; i<4; i++)
	  {
		  Output+=bytes[i]<<(i*8);
	  }
//	Output = ((uint8_t)bytes[3] << 24) | ((uint8_t)bytes[2] << 16) | ((uint8_t)bytes[1] << 8) | (bytes[0] << 0);
//	Output = (bytes[3] <<24) + (bytes[2] <<16) + (bytes[1] <<8) + bytes[0];
	return Output;


}

float DistanceMoved(uint32_t Ticks, uint16_t Resolution){
	float DistanceMoved_1 = 0;
	DegreePerPulse = 360.0/Resolution;
	DistanceMovedPerPulse = (DegreePerPulse/360) * 2 * 3.14 * (WheelRadius*1000); //in mm
	DistanceMoved_1 = DistanceMovedPerPulse * Ticks;
	return DistanceMoved_1;
}

/*float TotalDistanceMoved(){
	float Output = 0;
	//distance moved is the avg of distance moved by the individual motors
	Output = (LeftMotorArray [Dist_Arr][0] + RightMotorArray [Dist_Arr][0] )/ 2;
	return Output;
}*/
float VelocityPerCANMessageRecieved (uint32_t Distance){
	float VelocityBetweenLastMessage = 0;
	VelocityBetweenLastMessage = Distance/0.1; //message recieved every 100ms ==0.1sec

	return VelocityBetweenLastMessage;
}

/*
void GetMotorDistanceDiff(uint32_t RightDistance, uint32_t LeftDistance){

	DistanceDiff = RightDistance - LeftDistance;
	VectorAngle = MotorVectorAngle(DistanceDiff);
}

float MotorVectorAngle(float DistanceDiff){
	float OutputAngle = 0.0;

	OutputAngle = atan2(DistanceDiff, 370.0);
	return OutputAngle;
}
*/

