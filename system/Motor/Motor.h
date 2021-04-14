/**
  ******************************************************************************
  * @file           : Motor.c
  * @author  		: Mohsen Asadipour
  * @version 		: V1.0
  * @date    		: 23-November-2019
  * @brief          : controlling the motor
  ******************************************************************************
  */



#ifndef Motor_H_
#define Motor_H_

/*add your code heir*/
#include "main.h"

extern CAN_TxHeaderTypeDef TxHeader;
extern uint8_t  MotorMovementData[7];
extern float RightMotorDistance;
extern float LeftMotorDistance;
extern uint32_t RightEncoderCounts;
extern uint32_t LeftEncoderCounts;
extern uint32_t RightEncoderReset;
extern uint32_t LeftEncoderReset;
extern uint32_t RightTimeNow;
extern uint32_t RightTimeReset;
extern uint32_t LeftTimeReset ;
extern uint32_t LeftTimeNow;
extern float RightEncoderResolution;
extern float LeftEncoderResolution;
extern float DegreePerPulse;
extern float DistanceMovedPerPulse;
extern const float WheelRadius; //radius in meters
//extern float RightTempDistance;
//extern float LeftTempDistance ;
//extern float DistanceDiff;
extern const uint16_t DistanceBetweenMotors;
//extern float VectorAngle;
extern boolean RightAtRest;
extern boolean LeftAtRest;
extern float RightMotorArray[4][3];
extern float LeftMotorArray [4][3];
extern const uint8_t TICK_Arr ;
extern const uint8_t Dist_Arr ;
extern const uint8_t SegmentProgessData ;
extern const uint8_t Time_Arr;
extern boolean ResetEncoderRight;
extern boolean ResetEncoderLeft;
extern boolean ResetLeftSegmentProgessData;
extern boolean ResetRightSegmentProgessData;
extern uint32_t LeftSegmentProgessCount;
extern uint32_t RightSegmentProgessCount;
extern uint16_t EncoderDataRecieveFrequency;
//extern boolean DriveDistanceCalled;
//extern float TargetEncoderCounter;
//extern float NumRev;
//extern boolean PackageDelay;
//extern float VectorAngle;
//extern uint8_t mLine;
//extern float LineScanDeviationAngle;
//extern 	float LSAngle;
//extern uint16_t DiffInLastMotorReading;
//extern boolean UseDist;

void Motor_SpeedCommand( int16_t CANDeviceID , boolean  direction , float speedMPS);
void writeOnCAN_Speed(float speed);
void writeOnCAN_Direction(boolean direction);
float GetDistanceFromCANData( uint8_t Data[]);
uint32_t (uint8_To_uint32)(uint8_t Data[]);
void Motor_ReadRightMotorDistance(uint8_t CANData[]);
void Motor_ReadLeftMotorDistance(uint8_t CANData[]);
void Motor_ReadRightEncoderCounts(uint8_t CANData[]);
void Motor_ReadLeftEncoderCounts(uint8_t CANData[]);
float DistanceMoved(uint32_t Ticks , uint16_t Resolution);
//void GetMotorDistanceDiff(uint32_t RightDistance, uint32_t LeftDistance);
//float MotorVectorAngle(float DistanceDiff);
float VelocityPerCANMessageRecieved (uint32_t Distance);
//void DriveDistance (float distance,boolean UseDist);
//float map(float val, float I_Min, float I_Max, float O_Min, float O_Max);
//float TotalDistanceMoved();
#endif /* Motor_H_ */



