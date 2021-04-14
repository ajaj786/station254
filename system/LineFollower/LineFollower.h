/**
  ******************************************************************************
  * @file           : LineFollower.h
  * @author  		: Mohsen Asadipour
  * @version 		: V1.0
  * @date    		: 23-November-2019
  * @brief          : line following algorithms and functions
  ******************************************************************************
  */



#ifndef LineFollower_H_
#define LineFollower_H_

/*add your code heir*/
#include "main.h"


extern uint8_t temp_1 ;
extern int8_t temp_2 ;
extern 	int8_t derArray_temp [2];
extern uint8_t mLine;
extern int8_t derivative_1_array [20];
extern int8_t ActualDerivative_1[20];
extern float CheckDrvArray[8] ;
extern boolean CurveDetected;
extern boolean OnStraightLine;
extern int RightCurvevaluecheck;
extern int LeftCurvevaluecheck ;
extern float CurveDeviationCoefficient[20];
extern float InnerMotorDeviationCoefficient[35];
extern float OuterMotorDeviationCoefficient[35];
extern float pt_eight_OuterMotorDeviationCoefficient[33];
extern float pt_eight_InnerMotorDeviationCoefficient[35];
extern const uint8_t CurveDeviationCoefficient_MinIndex ;
extern int straightlineCounter;
extern const uint8_t CurveDeviationCoefficient_MaxIndex ;
extern const uint8_t InnerMotorDeviationCoefficient_MinIndex;
extern const uint8_t InnerMotorDeviationCoefficient_MaxIndex  ;
extern const uint8_t OuterMotorDeviationCoefficient_MinIndex ;
extern const uint8_t OuterMotorDeviationCoefficient_MaxIndex ;
const uint8_t pt_eight_InnerMotorDeviationCoefficient_MinIndex ;
const uint8_t pt_eight_InnerMotorDeviationCoefficient_MaxIndex  ;
const uint8_t pt_eight_OuterMotorDeviationCoefficient_MinIndex ;
const uint8_t pt_eight_OuterMotorDeviationCoefficient_MaxIndex ;
extern uint8_t MightBeOscillating;
extern int8_t Oscillator_count;
extern boolean Curve_present;
extern boolean pt_eight_curve;
extern boolean RightCurve;
extern boolean LeftCurve;
extern int mCurve;
extern float InnerCoefficient;
extern float OuterCoefficient;
extern int8_t max;
extern int8_t min;
extern int tempState [2];
extern 	int tempState_1;
extern int tempState_2;
extern int8_t LineCenterDiff  ;
extern int8_t mLineCenterDiff ;
extern int8_t LineLimits ;
extern lines Lines;
extern uint8_t mLine;
extern uint8_t LineCenter;
extern const float cMotorMaxSpeedMPS  ;
extern const float cMotorMinSpeedMPS ;
extern const float cMotorStopSpeed ;
extern double PID_Output ;
extern PIDParameters PIDParams_Dir;
extern PIDState PIDStateData_Dir;
extern PIDParameters PIDParams_Motor;
extern PIDState PIDStateData_Motor;
extern float DesiredSpeed;
extern uint32_t MotorLeft_Diversion ;
extern uint32_t MotorRight_Diversion ;
extern const int8_t cLineScanner_SensorCenter;
extern const int8_t cLineScanner_LowestPixel  ;
extern const int8_t cLineScanner_HighstPixel ;
extern float MotorLeft_Speed ;
extern float MotorRight_Speed  ;
extern int8_t Derivative_Max;
extern const uint8_t DeviationCoefficient_MinIndex;
extern const uint8_t DeviationCoefficient_MaxIndex;
extern float DeviationCoefficient[25] ;
extern boolean InGreenWindow;
extern int8_t LineCenterDiff_Derivative;
extern float MotorLeft_PIDSignal_Scaled;
extern float MotorRight_PIDSignal_Scaled;
extern float MotorLeft_PIDSignal ;
extern float MotorRight_PIDSignal ;
extern int8_t AGVDeviation;
extern uint8_t AGVDeviationDegree;
extern float AGVDeviationCoefficient;
//Move_S6Variables
extern float MappedAGVDeviation[2];
extern float NumberOfRevolution;
extern float TargetEncoderCounter;
extern float DiffInLastMotorReading;
extern float RightMotorArray[4][3];
extern float LeftMotorArray [4][3];
extern const uint8_t TICK_Arr ;
extern const uint8_t Dist_Arr ;
extern const uint8_t Vel_Arr ;
extern const uint8_t Time_Arr;
extern boolean ResetEncoderRight;
extern boolean ResetEncoderLeft;
extern float VectorAngle;
extern boolean PackageDelay;
//extern boolean UseDist;
extern float NumberOfRevolution;
extern boolean Move_S6Called;
extern uint32_t Move_S6CalledTime;
extern float TempSpeed[2];
extern float TempSpeed_1;

/***Motor***/
float Calc_SpeedCorrection(float Speed_maxDesired , int8_t LineCenterDiff_Derivative);

void CurveRecognition (uint8_t CurrentPosition  , uint8_t LinePosition );

float Calculate_InnerCoefficient (uint8_t AGVDeviationDegree );
float Calculate_OuterCoefficient (uint8_t AGVDeviationDegree );

void GetMaxDerivative (int8_t Derivative);
uint8_t Calculate_DiversionFromCenter(uint8_t SensorCenter , uint8_t LinePosition , boolean MotorPosition);
int8_t Calculate_DiversionFromCenter_S4(uint8_t SensorCenter , uint8_t LinePosition , boolean Reverse );
uint8_t Calculate_DeviationDegree(int8_t AGVDeviation , uint8_t Subdivisions , uint8_t SensorHighstPixel);
float Calculate_DeviationCoefficient(uint8_t AGVDeviationDegree);
float Calculate_MotorSpeed(int8_t AGVDeviation , boolean MotorPosition , boolean RightCurve, boolean LeftCurve, float AGVDeviationCoefficient,float DesiredSpeed );
float map(float val, float I_Min, float I_Max, float O_Min, float O_Max);

void Move_S1(boolean MoveStart);
void Move_S2(boolean MoveStart,uint8_t mLine);
void Move_S3(boolean MoveStart);
void Move_S4(boolean MoveStart, uint8_t LinePosition);
void Move_S5(boolean MoveStart,uint8_t mLine);
void Move_S6(boolean MoveStart, uint8_t mLine, boolean UseDist);

#endif /* LineFollower_H_ */



