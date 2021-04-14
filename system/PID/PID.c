/**
  ******************************************************************************
  * @file           : PID.h
  * @author  		: Mohsen Asadipour
  * @version 		: V1.0
  * @date    		: 23-November-2019
  * @brief          : PID Controller functions
  ******************************************************************************
  */

#include <system.h>

/*add your code heir*/

/**************PID Controller**************************/

/**
 * PID Controller: this function implement a full PID Controller with double precision calculation and anti wind-up feature
 * @param In: the captured inputs
 * @param Ref: the desired position of output
 * @param PIDParams: contains all Parameters of PID-Controller. "struct PIDParameters"
 * @param PIDState: save the state of PID Controller. "struct PIDState"
 * @return: the  correction that should be implemented to actuators
 * toDo: the D-Part of controller should be tuned and tested on a real system. At moment D-Part is deactive
 */
double PID_Controller(double In, double Ref, PIDParameters *PIDParams , PIDState *PIDState)
{
	double Error;
	double OutputP = 0 , OutputI = 0, OutputD = 0;

	  //calculating Error
	  Error = Ref - In;

	  //calculating P Part
	  OutputP = PIDParams->Kp * Error;

	  //calculating I Part
	  PIDState->iState += Error;
	  if (PIDState->iState > PIDParams->Imax){PIDState->iState =  PIDParams->Imax;} //applying anti-windup
	  if (PIDState->iState < PIDParams->Imin){PIDState->iState =  PIDParams->Imin; } //applying anti-windup
	  PIDState->IntTerm_C = PIDParams->Ki * PIDState->iState;
	  OutputI = PIDState->IntTerm_C;

	  //calculating D Part
	  OutputD += PIDParams->Kd * (Error - PIDState->PrevError_C);
	  PIDState->PrevError_C = Error;	//saving current d term for the next iteration

	  return OutputP + OutputI + OutputD;
}

/**
 * convert the output of PIC controller into valuable signal for motors
 * @param DesiredSpeed
 * @param PIDOutput
 */
void PIDMotorControl(boolean MoveStart , float DesiredSpeed , double PIDOutput)
{
	uint8_t Limit_Level [20] = {5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100};
//	uint8_t Limit_Level1 = 10 ;
//	uint8_t Limit_Level2 = 15 ;
//	uint8_t Limit_Level3 = 20 ;
//	uint8_t Limit_Level4 = 25 ;
//	uint8_t Limit_Level5 = 30 ;
//	uint8_t Limit_Level6 = 35 ;
//	uint8_t Limit_Level7 = 40 ;
//	uint8_t Limit_Level8 = 45 ;
//	uint8_t Limit_Level9 = 50 ;
//	uint8_t Limit_Level10 = 55 ;
//	uint8_t Limit_Level11 = 60 ;
//	uint8_t Limit_Level12 = 64 ;

	uint8_t weight_Level[20] = {2,6,10,15,20,25,30,35,40,45,55,60,65,70,75,80,85,90,95,100};
//	uint8_t weight_Level1 = 2;
//	uint8_t weight_Level2 = 3;
//	uint8_t weight_Level3 = 5;
//	uint8_t weight_Level4 = 7;
//	uint8_t weight_Level5 = 9;
//	uint8_t weight_Level6 = 11;
//	uint8_t weight_Level7 = 13;
//	uint8_t weight_Level8 = 15;
//	uint8_t weight_Level9 = 20;
//	uint8_t weight_Level10 = 25;
//	uint8_t weight_Level11 = 30;
//	uint8_t weight_Level12 = 35;

  int length = sizeof(Limit_Level)/sizeof(Limit_Level[0]);
	MotorLeft_Speed = 0.15;
	MotorRight_Speed = 0.15;

	/*PID output weight correction*/

	for (int i =0; i < length; i++){
		if(fabs(PIDOutput)> -Limit_Level[i] || fabs(PIDOutput) < Limit_Level[i]){
			PIDOutput = PIDOutput * weight_Level[i] ;
			break;
		}

	}

//	if(fabs(PIDOutput)> -Limit_Level1 || fabs(PIDOutput) < Limit_Level1)
//		PIDOutput = PIDOutput * weight_Level1 ;
//	else if(fabs(PIDOutput)> -Limit_Level2 || fabs(PIDOutput) < Limit_Level2)
//		PIDOutput = PIDOutput * weight_Level2 ;
//	else if(fabs(PIDOutput)> -Limit_Level3 || fabs(PIDOutput) < Limit_Level3)
//		PIDOutput = PIDOutput * weight_Level3 ;
//	else if(fabs(PIDOutput)> -Limit_Level4 || fabs(PIDOutput) < Limit_Level4)
//			PIDOutput = PIDOutput * weight_Level4 ;
//	else if(fabs(PIDOutput)> -Limit_Level5 || fabs(PIDOutput) < Limit_Level5)
//			PIDOutput = PIDOutput * weight_Level5 ;
//	else if(fabs(PIDOutput)> -Limit_Level6 || fabs(PIDOutput) < Limit_Level6)
//			PIDOutput = PIDOutput * weight_Level6 ;
//	else if(fabs(PIDOutput)> -Limit_Level7 || fabs(PIDOutput) < Limit_Level7)
//			PIDOutput = PIDOutput * weight_Level7 ;
//	else if(fabs(PIDOutput)> -Limit_Level8 || fabs(PIDOutput) < Limit_Level8)
//			PIDOutput = PIDOutput * weight_Level8 ;
//	else if(fabs(PIDOutput)> -Limit_Level9 || fabs(PIDOutput) < Limit_Level9)
//			PIDOutput = PIDOutput * weight_Level9 ;
//	else if(fabs(PIDOutput)> -Limit_Level10 || fabs(PIDOutput) < Limit_Level10)
//			PIDOutput = PIDOutput * weight_Level10 ;
//	else if(fabs(PIDOutput)> -Limit_Level11 || fabs(PIDOutput) < Limit_Level11)
//			PIDOutput = PIDOutput * weight_Level11 ;
//	else if(fabs(PIDOutput)> -Limit_Level12 || fabs(PIDOutput) < Limit_Level12)
//			PIDOutput = PIDOutput * weight_Level12 ;


	if(MoveStart /*&& Lines.NumberFoundLines > 0*/)			/*speed correction*/
	{
		if(PIDOutput == 0 ) /*pid output = 0 => robot 100 % on the line*/
		{
			/*do nothing*/
			MotorLeft_Speed = 0.1;
			MotorRight_Speed = 0.1;
		}
		else if(PIDOutput < 0) 	/*pid output > 0 => robot is too right*/
		{
			MotorLeft_Speed = DesiredSpeed - (fabs(PIDOutput)/200) * DesiredSpeed;
			//MotorRight_Speed = DesiredSpeed + (fabs(PIDOutput)/2000) * DesiredSpeed  ;
		}
		else if(PIDOutput > 0) 	/*pid output < 0 => robot is too left*/
		{
			MotorRight_Speed = DesiredSpeed - (fabs(PIDOutput)/200) * DesiredSpeed;
			//MotorLeft_Speed = DesiredSpeed + (fabs(PIDOutput)/2000) * DesiredSpeed ;
		}
	}
	else
	{
		MotorLeft_Speed = 0 ;
		MotorRight_Speed  = 0;
	}

	/*sending the commands to motor drivers*/
	Motor_SpeedCommand(CANDeviceID_MotorDriver_Right , Forward , MotorRight_Speed);
	Motor_SpeedCommand(CANDeviceID_MotorDriver_Left , Forward , MotorLeft_Speed);

}
