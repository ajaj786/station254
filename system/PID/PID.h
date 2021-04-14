/**
  ******************************************************************************
  * @file           : PID.h
  * @author  		: Mohsen Asadipour
  * @version 		: V1.0
  * @date    		: 23-November-2019
  * @brief          : PID Controller functions
  ******************************************************************************
  */


#ifndef PID_H_
#define PID_H_

/*add your code heir*/
#include "main.h"

extern float MotorLeft_Speed;
extern float MotorRight_Speed;
extern lines Lines;

/***PID Controller***/
double PID_Controller(double In, double Ref, PIDParameters *PIDParams , PIDState *PIDState);
void PIDMotorControl(boolean MoveStart , float DesiredSpeed , double PIDOutput);


#endif /* PID_H_ */



