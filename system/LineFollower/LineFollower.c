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


/**************LineFollowing**************************/

/*write all movement code heir*/
/**
 * toDo: backward movement
 * toDo: curves
 * toDo: cross overs
 */

/**
 * move without controlling
 * the movements command are based on simple state of line scanner sensor. there is not any PID controller in it.
 * in order to have some PID controlling , the "Move_S2" should be used
 */
void Move_S1(boolean MoveStart)
{

	boolean direction;
	float speed_Left ;
	float speed_Right ;


	mLineCenterDiff = LineCenterDiff;		//saving the last lineCenterDif into memory
	LineCenterDiff = LineCenter - Lines.Line[0] ;


	if(MoveStart && Lines.NumberFoundLines != 0)
	{
		if(-LineLimits < LineCenterDiff  && LineCenterDiff < LineLimits)	//stright away
		{
			/*calculate der. in green window*/
			LineCenterDiff_Derivative = LineCenterDiff - mLineCenterDiff;

			GetMaxDerivative (LineCenterDiff_Derivative); //getting the max derivative


			direction = Forward ;


			if (! InGreenWindow) //both motors with max speed
			{
				speed_Left = cMotorMaxSpeedMPS;
				speed_Right = cMotorMaxSpeedMPS;
			}
			else	//doing some speed correction on one motor
			{
				if (LineCenterDiff_Derivative > 0) //reduce left motor driver speed
				{
					//speed correction left
					speed_Left = Calc_SpeedCorrection(cMotorMaxSpeedMPS , LineCenterDiff_Derivative);
					speed_Right = cMotorMaxSpeedMPS;
				}
				else if (LineCenterDiff_Derivative < 0) //reduce right motor driver speed
				{
					//speed correction right
					speed_Left = cMotorMaxSpeedMPS;
					speed_Right = Calc_SpeedCorrection(cMotorMaxSpeedMPS , LineCenterDiff_Derivative);
				}
				else
				{
					speed_Left = cMotorMaxSpeedMPS;
					speed_Right = cMotorMaxSpeedMPS;
				}


			}


			InGreenWindow = True; //in the next iteration the robot calculate the speed better
		}
		else if (LineCenterDiff < -LineLimits ) //divergence to right => the left motor should be slower
		{
			direction = Forward ;
			speed_Left = 0;
			speed_Right = cMotorMaxSpeedMPS;

			InGreenWindow = False;	//robot is out of green windows
		}
		else if (LineCenterDiff > LineLimits) //divergence to left => the right motor should be slower
		{
			direction = Forward ;
			speed_Left = cMotorMaxSpeedMPS;
			speed_Right = 0;

			InGreenWindow = False; //robot is out of green windows
		}
	}
	else
	{
		direction = Forward ;
		speed_Left = 0;
		speed_Right = 0;
	}

	/*sending the commands to motor drivers*/
	Motor_SpeedCommand(CANDeviceID_MotorDriver_Right , direction , speed_Right);
	Motor_SpeedCommand(CANDeviceID_MotorDriver_Left , direction , speed_Left);
}

/**
 * move with PID controller
 * the movement commands are based on a PID controller that get line scanner signal as its input
 * this function is based on "Test_Scenario8_DirectionPIDController"
 */
void Move_S2(boolean MoveStart , uint8_t mLine)
{

	PID_Output =  PID_Controller(mLine , 64 , &PIDParams_Dir , &PIDStateData_Dir);

	PIDMotorControl(MoveStart , DesiredSpeed , PID_Output);
}

/**
 * in this move scenario, the input would be feet directly into the functions
 * and the PID correction would be implemented on Motor Signals
 * just one wheel would be turned slower than the DesiredSpeed in order to correct the direction
 * @param MoveStart
 */
void Move_S3(boolean MoveStart)
{
	//get the Line Signal

	//calculate the diversion from each center for each motor
	 MotorLeft_Diversion = Calculate_DiversionFromCenter(cLineScanner_SensorCenter, Lines.Line[0], MotorPosition_Left);
	 MotorRight_Diversion = Calculate_DiversionFromCenter(cLineScanner_SensorCenter, Lines.Line[0], MotorPosition_Right);

	//calculate PID signals for motors
	 MotorLeft_PIDSignal  = PID_Controller(MotorLeft_Diversion , 0 , &PIDParams_Motor , &PIDStateData_Motor);
	 MotorRight_PIDSignal = PID_Controller(MotorRight_Diversion , 0 , &PIDParams_Motor , &PIDStateData_Motor);

	 //scale PIDSignal
	 MotorLeft_PIDSignal_Scaled = Scale_PIDSignal(MotorLeft_PIDSignal , DesiredSpeed);
	 MotorRight_PIDSignal_Scaled = Scale_PIDSignal(MotorRight_PIDSignal, DesiredSpeed);

	 //check the boundaries of motor signals
	 MotorLeft_PIDSignal_Scaled = Check_Boundaries(MotorLeft_PIDSignal_Scaled, cMotorMinSpeedMPS, cMotorMaxSpeedMPS);
	 MotorRight_PIDSignal_Scaled = Check_Boundaries(MotorRight_PIDSignal_Scaled, cMotorMinSpeedMPS, cMotorMaxSpeedMPS);

	//implement the signals on Motors
	Motor_SpeedCommand(CANDeviceID_MotorDriver_Left , Forward , MotorLeft_PIDSignal_Scaled);
	Motor_SpeedCommand(CANDeviceID_MotorDriver_Right , Forward , MotorRight_PIDSignal_Scaled);
}

/*
 * in this strategy both motors would become a correction for speed and one would turn slower and the other one faster
 * there would not be any PID Controller but using simple linear coeffcient for different deviation from center
 * */
void Move_S4(boolean MoveStart, uint8_t LinePosition)
{
	//calculate the diversion from center
	AGVDeviation = Calculate_DiversionFromCenter_S4(cLineScanner_SensorCenter, LinePosition, False );
	CurveRecognition (AGVDeviation, LinePosition );

	if (CurveDetected == True ){

		float CurveSpeed = 0.11;
		if (RightCurve) mCurve = 1;
		if (LeftCurve) mCurve = 2;
		AGVDeviation = derivative_1_array[19];
		AGVDeviationDegree = Calculate_DeviationDegree(AGVDeviation, 30 ,cLineScanner_HighstPixel  );
		//AGVDeviationCoefficient = Calculate_DeviationCoefficient(AGVDeviationDegree);
		if (RightCurve || (mCurve == 1) ){
			InnerCoefficient = Calculate_InnerCoefficient(AGVDeviationDegree);
			OuterCoefficient = Calculate_OuterCoefficient(AGVDeviationDegree);
			if(OuterCoefficient < 0.2){
				OuterCoefficient = OuterMotorDeviationCoefficient[34];
			}
			MotorLeft_Speed = Calculate_MotorSpeed(AGVDeviation, MotorPosition_Left, True , True , OuterCoefficient, CurveSpeed);
			MotorRight_Speed =Calculate_MotorSpeed(AGVDeviation, MotorPosition_Right, True , True , InnerCoefficient, CurveSpeed);
		}
		else if (LeftCurve || (mCurve == 2)) {
			InnerCoefficient = Calculate_InnerCoefficient(AGVDeviationDegree);
			OuterCoefficient = Calculate_OuterCoefficient(AGVDeviationDegree);
			if(OuterCoefficient < 0.2){
				OuterCoefficient = OuterMotorDeviationCoefficient[34];
			}
			MotorLeft_Speed = Calculate_MotorSpeed(AGVDeviation, MotorPosition_Left, True , True , InnerCoefficient, CurveSpeed);
			MotorRight_Speed =Calculate_MotorSpeed(AGVDeviation, MotorPosition_Right, True , True , OuterCoefficient, CurveSpeed);
		}


	}
	else{
		mCurve = 0;
		//calculate the turn coefficient for both motors
		AGVDeviationDegree = Calculate_DeviationDegree(AGVDeviation, 15 ,cLineScanner_HighstPixel  );


		//calculate the diversion coefficient
		AGVDeviationCoefficient = Calculate_DeviationCoefficient(AGVDeviationDegree);


		//calculate each motor speed
		MotorLeft_Speed = Calculate_MotorSpeed(AGVDeviation, MotorPosition_Left, False, False, AGVDeviationCoefficient, DesiredSpeed);
		MotorRight_Speed = Calculate_MotorSpeed(AGVDeviation, MotorPosition_Right, False, False, AGVDeviationCoefficient, DesiredSpeed);
	}
	 if(MoveStart)
	 {
		 //implement the motor speed
		Motor_SpeedCommand(CANDeviceID_MotorDriver_Left , Forward , MotorLeft_Speed);
		Motor_SpeedCommand(CANDeviceID_MotorDriver_Right , Forward , MotorRight_Speed);
	 }
	 else
	 {
		 //stop both motors
		Motor_SpeedCommand(CANDeviceID_MotorDriver_Left , Forward , cMotorStopSpeed);
		Motor_SpeedCommand(CANDeviceID_MotorDriver_Right , Forward , cMotorStopSpeed);
	 }
}



/* Combining Move_S2 and Move_S4 task. using straight line array to move on the straight line where as PID on the curve
 */
void Move_S5(boolean MoveStart, uint8_t mLine){
	//calculate the diversion from center
	AGVDeviation = Calculate_DiversionFromCenter_S4(64, mLine, False );
	CurveRecognition (AGVDeviation, mLine );

	if ((MightBeOscillating==1)|| OnStraightLine){
		AGVDeviationDegree = Calculate_DeviationDegree(AGVDeviation, 15 ,cLineScanner_HighstPixel  );
		AGVDeviationCoefficient = Calculate_DeviationCoefficient(AGVDeviationDegree);
		MotorLeft_Speed = Calculate_MotorSpeed(AGVDeviation, MotorPosition_Left, False, False, AGVDeviationCoefficient, DesiredSpeed);
		MotorRight_Speed = Calculate_MotorSpeed(AGVDeviation, MotorPosition_Right, False, False, AGVDeviationCoefficient, DesiredSpeed);
		PID_Output =  PID_Controller(mLine , 64 , &PIDParams_Dir , &PIDStateData_Dir);
		if(MoveStart)
			{
				//implement the motor speed
				Motor_SpeedCommand(CANDeviceID_MotorDriver_Left , Forward , MotorLeft_Speed);
				Motor_SpeedCommand(CANDeviceID_MotorDriver_Right , Forward , MotorRight_Speed);
			}
		else
			{
				//stop both motors
				Motor_SpeedCommand(CANDeviceID_MotorDriver_Left , Forward , cMotorStopSpeed);
				Motor_SpeedCommand(CANDeviceID_MotorDriver_Right , Forward , cMotorStopSpeed);
			}
	}
	else{
		PID_Output =  PID_Controller(mLine , 64 , &PIDParams_Dir , &PIDStateData_Dir);

		PIDMotorControl(MoveStart , DesiredSpeed , PID_Output);
	}
}

/*Using Vector angle from motor EncoderData */
void Move_S6(boolean MoveStart, uint8_t mLine, boolean UseDist ){

	AGVDeviation =  Calculate_DiversionFromCenter_S4(60, mLine, False );
	//problem  with line sensor. does not properly read the left hand side values, i.e, values less than 64, doesnot read values below 10. So need to specifically give a different mapping input of -55 to 55.
	MappedAGVDeviation[0] = map(AGVDeviation, -50,50,-100 , 100);
	MappedAGVDeviation[1] = MappedAGVDeviation[0];
	if (MappedAGVDeviation[0] < 0 )MappedAGVDeviation[1] = MappedAGVDeviation[0] * -1;

	int MappedAGVDeviaitonThresholdMin = map( -2 ,-64,64,-100 , 100);
	int MappedAGVDeviaitonThresholdMax = map( 2 ,-64,64,-100 , 100);

	int CorrectionMaxArray[7]= {3000,5000,7000,9000,12000,15000,17000};
	int CorrectionMinArray[7]= {1500,2500,3500,4500,5500,7500,8500};
	float SpeedArray[7] = {0.1,0.12,0.15,0.18,0.2,0.22,0.25};

	int straightThresholdMin = -30;
	int straightThresholdMax = 30;
	float MappedSpeed = 0;
	int CorrectionMax = 5000; //DesiredSpeed= 0.1 Use>>3000, 0.12>>5000; 0.15>>7000; 0.18>>9000;0.2>>12000;0.22>>15000;0.25>>17000;
	int CorrectionMin = 2500; // DesiredSpeed = 0.1 Use>>1500; 0.12>>2500; 0.15>>3500;0.18>>4500;0.2>>6000;0.22>>7500;0.25>>8500;

	int LengthofArray = sizeof(SpeedArray)/sizeof(SpeedArray[0]);
//	for (int k = 0 ; k < LengthofArray ; k++){
//		if(SpeedArray[k] <= DesiredSpeed ){
//			CorrectionMax = CorrectionMaxArray[k];
//			CorrectionMin = CorrectionMinArray[k];
//		}
//	}


	//Calculate EncoderPulses based on distance.
	float PulsePerRevolution = 450.0;
	float WheelCircumference = 3.14 * 2 * WheelRadius * 1000;//in mm
	NumberOfRevolution = 250.0 / (WheelCircumference);
	//TargetEncoderCounter = NumberOfRevolution * PulsePerRevolution;

	//	int TimeDiffBetweenPackageRecieved = LeftMotorArray[Time_Arr][2] - RightMotorArray[Time_Arr][2] ;

	DiffInLastMotorReading = LeftMotorArray[TICK_Arr][2] - RightMotorArray[TICK_Arr][2];
	VectorAngle = atan2(DiffInLastMotorReading,370);

	if (Move_S6Called== True) {
	//		ResetEncoderRight = True;
	//		ResetEncoderLeft =True;
			Move_S6Called = False;
			//MoveStart = True;
			Move_S6CalledTime = HAL_GetTick();
		}

	int ForNegativeAGVDeviation = map(MappedAGVDeviation[0], -100,0,CorrectionMin,CorrectionMax);
	int ForPositiveAGVDeviation =map(MappedAGVDeviation[0], 100,0,CorrectionMin,CorrectionMax);
	MappedSpeed = map (MappedAGVDeviation[1],straightThresholdMax, 0, 0.1,DesiredSpeed);

	if (MappedAGVDeviation[0] > straightThresholdMax){
		MotorLeft_Speed = 0.1 - VectorAngle  + ((MappedAGVDeviation[0] )/3000);
		MotorRight_Speed = 0.1+  VectorAngle - (MappedAGVDeviation[0] /1500);
		TempSpeed[0] = 0.1;
	}else if (MappedAGVDeviation[0] < straightThresholdMin) {
		MotorLeft_Speed = 0.1 -  VectorAngle + ((MappedAGVDeviation[0] )/1500);// mappedAGVDeviation values are negative
		MotorRight_Speed = 0.1 + VectorAngle -  ((MappedAGVDeviation[0] )/3000);//Also the values vector angle are negative
		TempSpeed[0] = 0.1;
	} else {
//		if ((TempSpeed_1 >= DesiredSpeed)|| (TempSpeed[0] == 0.1)){
//			TempSpeed[0] = TempSpeed[0] + 0.0005;
//			DesiredSpeed = TempSpeed [0];
//		}
		if (MappedAGVDeviation[0] >= 0){
			MotorLeft_Speed =/*(DesiredSpeed*(1-((DesiredSpeed - MappedSpeed)/ForPositiveAGVDeviation)))*/(DesiredSpeed*(1-(1/ForPositiveAGVDeviation))) - /*((VectorAngle)/2)*/(VectorAngle) + MappedAGVDeviation[0] /CorrectionMax ; //use value 3000 for desired speed
			MotorRight_Speed =/*(DesiredSpeed*(1-((DesiredSpeed - MappedSpeed)/ForPositiveAGVDeviation))) */(DesiredSpeed*(1-(1/ForPositiveAGVDeviation))) +  /*((VectorAngle)/2)*/(VectorAngle)-MappedAGVDeviation[0] /ForPositiveAGVDeviation; /*MappedAGVDeviation[0] /3000;*/
		}else{
			MotorLeft_Speed =/*(DesiredSpeed*(1-((DesiredSpeed - MappedSpeed)/ForNegativeAGVDeviation)))*/(DesiredSpeed*(1-(1/ForNegativeAGVDeviation))) - /*((VectorAngle)/2)*/ (VectorAngle)+ MappedAGVDeviation[0] /ForNegativeAGVDeviation ; /*MappedAGVDeviation[0] /3000 ;*/
			MotorRight_Speed =/*(DesiredSpeed*(1-((DesiredSpeed - MappedSpeed)/ForNegativeAGVDeviation)))*/(DesiredSpeed*(1-(1/ForNegativeAGVDeviation))) +  /*((VectorAngle)/2)*/(VectorAngle) - MappedAGVDeviation[0] /CorrectionMax;
		}
	}
//	if (MappedAGVDeviation[0] >= 0){
//		MotorLeft_Speed = (DesiredSpeed*(1-(1/ForPositiveAGVDeviation))) - ((VectorAngle)) + MappedAGVDeviation[0]/CorrectionMax ;
//		MotorRight_Speed = (DesiredSpeed*(1-(1/ForPositiveAGVDeviation))) +  ((VectorAngle))- MappedAGVDeviation[0] /ForPositiveAGVDeviation;
//	}else{
//		MotorLeft_Speed = (DesiredSpeed*(1-(1/ForNegativeAGVDeviation))) - ((VectorAngle)) + MappedAGVDeviation[0] /ForNegativeAGVDeviation ;
//		MotorRight_Speed = (DesiredSpeed*(1-(1/ForNegativeAGVDeviation))) +  ((VectorAngle)) -  MappedAGVDeviation[0]/CorrectionMax;
//	}


	if (/*((LeftMotorArray[TICK_Arr][1] < TargetEncoderCounter)|| (RightMotorArray[TICK_Arr][1] < TargetEncoderCounter)) && (!PackageDelay)*/(MoveStart)){
		Motor_SpeedCommand(CANDeviceID_MotorDriver_Left , Forward , MotorLeft_Speed);
		Motor_SpeedCommand(CANDeviceID_MotorDriver_Right , Forward , MotorRight_Speed);
	}else{
		Motor_SpeedCommand(CANDeviceID_MotorDriver_Left , Forward , 0);
		Motor_SpeedCommand(CANDeviceID_MotorDriver_Right , Forward , 0);
	}
}
//
/**
 * calculate the modified speed based on speed diff derivation
 * return for Der. 0 the Max Speed
 * return for Der. Max(LineLimit * 2) 1/2 of max speed
 * Speed = (-0.5 MaxSpeed / 2 LineLimit).Der + MaxSpeed
 * @param Speed_maxDesired
 * @param LineCenterDiff_Derivative
 * @return the correct speed
 */
float Calc_SpeedCorrection(float Speed_maxDesired , int8_t LineCenterDiff_Derivative)
{
	float speed = 0 ;

	speed = (-1 / (2 * LineLimits )) * LineCenterDiff_Derivative + Speed_maxDesired ;

	return speed;
}

/**
 * its a test function to calculate the maximum derivative that can be captured by robot
 * @param Derivative
 */
void GetMaxDerivative (int8_t Derivative)
{
	if(Derivative > Derivative_Max)
	{
		Derivative_Max = Derivative;
	}
}

/**
 * calculate the sensor diversion from middle of the line for each axis
 * 0 (min) - 64 (max)
 * @param SensorCenter
 * @param LinePosition
 * @param MotorPosition
 * @return
 */
uint8_t Calculate_DiversionFromCenter(uint8_t SensorCenter , uint8_t LinePosition , boolean MotorPosition)
{
	int8_t Diverstion = 0;

	/*calculate the diversion based on the position of motor*/
	if (MotorPosition == MotorPosition_Right)
		Diverstion = SensorCenter - LinePosition ;

	if (MotorPosition == MotorPosition_Left)
		Diverstion = LinePosition - SensorCenter ;

	//the diversion can not be smaller than 0
	if(Diverstion < 0 )
		Diverstion = 0;

	return Diverstion ;
}

/**
 * calculate the diversion from the center of sensor
 * @param SensorCenter : the sensor is an array of 128 pixels and the middle is 64
 * @param LinePosition : the position of line received over CAN
 * @param Reverse	   : Reversely Calculate the deviation of center
 * @return a value between -64 to +64
 */
int8_t Calculate_DiversionFromCenter_S4(uint8_t SensorCenter , uint8_t LinePosition , boolean Reverse )
{
	int8_t result ;

	if (Reverse )
		result = LinePosition - SensorCenter;
	else
		result = SensorCenter -LinePosition;

	return result;
}

/**
 * Calculate the degree of deviation form center
 * the higher value means more deviation form center
 * @param AGVDeviation
 * @param Subdivisions
 * @return
 */
uint8_t Calculate_DeviationDegree(int8_t AGVDeviation , uint8_t Subdivisions , uint8_t SensorHighstPixel)
{
	uint8_t DeviationDegree = 0 ;
	uint8_t SensorLength = SensorHighstPixel + 1 ;
	uint8_t DevisionLegth = (SensorLength/2) / Subdivisions;

	DeviationDegree = fabs(AGVDeviation) / DevisionLegth ;

	return DeviationDegree ;
}

/**
 * calculate the deviation coefficient based on the deviation degree
 * this coefficient would be used to adjust the speed of each wheel
 * it just peak the index of an array that contain the right coefficient for that deviation
 * @param AGVDeviationDegree
 * @return
 */
float Calculate_DeviationCoefficient(uint8_t AGVDeviationDegree)
{
	//control the index of array
	if ((DeviationCoefficient_MinIndex <= AGVDeviationDegree) && (AGVDeviationDegree <= DeviationCoefficient_MaxIndex))
		return DeviationCoefficient[AGVDeviationDegree];
	else
		return 0 ;
}

float Calculate_InnerCoefficient (uint8_t AGVDeviationDegree ){
	if(pt_eight_curve){
		if ((pt_eight_InnerMotorDeviationCoefficient_MinIndex <= AGVDeviationDegree) && (AGVDeviationDegree <= pt_eight_InnerMotorDeviationCoefficient_MaxIndex))
			return pt_eight_InnerMotorDeviationCoefficient[AGVDeviationDegree];
		else
			return 0 ;
	}else{
		if ((InnerMotorDeviationCoefficient_MinIndex <= AGVDeviationDegree) && (AGVDeviationDegree <= InnerMotorDeviationCoefficient_MaxIndex))
			return InnerMotorDeviationCoefficient[AGVDeviationDegree];
		else
			return 0 ;
	}

}

float Calculate_OuterCoefficient (uint8_t AGVDeviationDegree ){
	if(pt_eight_curve){
		if ((pt_eight_OuterMotorDeviationCoefficient_MinIndex <= AGVDeviationDegree) && (AGVDeviationDegree <= pt_eight_OuterMotorDeviationCoefficient_MaxIndex))
			return pt_eight_OuterMotorDeviationCoefficient[AGVDeviationDegree];
		else
			return 1 ;
	}else{
		if ((OuterMotorDeviationCoefficient_MinIndex <= AGVDeviationDegree) && (AGVDeviationDegree <= OuterMotorDeviationCoefficient_MaxIndex))
			return OuterMotorDeviationCoefficient[AGVDeviationDegree];
		else
			return 1 ;
	}

}


/**
 * calculate the speed correction based on deviation area coefficient and motor position
 * @param AGVDeviation
 * @param MotorPosition
 * @param AGVDeviationCoefficient
 * @param DesiredSpeed
 * @return
 */
float Calculate_MotorSpeed(int8_t AGVDeviation , boolean MotorPosition, boolean RightCurve, boolean LeftCurve
		, float AGVDeviationCoefficient,float DesiredSpeed )
{
	float MotorSpeed = 0;
	float Assigning_speed = 0;
	float correction = 0;
	float variableCorrection = 0;

	//calculate the correction

//	if (RightCurve == True || LeftCurve ==True ){
//
//		variableCorrection = AGVDeviationCoefficient/5;
//		correction  =  AGVDeviationCoefficient ;
//		Assigning_speed = DesiredSpeed *0.9;       // reducing the speed at the curves.
//		RightCurve = False;
//		LeftCurve = False;
//	}
//	else 	{
		correction  =  AGVDeviationCoefficient ;
		variableCorrection = AGVDeviationCoefficient/5;
		Assigning_speed = DesiredSpeed;
//	}


	//findout the effect on corresponding motor
	if(MotorPosition == MotorPosition_Left)
	{
		if(AGVDeviation <= 0) //line on the left side of center
			MotorSpeed = Assigning_speed - variableCorrection*  Assigning_speed;	//slow down speed
		if(AGVDeviation > 0) //line on the right side of center
			MotorSpeed = Assigning_speed + correction *  Assigning_speed;	// speed up
	}

	if(MotorPosition == MotorPosition_Right)
	{
		if(AGVDeviation <= 0) //line on the left side of center
			MotorSpeed = Assigning_speed + correction *  Assigning_speed;	// speed up
		if(AGVDeviation > 0) //line on the right side of center
			MotorSpeed = Assigning_speed - variableCorrection *  Assigning_speed;	//slow down speed
	}

	//check the speed to be in the right area
	MotorSpeed = Check_Boundaries(MotorSpeed, cMotorMinSpeedMPS, cMotorMaxSpeedMPS);

	return MotorSpeed;
}

void CurveRecognition (uint8_t CurrentPosition  , uint8_t LinePosition ){

	/* if CurrentDeviationDervt_1 is +ve then line is on the right side,
		 * if -ve then line is on the left side of the sensor
		 * 64-60 = 4 | 64-58= 6 | 64-40 = 14  line on Right side
		 * 64-70 = -6 | 64-90 = -26 | 64-100 = -36   line on the Left Side*/
	uint8_t CurrentDeviationDervt_1 = 0 ;
//	uint8_t ActualDerivative_1_helper = LinePosition;


//	char tempState_2;
	int  arrayfillcheck = 0;
	boolean derivative_1_arrayfilled;
	boolean PossibleRightCurve;
	boolean PossibleLeftCurve;


	int8_t straightline_boundary_min = -25;
	int8_t straightline_boundary_max = 25;

	CurrentDeviationDervt_1 = CurrentPosition;
	// changing the value in the array ONLY if there is a change in the line value
	if (CurrentDeviationDervt_1 != temp_1){

		temp_1 = CurrentDeviationDervt_1;

		/*move the value to the left in the array and make space for the new value at the end of the array*/
		for (int k = 1; k < 20; k ++){
			derivative_1_array[k-1]=derivative_1_array[k];
			}
		derivative_1_array[19] = CurrentDeviationDervt_1 ;
		arrayfillcheck = arrayfillcheck + 1;


	}
//
//	if (ActualDerivative_1_helper != temp_2){
//
//		temp_2 = ActualDerivative_1_helper;
//		derArray_temp [1] = derArray_temp[0];
//		derArray_temp[0] = LinePosition;
//		for (int b =0; b < 19; b++){
//			ActualDerivative_1[b] = ActualDerivative_1[b+1];
//		}
//
//		ActualDerivative_1[19] =derArray_temp[0] - derArray_temp[1];
//	}

	if (arrayfillcheck >= 20) derivative_1_arrayfilled = True;

	if (derivative_1_arrayfilled){
//		max = derivative_1_array[0];
//		for(int i=1; i<20; i++){
//			if(derivative_1_array[i] > max){
//				max = derivative_1_array[i];
//			}
//
//			min = derivative_1_array[0];
//			for(int i=1; i<20; i++){
//					if(derivative_1_array[i] < min){
//						min = derivative_1_array[i];
//					}
//				}
//		if ((min <= (-50)) && (max >= 50)){
//			MightBeOscillating = 1;
//		}
//		if ((MightBeOscillating >3) && CurveDetected ){
//			CurveDetected =False;
//		}

		//checking the last 5 values of the derivative_1_array.
		/* here the array will be like [-9,-6,-4,4,9,14,16,18,19,20] these values mean that the line is on the right side
		 * hence if the same trend follows for the array value of greater than -10 value that means you might be in a Right curve */

		if ((derivative_1_array[19] > derivative_1_array[18]) && (derivative_1_array[18] > derivative_1_array[17])
				&& (derivative_1_array[17] > derivative_1_array[16])&& (derivative_1_array[16] > derivative_1_array[15])&& (derivative_1_array[15] > derivative_1_array[14])){

			RightCurvevaluecheck = 1;
			PossibleRightCurve = True;
		}
		else{
			RightCurvevaluecheck = 0;
			PossibleRightCurve = False;
		}
			/* here the array will be like [21,19,16,9,5,1,-1,-3,-7,-10] these values mean that the line is on the left side
			 * hence if the same trend follows for the array value of greater than -10 value that means you might be in a left curve */

		if(((derivative_1_array[19]) < (derivative_1_array[18])) && ((derivative_1_array[18]) < (derivative_1_array[17]))
				&& ((derivative_1_array[17]) < (derivative_1_array[16]))&& ((derivative_1_array[16]) < (derivative_1_array[15]))&& ((derivative_1_array[15]) < (derivative_1_array[14])))	{
			LeftCurvevaluecheck =  1;
			PossibleLeftCurve = True;
		}
		else {
			PossibleLeftCurve = False;
			LeftCurvevaluecheck = 0;
		}

		if( (((straightline_boundary_min <= (derivative_1_array[19]))&&((derivative_1_array[19]) <= straightline_boundary_max))&&
				((straightline_boundary_min <= (derivative_1_array[18]))&&((derivative_1_array[18]) <= straightline_boundary_max))&&
				((straightline_boundary_min <= (derivative_1_array[17]))&&((derivative_1_array[17]) <= straightline_boundary_max))&&
				((straightline_boundary_min <= (derivative_1_array[16]))&&((derivative_1_array[16]) <= straightline_boundary_max))&&
				((straightline_boundary_min <= (derivative_1_array[15]))&&((derivative_1_array[15]) <= straightline_boundary_max)))){
			OnStraightLine = True;
			//MightBeOscillating = 0;
			straightlineCounter = straightlineCounter +1;
			tempState_1 = 100;
		}
		else 			OnStraightLine= False;

	}


	if (PossibleLeftCurve && (derivative_1_array[19] < -30 )){
		LeftCurve = True;
		tempState_1 = 50;
	}else LeftCurve = False;

	if (PossibleRightCurve && (derivative_1_array[19] >30 )){
		RightCurve = True;
		tempState_1 = 150;
	}else RightCurve = False;

	if (tempState_1 != tempState_2){
		tempState_2 = tempState_1;
		tempState [1] = tempState [0];
		tempState[0]  = tempState_1;
	}


	if (((tempState[1] == 50)&( tempState[0]== 150)) || ((tempState[1] == 150)&( tempState[0 ]==50 ))){
			MightBeOscillating = 1;
			Oscillator_count = Oscillator_count + 1;
	}
	/*else MightBeOscillating = 0;*/

	if ((CurveDetected && OnStraightLine) ){
		CurveDetected = False;
		MightBeOscillating = 0;
		//OnStraightLine= False;

	}

	if (LeftCurve == True || RightCurve == True || Curve_present == True){
			CurveDetected = True;
			//MightBeOscillating = 0;
			OnStraightLine= False;

			straightlineCounter = 0;
		}
}

// input values ​​are: input value, a minimum value input, the maximum input, output minimum, maximum output
float map(float val, float I_Min, float I_Max, float O_Min, float O_Max){
	if (val < I_Min) val = I_Min;
	if(val > I_Max) val = I_Max;
		return(((val-I_Min)*((O_Max-O_Min)/(I_Max-I_Min)))+O_Min);
   }





