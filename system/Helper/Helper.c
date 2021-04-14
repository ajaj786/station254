/*
 * utility.c
 *
 *  Created on: 12.12.2020
 *      Author: ALG002-vm
 */


/**************Helper**************************/
/**
 * check the boundaries of a signal and give a safe value back
 * @param input			: the input
 * @param HighBoudary	: the HighBoundary of Signal
 * @param LowBoudary	: the LowBoundary of Signal
 * @return				: an output which is between Low and High Boundary
 */
float Check_Boundaries(float Input, float LowBoudary, float HighBoudary)
{
	if(Input < LowBoudary) Input = LowBoudary;
	if(Input > HighBoudary)Input = HighBoudary;

	return Input;
}

/*
 * this function scale the PIDSignal to be used for motor controlling
 * Input -64(min PID Controller) -> 0 (max PID Controller)
 * Output 0 (min Motor speed)  -> 2.3 (m/s)(max motor speed)
 * y = mx+x0
 * m  = DesiredSpeed  / 64
 * x0 = DesiredSpeed
 * */
float Scale_PIDSignal(float PIDInput , float DesiredSpeed)
{
	float output = 0;
	float m =  DesiredSpeed / 64;
	float x0 = DesiredSpeed;

	output = m * PIDInput + x0;

	return output ;

}
