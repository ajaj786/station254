/*
 * utility.h
 *
 *  Created on: 12.12.2020
 *      Author: ALG002-vm
 */

#ifndef HELPER_HELPER_H_
#define HELPER_HELPER_H_


#include "main.h"

float Check_Boundaries(float Input, float LowBoudary, float HighBoudary);
float Scale_PIDSignal(float PIDInput , float DesiredSpeed);


#endif /* HELPER_HELPER_H_ */
