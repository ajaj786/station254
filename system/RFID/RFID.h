/**
  ******************************************************************************
  * @file           : RFID.h
  * @author  		: Mohsen Asadipour
  * @version 		: V1.0
  * @date    		: 23-November-2019
  * @brief          : Receiving and processing RFID
  ******************************************************************************
  */



#ifndef RFID_H_
#define RFID_H_


#include "main.h"

/*
 * RFID Code
 * */
typedef struct
{
 char Code[10];
}RFID_Code;


extern RFID_Code RFID_ReceivedData ;
extern uint8_t ConsistencyCounter_FirstHalfPacket;
extern uint8_t ConsistencyCounter_SecondHalfPacket;
extern boolean RFID_Received;

void  RFIF_ReadRFIDCode( uint8_t CANData[] , boolean SecondHalfData );
void RFID_ClearState();


#endif /* RFID_H_ */



