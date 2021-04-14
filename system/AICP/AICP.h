/**
  ******************************************************************************
  * @file           : AICP.h
  * @author  		: Mohsen Asadipour
  * @version 		: V1.0
  * @date    		: 23-November-2019
  * @brief          : processing AICP Commands
  ******************************************************************************
  */



#ifndef AICP_H_
#define AICP_H_

/*add your code heir*/
#include "main.h"

/*
 * AICP Header
 * for more information see "AICP-EntwicklerAnleitung"
 * */
typedef struct
{
	uint8_t Telegram_Length_MSB;		/*10...65535    Used data length + 10 Byte AICP 1 Header*/
	uint8_t Telegram_Length_LSB;
	uint8_t DestinationAddress;			/*0...25		Node-ID of the connection partner5*/
	char TelegramType;					/*D: Data    P:Pairing(without ack)  A: Ack telegram*/
	char TelegramCounter;				/* '0'...'9' , '/' after start of program*/
	uint8_t ProtocolVersion;			/* version of protocol*/
	uint8_t Reserve1;
	uint8_t Reserve2;
	uint8_t CRC16_LSB;					/*Checksum*/
	uint8_t CRC16_MSB;
}AICP_Header;

/**
 * states of AICP state machine
 */
typedef enum {
	AICP_Idle = 0,				// AICP State machine wait for new Data to process
	AICP_CheckNewData = 1,		// checking the data for validity as AICP Packet
	AICP_CheckValidPacket = 2,	// check AICP Packet information. if the received packet is for this client
	AICP_ProcessData = 3,		// process the received information
	AICP_Reset = 4 				// reset alle internal state variables and start from idle
}AICP_States;

extern uint8_t AICP_State;
extern char AICP_NewReceicedData[Size_DataBuffer];
extern boolean UART_NewDataToInterprate ;
extern AICP_Header AICPReceivedHeader[10];				//all received headers would be saved heir
extern char UART_ReceicedData[300];

void AICP_StateMachine();
boolean AICP_CheckCRC(uint8_t AICPData[]);
int16_t AICP_FindPacketHeader();
void AICP_CopyDataToAICPBuffer();
AICP_Header AICP_ExtractHeader (uint16_t PacketStartIndex);



#endif /* AICP_H_ */



