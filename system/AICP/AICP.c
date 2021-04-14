/**
  ******************************************************************************
  * @file           : AICP.c
  * @author  		: Mohsen Asadipour
  * @version 		: V1.0
  * @date    		: 23-November-2019
  * @brief          : processing AICP Commands
  ******************************************************************************
  */

#include <system.h>

/**
 * this state machine implement the neccessary processes for AICP
 * Reference: read the "Diagram/AICP-DataFlow" to fully understand the data flow of this state machine
 */
void AICP_StateMachine()
{
	int16_t Index_AICPHeaderStart ;

	switch (AICP_State)
	{
		case(AICP_Idle):	//wait for receiving new Data. copy new data to AICP input Data Buffer
				if(UART_NewDataToInterprate)
				{
					AICP_CopyDataToAICPBuffer();


					UART_NewDataToInterprate = False;	//reset the UART Flag
					AICP_State = AICP_CheckNewData;
				}
				return;
		case(AICP_CheckNewData):
				//check to find the start of AICP packet
				Index_AICPHeaderStart = AICP_FindPacketHeader();


				if(Index_AICPHeaderStart >= 0)
				{
					//extract header
					AICPReceivedHeader[0] = AICP_ExtractHeader(Index_AICPHeaderStart);

					//extract payload

					AICP_State = AICP_CheckValidPacket;
				}
				else // the return value was a minus value which means error
				{
					AICP_State = AICP_Reset;
				}
				return;
		case(AICP_CheckValidPacket):
					//extract header

					//extract payload

					AICP_State = AICP_Idle;
				return;
		case(AICP_ProcessData):
				return;
		case(AICP_Reset):	//reseting all state variables
				Index_AICPHeaderStart = 0 ;

				AICP_State = AICP_Idle;
				return;
		default:
				AICP_State = AICP_Idle;
				return;




		osDelay(10);


	}
}

/*
 * check the currectness of CRC Code
 * Note:
 * 1-   on the sender side the crc would be implemented on 8 bytes and would be added at the end of header
 *      but in the receiver side the crc would be implemented on whole 10 bytes and if the result is 0 then the packet is currectly received and if its not 0 the packet has some errors
 *
 * 2-   its also possible to change the parameter of this function to accept a AICP_Header struct but
 * 		this funciton is supposed to process an stream of bytes and based on 0 result of CRC check find the start of a data packet
 * 		and its more conveniet to have an array of bytes as input
 *
 */
boolean AICP_CheckCRC(uint8_t AICPData[])
{
      uint16_t CRC16 = 0x0000;
      uint16_t Polynome = 0xA001;
      uint16_t i = 0;  //counter for the data bytes
      uint16_t n = 0;  //counter for the data bits
      boolean bCRCTrue = False;
      boolean flag;
      uint8_t headerLength = 10 ;	//10 is the length of a AICP Header. this length is a part of standard


      while (i < headerLength)
      {
          CRC16 ^= AICPData[i];
          n = 0;

          do
          {
              flag = (CRC16 & 1) > 0;  //ToDo: accourding to the flow chart in "AICP-Entwickleranleitung"  this line should be after the next line but it works currectly like this, find the reason
              CRC16 >>= 1;

              if (flag)
                  CRC16 ^= Polynome;

              n++;
          } while (n <= 7);

          i++;
      }

      /*the receiver would apply crc on the whole block
        and if the result was 0 then we dont have any error
        and its the start of a new AICP data packet*/
      if (CRC16 == 0)
      {
    	  bCRCTrue = True;

    	  //put the CRC into the received header data structure
      }

      return bCRCTrue;
}

/**
 * check the AICP received data array "AICP_NewReceicedData" and scann the array to find the start header of a AICP Data packet
 * for each index of array a AICP_CheckCRC would be implemented and if this function return true it means that it has found the start fo AICP Data packet
 * @return: a positive value as the index of start of AICP Header if it was found
 * 			-1 if no AICP Heade was found
 */
int16_t AICP_FindPacketHeader()
{
	uint8_t tempAICPHeader[10] ;
	uint16_t LastValidIndex ;
	boolean Flag_AICPStart_Found = False;
	uint16_t LastNotZeroIndex = 0;
	uint16_t NumberOfZeros = 0;

	/*get the last valid index*/
	for(uint16_t i= 0 ; i < Size_DataBuffer;i++)
	{
		NumberOfZeros = 0;	//reset zero counter

		for(uint16_t j = i ; j < Size_DataBuffer;  j++)
		{
			if(AICP_NewReceicedData[j] == 0)
					NumberOfZeros ++;
		}

		if(NumberOfZeros == (Size_DataBuffer - i))
		{
			LastNotZeroIndex = i ;
			break;
		}
	}

	/*check: the Last not Zero data should be in a index bigger than 9, if not it means this packet is not an AICP Data Packet*/
	if(LastNotZeroIndex < Size_AICPHeaderLength-1)
		return -1;

	/*calculate the LastValidIndex*/
	if(LastNotZeroIndex < Size_DataBuffer -1 )		//the calculated LastValidIndex is correct
		LastValidIndex = LastNotZeroIndex - (Size_AICPHeaderLength-1);
	else 											//the calculated LastValidIndex is NOT correct
		return -1;



	for(uint16_t i = 0 ; i < LastValidIndex ; i++)
	{
		//extract 10 byte to check if its AICP header
		for(uint8_t j = 0 ; j < 10 ; j++)
		{
			tempAICPHeader[j] = AICP_NewReceicedData[j+i];
		}

		//check if its the start of packet
		Flag_AICPStart_Found = AICP_CheckCRC(tempAICPHeader);
		if(Flag_AICPStart_Found)
			return i;
	}

	return -1;
}

/*AICP Helpers*/

/**
 * copy data from UART input buffer "UART_ReceicedData" to AICP input buffer "AICP_NewReceicedData"
 */
void AICP_CopyDataToAICPBuffer()
{
	for(uint16_t i = 0 ; i < Size_DataBuffer; i++)
	{
		AICP_NewReceicedData[i] = UART_ReceicedData[i] ;
	}
}

/**
 * extract the header of AICP from input array "AICP_NewReceicedData" based on the start index
 * @param PacketStartIndex: index of first telegram byte in the received stream form uart
 * @return a structure AICP_Header
 */
AICP_Header AICP_ExtractHeader (uint16_t PacketStartIndex)
{
	AICP_Header AICP_Header ;

	AICP_Header.Telegram_Length_MSB = 	AICP_NewReceicedData[PacketStartIndex];
	AICP_Header.Telegram_Length_LSB = 	AICP_NewReceicedData[PacketStartIndex + 1 ];
	AICP_Header.DestinationAddress = 	AICP_NewReceicedData[PacketStartIndex + 2 ];
	AICP_Header.TelegramType = 			AICP_NewReceicedData[PacketStartIndex + 3 ];
	AICP_Header.TelegramCounter =		AICP_NewReceicedData[PacketStartIndex + 4 ];
	AICP_Header.ProtocolVersion = 		AICP_NewReceicedData[PacketStartIndex + 5 ];
	AICP_Header.Reserve1 = 				AICP_NewReceicedData[PacketStartIndex + 6 ];
	AICP_Header.Reserve2 = 				AICP_NewReceicedData[PacketStartIndex + 7 ];
	AICP_Header.CRC16_LSB = 			AICP_NewReceicedData[PacketStartIndex + 8 ];
	AICP_Header.CRC16_MSB = 			AICP_NewReceicedData[PacketStartIndex + 9 ];

	return AICP_Header;
}

