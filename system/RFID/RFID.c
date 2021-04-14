/**
 ******************************************************************************
 * @file           : RFIC.c
 * @author  		: Mohsen Asadipour
 * @version 		: V1.0
 * @date    		: 23-November-2019
 * @brief          : Receiving and processing RFID
 ******************************************************************************
 */

#include <system.h>


/**
 * Read each CAN Half Data Packet, control it to have consistent counter and put it in the local RFID Variable
 * >>>Receiving the correct data: after receiving the data would be written in the "RFID_ReceivedData" structure
 * and
 * >>>Receiving the not valid data: the "RFID_ReceivedData"  would be reseted.
 * @param CANData: received CAN Half Data packet
 * @param SecondHalfData: define if the Received Data is the first or second half packet
 */
void  RFIF_ReadRFIDCode( uint8_t CANData[] , boolean SecondHalfData )
{
	extern char fc_telegram_rfid[10];
	extern boolean agv_Segment_End_reached;
	extern boolean locationFound;

	if(!SecondHalfData)	//process first part of data
	{
		//copy data
		for(uint8_t i = 0 ; i <= 4 ; i++)
		{
			RFID_ReceivedData.Code[i] = CANData[i]; //copy to byte 0 to 4
		}

		//copy consistency counter
		ConsistencyCounter_FirstHalfPacket = CANData[5];

		//reset the receive flag cause the data is not complete
		RFID_Received = False;
	}
	else	//process second part of data
	{
		//compare the consistency counter with the first received half packet
		ConsistencyCounter_SecondHalfPacket = CANData[5];

		if(ConsistencyCounter_SecondHalfPacket == ConsistencyCounter_FirstHalfPacket)
		{
			//received data is valid
			//copy data
			for(uint8_t i = 0 ; i <= 4 ; i++)
			{
				RFID_ReceivedData.Code[i + 5] = CANData[i];	//copy to byte 5 to 9
			}
			Send_ConsistencyCounter_Value(CANDeviceID_RFID, ConsistencyCounter_SecondHalfPacket);		//sending a handshake signal

			//reset the consistency counters
			ConsistencyCounter_FirstHalfPacket = 0;
			ConsistencyCounter_SecondHalfPacket = 0;

			//rising the receive Flag
			RFID_Received = True;
			for(uint8_t i = 0 ; i < 10 ; i++)
			{
				fc_telegram_rfid[i] = RFID_ReceivedData.Code[i];
			}
			agv_Segment_End_reached = True;
			locationFound = True;
		}
		else	//on receiving not consistent data, clean the RFID DataStructure
		{
			RFID_ClearState();
		}
	}
}

/**
 * this function clear the state of RFID
 * the usage is after receiving not valid data
 */
void RFID_ClearState()
{

	//claering state variables
	RFID_Received = False;
	ConsistencyCounter_FirstHalfPacket = 0;
	ConsistencyCounter_SecondHalfPacket = 0;


	//clear the RFID Data Structure
	for(uint8_t i = 0 ; i <= 9 ; i++)
	{
		RFID_ReceivedData.Code[i] = 0;
	}
}
