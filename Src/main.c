/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "memory_map.h"
#include "string.h"
#include "math.h"

#include "system.h"

#include "SEGGER_RTT.h"
#include "SEGGER_SYSVIEW.h"
#include "SEGGER_SYSVIEW_Conf.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
 *LEDSignaSchema:
 *LEDState: could be True or False
 *BilnkDelay: is the delay to next LEDState
 */
typedef struct {
	uint8_t MaxState;
	boolean LEDState[10];
	uint32_t BlinkDelay[10];

} LEDSignalSchema;

/**
 * states of LED-Signal state machine
 */
typedef enum {
	LEDSignalState_Idle = 0, //!< Idle
	LEDSignalState_Error = 1, //!< Error
	LEDSignalState_CANRecieved = 2,
	LEDSignalState_Calibrate = 3,
	LEDSignalState_undevoltage = 4,
	LEDSignalState_CAN_Error = 5,
	LEDSignalState_No_Modules_Found = 6,
	LEDSignalState_Got_Corrupt_Data = 7,
	LEDSignalState_AccesPoint_Not_Reachable = 8,
	LEDSignalState_XBEE_NoResponse = 9,
	LEDSignalState_Maintenence_Mode = 10
} LED_States;

typedef enum {
LEDSignalPort_MCU = 0,
LEDSignalPort_Error = 1,
LEDSignalPort_Status = 2
} LED_Port;

/*
 *FC_Data Settings
 */
typedef enum {
	FC_Telegram_agv_id = 0,
	FC_Telegram_type = 1,
	FC_Telegram_counter = 2,
	FC_Telegram_version = 3,
	FC_Telegram_reserve_a = 4,
	FC_Telegram_reserve_b = 5,
	FC_Telegram_max_speed = 6,
	FC_Telegram_current_speed = 7,
	FC_Telegram_battery = 8,
	FC_Telegram_rfid = 9,
	FC_Telegram_segmentprocess = 10,
	FC_Telegram_isidle = 11
} FC_Dataset;

/*WLAN Network Setting*/
typedef struct {
	char NetworkType[50];
	char InfrastructureMode[50];
	char SSID[50];
	char EncryptionType[50];
	char Passphrase[50];
	char IPProtocol[50];
	char IPAddressingMode[50];
	char ClientConnectionTimeout[50];
	char ServerConnectionTimeout[50];
} wLAN_NetworkSetting;

/*WLAN Address Setting*/
typedef struct {
	char DNSAddress[50];
	char DestinationIPAddress[50];
	char SourcePort[50];
	char DestinationPort[50];
	char GatewayIPAddress[50];
	char IPAddressMask[50];
	char ModuleIPAddress[50];
} wLAN_AddressingSetting;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//todo: change the length of all other data buffers to use this variable. its easier to use the same variable for all buffers with length of 300
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 256 * 4
};
/* Definitions for MoveTask */
osThreadId_t MoveTaskHandle;
const osThreadAttr_t MoveTask_attributes = {
  .name = "MoveTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 1024 * 4
};
/* Definitions for LineScannerTask */
osThreadId_t LineScannerTaskHandle;
const osThreadAttr_t LineScannerTask_attributes = {
  .name = "LineScannerTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* Definitions for ObstacleDetecti */
osThreadId_t ObstacleDetectiHandle;
const osThreadAttr_t ObstacleDetecti_attributes = {
  .name = "ObstacleDetecti",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* Definitions for CommunicationTa */
osThreadId_t CommunicationTaHandle;
const osThreadAttr_t CommunicationTa_attributes = {
  .name = "CommunicationTa",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* Definitions for AICPTask */
osThreadId_t AICPTaskHandle;
const osThreadAttr_t AICPTask_attributes = {
  .name = "AICPTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* Definitions for TimerTask */
osThreadId_t TimerTaskHandle;
const osThreadAttr_t TimerTask_attributes = {
  .name = "TimerTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* Definitions for TestingTask */
osThreadId_t TestingTaskHandle;
const osThreadAttr_t TestingTask_attributes = {
  .name = "TestingTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* Definitions for LEDTask */
osThreadId_t LEDTaskHandle;
const osThreadAttr_t LEDTask_attributes = {
  .name = "LEDTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* Definitions for Timer_Linescanner */
osTimerId_t Timer_LinescannerHandle;
const osTimerAttr_t Timer_Linescanner_attributes = {
  .name = "Timer_Linescanner"
};
/* Definitions for Timer_Motormodule */
osTimerId_t Timer_MotormoduleHandle;
const osTimerAttr_t Timer_Motormodule_attributes = {
  .name = "Timer_Motormodule"
};
/* USER CODE BEGIN PV */
osThreadId_t ObstacleDetectionTaskHandle;
osThreadId_t CommunicationTaskHandle;
osThreadId_t AICPTaskHandle;
osThreadId_t TimerTaskHandle;
osThreadId_t TestingTaskHandle;

//Application Variables
//-----line following
boolean DebugMode = True;
uint8_t mLine = 0;
int8_t AGVDeviation = 0;
uint8_t AGVDeviationDegree = 0;
float AGVDeviationCoefficient = 0;
float DeviationCoefficient[25] = { 0.0, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12,	0.14, 0.16, 0.20, 0.22, 0.26, 0.28, 0.30, 0.32, 0.34, 0.36, 0.38, 0.41,
									0.42, 0.44, 0.46, 0.48, 0.50, 0.52 };

/*
 * works good for slow speed and direct lines {0.0, 0.015 , 0.03, 0.045, 0.06 , 0.075 , 0.09 , 0.105, 0.12 , 0.135 , 0.15};
 * not working: to shasp changes on the outer areas and cause oscilating behavior {0.0, 0.015 , 0.03, 0.045, 0.06 , 0.075 , 0.09 , 0.105, 0.12 , 0.135 , 0.15};
 * */
boolean LineScanner_LostLine = True;
uint8_t temp_1 = 0;
int8_t temp_2 = 0;
int8_t derivative_1_array[20];
int8_t Oscillator_count = 0;
int8_t ActualDerivative_1[20] = { 0 };
float CheckDrvArray[8];
int RightCurvevaluecheck = 0;
int LeftCurvevaluecheck = 0;
boolean CurveDetected;
boolean Curve_present;
boolean pt_eight_curve;
boolean RightCurve;
boolean LeftCurve;
int straightlineCounter = 0;
int mCurve = 0;
boolean OnStraightLine;
const uint8_t CurveDeviationCoefficient_MinIndex = 0;
const uint8_t CurveDeviationCoefficient_MaxIndex = 25;
const uint8_t InnerMotorDeviationCoefficient_MinIndex = 0;
const uint8_t InnerMotorDeviationCoefficient_MaxIndex = 34;
const uint8_t OuterMotorDeviationCoefficient_MinIndex = 0;
const uint8_t OuterMotorDeviationCoefficient_MaxIndex = 34;
const uint8_t pt_eight_InnerMotorDeviationCoefficient_MinIndex = 0;
const uint8_t pt_eight_InnerMotorDeviationCoefficient_MaxIndex = 34;
const uint8_t pt_eight_OuterMotorDeviationCoefficient_MinIndex = 0;
const uint8_t pt_eight_OuterMotorDeviationCoefficient_MaxIndex = 32;
float InnerCoefficient = 0;
float OuterCoefficient = 0;
uint8_t MightBeOscillating = 0;
int8_t max;
int8_t min;
int tempState[2] = { 0 };
int8_t derArray_temp[2] = { 0 };
int tempState_1 = 0;
int tempState_2 = 0;

const uint8_t DeviationCoefficient_MinIndex = 0;
const uint8_t DeviationCoefficient_MaxIndex = 20;
const int8_t cLineScanner_SensorCenter = 64;
const int8_t cLineScanner_LowestPixel = 0;
const int8_t cLineScanner_HighstPixel = 127;

//LED Signal
uint8_t LEDErrorCounter = 0;
uint8_t LEDErrorQueue[6];	//stores all Module errors
uint8_t LED_State = 0; 		//the state of LED Task
uint8_t LED_Port_State;
uint8_t LED_NextState = 0;		//next state of LED Task
boolean LEDSchema_Done = False;
uint32_t LEDSchema_DelayCounter = 0; 	//the counter for each schema-delay
uint8_t LEDSchema_Index = 0; 	//the current index of excution in schema
LEDSignalSchema LEDSignal_SchemaToExcute; //this schema would be excuted by ExcuteSchema()
uint8_t ExcuteSchema_State = 0; //the state of ExcuteSchema State machine
LEDSignalSchema LEDSignalSchema_Idle = { 2, { 1, 0 }, { 20, 20 } };
LEDSignalSchema LEDSignalSchema_CANReceived = { 4, { 1, 0, 1, 0 }, { 1, 1, 1, 1 } };
LEDSignalSchema LEDSignalSchema_Error = { 10, { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, }, { 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, } };
LEDSignalSchema LEDSignalSchema_Calibrate =	{ 4, { 1, 0, 1, 0 }, { 2, 2, 2, 2 } };
LEDSignalSchema LEDSignalSchema_undevoltage = { 8, { 1, 0, 1, 0, 1, 0, 1, 0 }, { 10, 10, 10, 10, 10, 10, 10, 100 } };
LEDSignalSchema LEDSignalSchema_CAN_Error = { 8, { 1, 0, 1, 0, 1, 0, 1, 0 }, { 2,	10, 50, 10, 50, 10, 50, 100 } };
LEDSignalSchema LEDSignalSchema_No_Modules_Found = { 8, { 1, 0, 1, 0, 1, 0, 1, 0 }, { 10, 10, 50, 10, 10, 10, 10, 100 } };
LEDSignalSchema LEDSignalSchema_Got_Corrupt_Data = { 8, { 1, 0, 1, 0, 1, 0, 1, 0 }, { 10, 10, 10, 10, 50, 10, 50, 100 } };
LEDSignalSchema LEDSignalSchema_AccesPoint_Not_Reachable = { 8, { 1, 0, 1, 0, 1, 0, 1, 0 }, { 50, 10, 10, 10, 10, 10, 50, 100 } };
LEDSignalSchema LEDSignalSchema_XBEE_NoResponse = { 8, { 1, 0, 1, 0, 1, 0, 1, 0 }, { 10, 10, 50, 10, 50, 10, 10, 100 } };
LEDSignalSchema LEDSignalSchema_Maintenence_Mode = { 8, { 1, 0, 1, 0, 1, 0, 1, 0 }, { 10, 10, 10, 10, 10, 10, 10, 10 } };

//----Motors Driver
float DesiredSpeed = 0.15;
uint32_t MotorLeft_Diversion = 0;
uint32_t MotorRight_Diversion = 0;
float MotorLeft_PIDSignal = 0;
float MotorRight_PIDSignal = 0;
float MotorLeft_PIDSignal_Scaled = 0;
float MotorRight_PIDSignal_Scaled = 0;
float MotorLeft_Speed = 0;
float MotorRight_Speed = 0;
float speed = 0;
boolean MotorDriver_Start = False;
boolean MoveStartCondition = False;
const float cMotorMaxSpeedMPS = 2.3;
const float cMotorMinSpeedMPS = 0;
const float cMotorStopSpeed = 0;
float RightMotorDistance = 0;
float LeftMotorDistance = 0;
uint8_t motorConsistencyCounter = 0;
uint8_t timeoutCheckerLeft = 0;
uint8_t timeoutCheckerRight = 0;
uint32_t RightEncoderCounts = 0;
uint32_t LeftEncoderCounts = 0;
uint32_t RightEncoderReset= 0;
uint32_t LeftEncoderReset = 0;
uint32_t RightTimeNow;
uint32_t RightTimeReset = 0;
uint32_t LeftTimeReset = 0   ;
uint32_t LeftTimeNow;
const float EncoderResolution = 450.0;
float RightEncoderResolution = 450.0 ;
float LeftEncoderResolution = 450.0 ;
float DegreePerPulse = 0;
float DistanceMovedPerPulse = 0;
const float WheelRadius = 0.0625; //radius in meters
float RightTempDistance= 0 ;
float LeftTempDistance = 0;
float DistanceDiff = 0;
const uint16_t DistanceBetweenMotors = 3750; // in mm
float VectorAngle = 0;
const uint8_t TICK_Arr = 0;
const uint8_t Dist_Arr =1 ;
const uint8_t SegmentProgessData = 3;
const uint8_t Time_Arr = 2;
float RightMotorArray [4][3] = {0}; // 4_ROWS>>EncoderCounts,Distance,Velocity,Angle ; 3_Columns>>Store previous values
float LeftMotorArray [4][3] = {0}; // 4_ROWS>>EncoderCounts,Distance,Velocity,Angle ; 3_Columns>>Store previous values
boolean ResetLeftSegmentProgessData = True;
boolean ResetRightSegmentProgessData = True;
uint32_t LeftSegmentProgessCount = 0;
uint32_t RightSegmentProgessCount = 0;
boolean LeftAtRest;
boolean RightAtRest;
boolean ResetEncoderRight = True;
boolean ResetEncoderLeft = True;
boolean Move_S6Called = True;
float TargetEncoderCounter = 0;
float NumberOfRevolution = 0;
boolean PackageDelay = True;
float DistanceMovedByRobot = 0;
uint16_t EncoderDataRecieveFrequency = 73;
//------TEMP Var MotorDriver
float LineScanDeviationAngle = 0;
float DiffInLastMotorReading= 0.0;
//boolean UseDist = True;
float MappedAGVDeviation[2] = {0};
uint32_t Move_S6CalledTime = 0;
float TempSpeed[2] = {0};
float TempSpeed_1 = 0.2;
uint32_t  RightMotorLiveCheck =0;
uint32_t LeftMotorLiveCheck = 0;
uint32_t LightControllerLiveCheck = 0;
boolean RightMotorLive =False;
boolean LeftMotorLive = False;

//----Obstacle detection
uint32_t ObstacleDetection_ResumeDelay_CounterLimit = 1000;
boolean ObstacleDetection_ResumeDelay_Output = False;
const uint32_t LiveSignal_CounterLimit = 100; // *10ms = Time Period
//----UART
char EndDelimiter = '\n';
char EndDelimiter1 = '\r';

//----WLAN
wLAN_NetworkSetting WLAN_NetworkSetting = { "ATAH2\r", "ATCE2\r", "ATIDSaphir\r",
											"ATEE2\r", "ATPKAlg&/18399113950478\r",
											"ATIP1\r", "ATMA1\r", "ATTM67\r", "ATTS258\r" };
wLAN_AddressingSetting WLAN_AddressingSetting = { "ATNS192.168.178.1\r", "ATDL192.168.178.63\r",
												"ATC02616\r", "ATDE2616\r", "ATGW192.168.178.1\r",
												"ATMK255.255.255.0\r", "ATMY192.168.178.50\r" };
//----timer
/*
 * timer usage table
 * timer 0 : WLAN communication start+
 * timer 1 & 2 WLAN
 * timer 3 LineScanner
 * timer 4 obstacle detection.resume delay
 * timer 5 RightMotorModule Check
 * timer 6 LeftMotorModuleCheck
 * */

boolean TON_Start[Timer_Max];
boolean TON_Out[Timer_Max];
boolean mTON_Start[Timer_Max];
boolean TON_StartCounter[Timer_Max];
uint32_t TON_Counter[Timer_Max];
uint32_t TON_PT[Timer_Max];
const uint32_t TON_PT_ms[Timer_Max] = { 5000, 500, 30, 2000, 3000, 1500,1500 };
const uint32_t Timer_Coefficient = 1;

//General variables
uint16_t holdingReg[TOTAL_REG_COUNT];
boolean ButtomPushed = False;
boolean IsFirstCycle = True; //this variable is true just on the first cycle. in the cycleEnd it would be written to False
uint32_t timingTimeout;
uint32_t TimingDelay;
uint8_t segmentIdIndex = 0;
uint32_t Current_Ticks = 0;
//CAN
uint8_t CANData[7];
uint8_t FunctionCode = 0;
uint8_t FunctionCode1ByteValue = 0;
uint16_t FunctionCode2ByteValue = 0;
boolean FunctionCodeFlag = False;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8]; //toDo : replace MotorMovementData with this variable
uint8_t RxData[8];
uint32_t TxMailbox;
boolean rxFlagCAN = False;
uint8_t MotorMovementData[7];
uint8_t LightControllerSendData[7] = {0};
HAL_StatusTypeDef ReturnStatus;
uint32_t lineCounter = 0;
uint32_t SegCountertest = 0;

//LineFollower
uint8_t Line1_Position = 0;
lines Lines;
uint8_t LineCenter = 64;
int8_t LineCenterDiff = 0;
int8_t mLineCenterDiff = 0;
int8_t LineCenterDiff_Derivative = 0;
int8_t LineLimits = 50;
boolean InGreenWindow = False;
int8_t Derivative_Max = 0;
boolean MirrorLines = False; /*to mirror all lines if the line scanner sensor is mounted with 180 degree*/
boolean error_LineScan_CAN  = False;
uint32_t LineScanLiveCheck = 0;


//PID Controller
PIDParameters PIDParams_Dir = { 1.05, 0.1, 0.6, 0, 1.00 }; //PID Parameters Kp, Ki, Kd, Imax, Imin
PIDParameters PIDParams_Motor = { 1.2, 0.0, 1.0, 0, 0.00 }; //PID Parameters Kp, Ki, Kd, Imax, Imin
PIDState PIDStateData_Dir = { 0, 0, 0 };
PIDState PIDStateData_Motor = { 0, 0, 0 };
double PID_Output = 0;

//Errorflags for LED Errorcodes
boolean error_undervolt = False;
boolean error_CAN_error = False;
boolean error_ModulesNotFound = False;
boolean error_CorruptData = False;
boolean error_NoAccesPoint = False;
boolean error_XBee_NoResponse = False;
boolean error_Motordriver_Left = False;
boolean error_Motordriver_Right = False;
boolean error_LightController = False;

//Obstacle detection
boolean ObstacleDetection_SensorState[5];
boolean ObstacleDetection_SensorActive[5] = { True, True, True, True, True };
boolean ObstacleDetection_Detected = False;
boolean ObstacleDetection_Detected_WithDelay = False;
boolean mObstacleDetected = False;
uint32_t ObstacleDetection_ResumeDelay_Counter = 0;
boolean error_ObstacleDetectionModule   = False;
uint32_t ObstacleDetectionLiveCheck  = 0;

//LightController
uint8_t LightControllerCounter = 0;
uint8_t LightControllerTxData[7]= {0};
const uint8_t MainModule_ON = 0xEE;
const uint8_t MotorDriver_OFF = 0xF0;
const uint8_t Movement = 0xF1;
const uint8_t Obstacle_Present = 0xF2;
const uint8_t Wifi_Problem = 0xF3;
const uint8_t Battery_Status = 0xF4;
const uint8_t Maintainance_ON = 0xF5;
const uint8_t Turning_Right = 2;
const uint8_t Turning_Left = 1;
const uint8_t Moving_Straight = 0;
//const uint8_t Idle = 2;
boolean CorrectPacket = False;
uint16_t CAN_TxRate = 0;
uint8_t LC_Recieved_Ack=0;

int a = 0;

//WLAN
char rxBuffer[300];
const uint16_t rxBufferSize = 300;
uint16_t rxPosition = 0;
boolean UART_NewDataToInterprate;
char response[10];
uint8_t UART_ReceicedByte[1];
char UART_ReceicedData[300];
uint16_t UART_ReceicedData_Size = 300;
boolean UART_RecFlag = False;
boolean commandModeFlag = False;
int wlan_state = 0;
int WLAN_State_InitNetworkSetting = 0;
int WLAN_State_InitAddressSetting = 0;
int WLAN_State_Start = 0;
int WLAN_State_End = 0;
boolean XBee_NetworkSetting_Initialized;
boolean WLAN_DHCPIsActive;
boolean XBee_AddressingSetting_Initialized;
boolean CMD_Start;
boolean Flag_SendLiveSignal;
uint32_t Counter_LiveSignal;
boolean TC_Is_Rdy_For_Transmission = False;

// telegram Send
char fc_telegram_agv_id[] = "TCS00001";
char fc_telegram_type[] = "D";
char fc_telegram_counter[] = "00";
char fc_telegram_version[] = "01";
char fc_telegram_reserve_a[] = "00";
char fc_telegram_reserve_b[] = "00";
char fc_telegram_max_speed[] = "0000";
char fc_telegram_current_speed[] = "0000";
char fc_telegram_battery[] = "0000";
char fc_telegram_rfid[] = "0000000000";
char fc_telegram_segmentprocess[] = "0000";
char fc_telegram_isidle[] = "1";
uint8_t telegramCounter = 0;
char fc_sendData[] = "#TCS00001D000100010500020550006100A8833F00001[";
uint8_t fc_sendDataPosition = 0 ;

// telegram Receive
char fc_telegram_destination_rfid[] = "0000000000";
char fc_Current_Segment[] = "0000000000";
boolean agv_destination_reached = False;
boolean agv_Segment_End_reached = False;
char currentSegmentLength[6];
boolean TC_SignalIsFresh = False;
uint32_t TC_SignalIsFreshCheck = 0;

//AICP
AICP_Header Received_AICP_Header;
uint8_t AICP_State;
char AICP_NewReceicedData[Size_DataBuffer];
AICP_Header AICPReceivedHeader[10];	//all received headers would be saved heir

//RFID Reader
RFID_Code RFID_ReceivedData;
uint8_t ConsistencyCounter_FirstHalfPacket;
uint8_t ConsistencyCounter_SecondHalfPacket;
boolean RFID_Received;
char TC_CurrentRFID[10];
uint8_t dataRFIDResponse[2];
boolean locationFound = False;
boolean startPositionReached = False;

//FleetController (FC)
char FC_Order[50];
uint8_t *FC_pBuffer;
boolean maintenanceMode = False;

//Debuging
char LogText[200];
boolean DebugEnable = False;

//Test
boolean t5_Start = False;
boolean t5_Direction = False;
boolean t13_Start = False;
boolean t_Start = False;
float tSpeed_L = 0.1;
float tSpeed_R = 0.1;
boolean tDirection_L = True;
boolean tDirection_R = True;
boolean tRecievedRFID;
boolean tMotorStatusFlag = False;
uint32_t timetester=0;

//LED
void ExcuteSchema();
void LEDState(boolean LEDState);
void ErrorLEDState(boolean LEDState);
void fillingErrorQueue();

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void movetask_2_Main(void *argument);
void LineScannerTask_Main(void *argument);
void ObstacleDetectionTask_Main(void *argument);
void CommunicationTask_Main(void *argument);
void AICPTask_Main(void *argument);
void TimerTask_Main(void *argument);
void TestingTask_Main(void *argument);
void LEDTask_Main(void *argument);
void timeoutLinescanner(void *argument);
void timeoutMotormodule(void *argument);
// boolean RightMotorModuleCheck ();
// boolean LeftMotorModuleCheck();
/* USER CODE BEGIN PFP */

/***CAN***/
void CAN_Transmit(CAN_TxHeaderTypeDef *TxHeader, uint8_t TxData[]);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void RunReceivedFunctionCodes(uint8_t Data);
void CAN_SeparateDataAndFunctionCode(uint8_t RxData[], uint32_t DLC, uint8_t *FunctionCode, uint8_t *CANData);
void SendData(uint32_t CAN_ID, uint8_t Data);
 /***FleetControl***/
void FC_Controller();
short FC_GetCRC(uint8_t data[], int start, int length);

/***WLAN***/
int XBeeInit_NetworkSetting(wLAN_NetworkSetting *NetworkSetting);
int XBeeInit_AddressingSetting(wLAN_AddressingSetting *AddressingSetting);
int XBeeCommandSend(char *cmd, char *res, int timeout);
int XBee_AT_Start(void);
int XBee_AT_SaveAndExit(void);
void FC_Order_Send();
void FC_FillDataset(char data[]);
void FC_Transmission_Out(FC_Dataset dataset, boolean sendData);
void XBee_SendLiveSignal();
void UART_SendByte(char data);
void UART_WriteTextLine(uint8_t *pBuffer);
void UART_CopyDataToLocalBuffer_1();
void UART_ResetRxBuffer();
void UART_InterpretReceivedData();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);

/***AICP***/
void AICPTask_Main();

/***Testing***/
void Test001_UART_SendCommand();
void Test002_CAN_SendingTest();
void Test003_Communication_Uart();
void Test004_Communication_ATStart();
void Test005_AICP_CheckCRC();
void Test006_IBN1_MoveWithObstacleDetection();
void Test007_DataSendToRFIDModul();
void Test008_GetDistance();
void Test009_DataSendToObstacleModul();
void Send_DataToTc ();

/***Utility***/
boolean PushButtom_GetState();
//void Debug(char * pBuffer);	//debug function would not work in the controller cause the uart 2 is not used
void FirstCycle();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	/*SEGGER Change*/
	/*Enable ARM Cortex Cycle Counter to get the time stamp of each event*/
	DWT->CTRL |= (1 << 0); //Enable CYCCNT in the DWT_CTRL
	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();
	error_ModulesNotFound = True;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	for (int i = 0; i < 10; i++) 	//RDID Data set to 0
			{
		RFID_ReceivedData.Code[i] = '0';
	}
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  if (!HAL_GPIO_ReadPin(USER_BT_GPIO_Port, USER_BT_Pin))
  {
	  maintenanceMode = True;

	  while(!HAL_GPIO_ReadPin(USER_BT_GPIO_Port, USER_BT_Pin))
	  {

	  }
  }
  else
  {
	  maintenanceMode = False;
  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin,GPIO_PIN_SET);

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of Timer_Linescanner */
  Timer_LinescannerHandle = osTimerNew(timeoutLinescanner, osTimerPeriodic, NULL, &Timer_Linescanner_attributes);

  /* creation of Timer_Motormodule */
  Timer_MotormoduleHandle = osTimerNew(timeoutMotormodule, osTimerPeriodic, NULL, &Timer_Motormodule_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MoveTask */
  MoveTaskHandle = osThreadNew(movetask_2_Main, NULL, &MoveTask_attributes);

  /* creation of LineScannerTask */
  LineScannerTaskHandle = osThreadNew(LineScannerTask_Main, NULL, &LineScannerTask_attributes);

  /* creation of ObstacleDetecti */
  ObstacleDetectiHandle = osThreadNew(ObstacleDetectionTask_Main, NULL, &ObstacleDetecti_attributes);

  /* creation of CommunicationTa */
  CommunicationTaHandle = osThreadNew(CommunicationTask_Main, NULL, &CommunicationTa_attributes);

  /* creation of AICPTask */
  AICPTaskHandle = osThreadNew(AICPTask_Main, NULL, &AICPTask_attributes);

  /* creation of TimerTask */
  TimerTaskHandle = osThreadNew(TimerTask_Main, NULL, &TimerTask_attributes);

  /* creation of TestingTask */
  TestingTaskHandle = osThreadNew(TestingTask_Main, NULL, &TestingTask_attributes);

  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew(LEDTask_Main, NULL, &LEDTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */

  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
	CAN_FilterTypeDef sFilterConfig;
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

	/* CAN Baudrate = 125kBps (CAN clocked at 45 MHz)*/
  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 24;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

	// Configure the CAN Filter
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
	}

	// Start the CAN peripheral
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		/* Start Error */
		Error_Handler();
	}

	// Activate CAN RX notification
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		/* Notification Error */
		Error_Handler();
	}

	/* CAN Set EN and RS */
	HAL_GPIO_WritePin(CAN_RS_GPIO_Port, CAN_RS_Pin, False);
	HAL_GPIO_WritePin(CAN_EN_GPIO_Port, CAN_EN_Pin, True);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_MCU_Pin|LED_ERROR_Pin|LED_STATUS_Pin|LED_AICP_RX_Pin
                          |LED_AICP_TX_Pin|CAN_RS_Pin|CAN_EN_Pin|RS485_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(XBEE_RESET_GPIO_Port, XBEE_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BT_Pin */
  GPIO_InitStruct.Pin = USER_BT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_MCU_Pin LED_ERROR_Pin LED_STATUS_Pin LED_AICP_RX_Pin
                           LED_AICP_TX_Pin CAN_RS_Pin CAN_EN_Pin RS485_EN_Pin */
  GPIO_InitStruct.Pin = LED_MCU_Pin|LED_ERROR_Pin|LED_STATUS_Pin|LED_AICP_RX_Pin
                          |LED_AICP_TX_Pin|CAN_RS_Pin|CAN_EN_Pin|RS485_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : R_SW1_Pin R_SW_2_Pin R_SW_3_Pin R_SW_4_Pin */
  GPIO_InitStruct.Pin = R_SW1_Pin|R_SW_2_Pin|R_SW_3_Pin|R_SW_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : XBEE_RESET_Pin */
  GPIO_InitStruct.Pin = XBEE_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(XBEE_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**************CAN Communication**************************/
/**
 * Process the received CAN Messages
 * @param Data
 */

void RunReceivedFunctionCodes(uint8_t Data) {
	switch (Data) {
	case (0x01): //Reading the Linesfollower Sensor
		LineScanner_ReadLines(CANData, MirrorLines, &Lines);
		HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
//		error_LineScan_CAN  = False;
		LineScanLiveCheck = HAL_GetTick();
		error_CorruptData = False;
		break;
	case (0x02):	//Obstacle Detection
		ObstacleDetection_GetSensorsState(CANData);
		HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
		ObstacleDetectionLiveCheck =  HAL_GetTick();
		error_CorruptData = False;
		break;
	case (0x10):	//RFID First Half
		RFIF_ReadRFIDCode(CANData, False);
		HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
		error_CorruptData = False;
		break;
	case (0x11):	//RFID Second Half
		RFIF_ReadRFIDCode(CANData, True);
		HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
		error_CorruptData = False;
		break;
	case (0x20): //Distance form right  motor
		Motor_ReadRightMotorDistance(CANData);
		HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
//		RightMotorLive = True;
		RightMotorLiveCheck =  HAL_GetTick();
		error_CorruptData = False;
		break;
	case (0x21): //Distance form Left motor
		Motor_ReadLeftMotorDistance(CANData);
		HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
//		LeftMotorLive = True;
		LeftMotorLiveCheck =  HAL_GetTick();
		error_CorruptData = False;
		break;
	case (0x30):	//LightController
		//LightController_Ack_SignalCheck(CANData);
		HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
		a = a+1;
		LC_Recieved_Ack = CANData[0];

		LightControllerLiveCheck =  HAL_GetTick();
		error_CorruptData = False;
		break;
	case (0xFF): //Receive Errorcodes from other Modules
		switch (CANData[0]) {
		case 2:
			fc_telegram_reserve_b[1] = '2'; // LineScaner Kamera not found
			break;
		case 3: 							//
			break;
		}
		break;
	default:	//Undefinited Signal
		error_CorruptData = True;
		break;


	}
}

/**
 * sends data over CAN
 */
void CAN_Transmit(CAN_TxHeaderTypeDef *TxHeader, uint8_t TxData[]) {

	ReturnStatus = HAL_CAN_AddTxMessage(&hcan1, TxHeader, TxData, &TxMailbox);
	if (ReturnStatus != HAL_OK) {
		Error_Handler();
		//Debug(&ReturnStatus);
	}
}

/**
 * @brief  Rx Fifo 0 message pending callback
 * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	error_ModulesNotFound = False;
	/* Get RX message */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
		/* Reception Error */
		error_CAN_error = True;
		Error_Handler();
	}
	/* Save RX message */
	else if ((RxHeader.StdId == CANDeviceID_ThisModule)
			&& (RxHeader.IDE == CAN_ID_STD)) {

		error_CAN_error = False;
		CAN_SeparateDataAndFunctionCode(RxData, RxHeader.DLC, &FunctionCode, CANData);

		RunReceivedFunctionCodes(FunctionCode);

		rxFlagCAN = True;
	}

}

/**
 *
 * @param RxData: Received Data Packet over CAN
 * @param DLC	: the length of CAN received Data
 * @param FunctionCode : extracted function code from CAN Data Packet . FucntionCode is the first byte of CAN Data packet
 * @param CANData : the Data correspond to the extracted funciton code from CAN Data Packet. the Data is the rest of Received Data over CAN
 */
void CAN_SeparateDataAndFunctionCode(uint8_t RxData[], uint32_t DLC, uint8_t *FunctionCode, uint8_t *CANData)
{
	if (DLC == 1)
	{
		*FunctionCode = RxData[0];
	}
	else if (DLC == 2)
	{
		*FunctionCode = RxData[0];
		CANData[0] = RxData[1];
	}
	else if (DLC == 3)
	{
		*FunctionCode = RxData[0];
		CANData[0] = RxData[1];
		CANData[1] = RxData[2];
	}
	else if (DLC == 4)
	{
		*FunctionCode = RxData[0];
		CANData[0] = RxData[1];
		CANData[1] = RxData[2];
		CANData[2] = RxData[3];
	}
	else if (DLC == 5)
	{
		*FunctionCode = RxData[0];
		CANData[0] = RxData[1];
		CANData[1] = RxData[2];
		CANData[2] = RxData[3];
		CANData[3] = RxData[4];
	}
	else if (DLC == 6)
	{
		*FunctionCode = RxData[0];
		CANData[0] = RxData[1];
		CANData[1] = RxData[2];
		CANData[2] = RxData[3];
		CANData[3] = RxData[4];
		CANData[4] = RxData[5];
	}
	else if (DLC == 7)
	{
		*FunctionCode = RxData[0];
		CANData[0] = RxData[1];
		CANData[1] = RxData[2];
		CANData[2] = RxData[3];
		CANData[3] = RxData[4];
		CANData[4] = RxData[5];
		CANData[5] = RxData[6];
	}
	else if (DLC == 8)
	{
		*FunctionCode = RxData[0];
		CANData[0] = RxData[1];
		CANData[1] = RxData[2];
		CANData[2] = RxData[3];
		CANData[3] = RxData[4];
		CANData[4] = RxData[5];
		CANData[5] = RxData[6];
		CANData[6] = RxData[7];
	}
}

void SendData(uint32_t CAN_ID, uint8_t Data) {	//Line follow module
	LightControllerSendData[0] = LightControllerTxData[0];		//function Code
	LightControllerSendData[1] = LightControllerTxData[1];
	LightControllerSendData[2] = LightControllerTxData[2];		//Acknowledge Data
	//making the header
	TxHeader.DLC = 3;		//setting data length
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = CAN_ID;	//setting the device can id
	CAN_Transmit(&TxHeader, LightControllerSendData);	//sending the can packet
}


void Send_ConsistencyCounter_Value(uint32_t CAN_ID, uint8_t ConsistencyValue) {	//Line follow module
	dataRFIDResponse[0] = 0xEE;		//function Code
	dataRFIDResponse[1] = ConsistencyValue;		//Response Value

	//making the header
	TxHeader.DLC = 2;		//setting data length
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = CAN_ID;	//setting the device can id

	CAN_Transmit(&TxHeader, dataRFIDResponse);	//sending the can packet
}
/**************WLAN**************************/
/*
 * toDo:
 * initiate the RF Module throw AT Commands
 */

/*XBee Communication Functions*/
/**
 *initialize the communication module.
 *network settings like SSID , IP, Gateway would be transmited to the module.
 */
int XBeeInit_NetworkSetting(wLAN_NetworkSetting *NetworkSetting) {
	switch (WLAN_State_InitNetworkSetting) {
	case (0): /*initiate the state machine. -reset state machine variables */
		WLAN_State_Start = 0;

		WLAN_State_InitNetworkSetting = 100;
		break;

	case (100): /*puting the module in at mode*/
		TON_Start[0] = True; /*start XBee_AT_Start() timeout timer*/

		if (XBee_AT_Start() == 1)
			WLAN_State_InitNetworkSetting = 1;
		else
			WLAN_State_InitNetworkSetting = 900;

		TON_Start[0] = False; /*stop XBee_AT_Start() timeout timer*/

		break;

	case (1): /*starting in AT Mode*/
		XBeeCommandSend("AT\r", response, 500);
		osDelay(200);
		if (strcmp(response, "OK\r") == 0) {
			WLAN_State_InitNetworkSetting = 2;
		} else if (strcmp(response, "ERROR\r") == 0)
			WLAN_State_InitNetworkSetting = 900;
		break;

		/***********Network configuration***********/
	case (2): /*setting the NetworkType*/
		XBeeCommandSend(NetworkSetting->NetworkType, response, 500);
		osDelay(200);
		if (strcmp(response, "OK\r") == 0) {
			WLAN_State_InitNetworkSetting = 3;
		} else if (strcmp(response, "ERROR\r") == 0)
			WLAN_State_InitNetworkSetting = 900;
		break;

	case (3): /*setting the Infrastructure Mode*/
		XBeeCommandSend(NetworkSetting->InfrastructureMode, response, 500);
		osDelay(200);
		if (strcmp(response, "OK\r") == 0) {
			WLAN_State_InitNetworkSetting = 4;
		} else if (strcmp(response, "ERROR\r") == 0)
			WLAN_State_InitNetworkSetting = 900;
		break;

	case (4): /*SSID*/
		XBeeCommandSend(NetworkSetting->SSID, response, 500);
		osDelay(200);
		if (strcmp(response, "OK\r") == 0) {
			WLAN_State_InitNetworkSetting = 5;
		} else if (strcmp(response, "ERROR\r") == 0)
			WLAN_State_InitNetworkSetting = 900;
		break;

	case (5): /*Encryption type*/
		XBeeCommandSend(NetworkSetting->EncryptionType, response, 500);
		osDelay(200);
		if (strcmp(response, "OK\r") == 0) {
			WLAN_State_InitNetworkSetting = 6;
		} else if (strcmp(response, "ERROR\r") == 0)
			WLAN_State_InitNetworkSetting = 900;
		break;

	case (6): /*passphrase*/
		XBeeCommandSend(NetworkSetting->Passphrase, response, 500);
		osDelay(200);
		if (strcmp(response, "OK\r") == 0) {
			WLAN_State_InitNetworkSetting = 7;
		} else if (strcmp(response, "ERROR\r") == 0)
			WLAN_State_InitNetworkSetting = 900;
		break;

	case (7): /*IP Protocol*/
		XBeeCommandSend(NetworkSetting->IPProtocol, response, 500);
		osDelay(200);
		if (strcmp(response, "OK\r") == 0) {
			WLAN_State_InitNetworkSetting = 8;
		} else if (strcmp(response, "ERROR\r") == 0)
			WLAN_State_InitNetworkSetting = 900;
		break;

	case (8): /*IP Addressing Mode*/
		XBeeCommandSend(NetworkSetting->IPAddressingMode, response, 500);
		osDelay(200);
		if (strcmp(NetworkSetting->IPAddressingMode, "ATMA0") == 0) /*if DHCP is active , in addressing settings some steps should be postponed*/
			WLAN_DHCPIsActive = True;

		if (strcmp(response, "OK\r") == 0) {
			WLAN_State_InitNetworkSetting = 9;
		} else if (strcmp(response, "ERROR\r") == 0)
			WLAN_State_InitNetworkSetting = 900;
		error_NoAccesPoint = True;
		break;

	case (9): /*TCP Client Connection Timeout*/
		XBeeCommandSend(NetworkSetting->ClientConnectionTimeout, response, 500);
		osDelay(200);
		if (strcmp(response, "OK\r") == 0) {
			WLAN_State_InitNetworkSetting = 10;
		} else if (strcmp(response, "ERROR\r") == 0)
			WLAN_State_InitNetworkSetting = 900;
		break;

	case (10): /*TCP Server Connection Timeout*/
		XBeeCommandSend(NetworkSetting->ServerConnectionTimeout, response, 500);
		osDelay(200);
		if (strcmp(response, "OK\r") == 0) {
			WLAN_State_InitNetworkSetting = 1000;
		} else if (strcmp(response, "ERROR\r") == 0)
			WLAN_State_InitNetworkSetting = 900;
		break;

	case (900): /*Error state*/
		commandModeFlag = False;
		WLAN_State_InitNetworkSetting = 0;
		return -1;
		break;

	case (1000): /*Successfully finished*/

		/*save and exit*/
		TON_Start[1] = True; /*start XBee_AT_Start() timeout timer*/

		WLAN_State_End = 0;

		if (XBee_AT_SaveAndExit() == 1)
			WLAN_State_InitNetworkSetting = 0;
		else {
			WLAN_State_InitNetworkSetting = 900;
			break;
		}

		TON_Start[1] = False; /*stop XBee_AT_Start() timeout timer*/

		return 1;
		break;

	default:
		break;
	}
}

/**
 * initialize the addressing of module
 * @param AddressingSetting
 * @return
 */
int XBeeInit_AddressingSetting(wLAN_AddressingSetting *AddressingSetting) {
	int repeat = 0;
	switch (WLAN_State_InitAddressSetting) {
	case (0): /*initiate the state machine. -reset state machine variables */
		WLAN_State_Start = 0;

		WLAN_State_InitAddressSetting = 1;
		break;

	case (1): /*puting the module in at mode*/
		TON_Start[0] = True; /*start XBee_AT_Start() timeout timer*/

		if (XBee_AT_Start() == 1)
			WLAN_State_InitAddressSetting = 2;
		else
			WLAN_State_InitAddressSetting = 900;

		TON_Start[0] = False; /*stop XBee_AT_Start() timeout timer*/

		break;

	case (2): /*starting in AT Mode*/
		XBeeCommandSend("AT\r", response, 500);
		osDelay(200);
		repeat++;

		if (strcmp(response, "OK\r") == 0) {
			WLAN_State_InitAddressSetting = 10;
			repeat = 0;
		} else if (strcmp(response, "ERROR\r") == 0)
			WLAN_State_InitAddressSetting = 900;
		break;

		/***********Addressing setting***********/
	case (10): /*DNSAddress*/
		XBeeCommandSend(AddressingSetting->DNSAddress, response, 500);
		osDelay(200);
		if (strcmp(response, "OK\r") == 0) {
			WLAN_State_InitAddressSetting = 20;
		} else if (strcmp(response, "ERROR\r") == 0)
			WLAN_State_InitAddressSetting = 900;
		break;

	case (20): /*DestinationIPAddress*/
		XBeeCommandSend(AddressingSetting->DestinationIPAddress, response, 500);
		osDelay(200);
		if (strcmp(response, "OK\r") == 0) {
			WLAN_State_InitAddressSetting = 30;
		} else if (strcmp(response, "ERROR\r") == 0)
			WLAN_State_InitAddressSetting = 900;
			error_NoAccesPoint = True;
		break;

	case (30): /*SourcePort*/
		XBeeCommandSend(AddressingSetting->SourcePort, response, 500);
		osDelay(200);
		if (strcmp(response, "OK\r") == 0) {
			WLAN_State_InitAddressSetting = 40;
		} else if (strcmp(response, "ERROR\r") == 0)
			WLAN_State_InitAddressSetting = 900;
		break;

	case (40): /*DestinationPort*/
		XBeeCommandSend(AddressingSetting->DestinationPort, response, 500);
		osDelay(200);
		if (strcmp(response, "OK\r") == 0) {

			/*the last 3 commands are enable just if DHCP is deactivated*/
			if (!WLAN_DHCPIsActive)
				WLAN_State_InitAddressSetting = 50;
			else
				WLAN_State_InitAddressSetting = 1000;
		} else if (strcmp(response, "ERROR\r") == 0)
			WLAN_State_InitAddressSetting = 900;
		break;

	case (50): /*GatewayIPAddress*/
		XBeeCommandSend(AddressingSetting->GatewayIPAddress, response, 500);
		osDelay(200);
		if (strcmp(response, "OK\r") == 0) {
			WLAN_State_InitAddressSetting = 60;
		} else if (strcmp(response, "ERROR\r") == 0)
			WLAN_State_InitAddressSetting = 900;
		break;

	case (60): /*IPAddressMask*/
		XBeeCommandSend(AddressingSetting->IPAddressMask, response, 500);
		osDelay(200);
		if (strcmp(response, "OK\r") == 0) {
			WLAN_State_InitAddressSetting = 70;
		} else if (strcmp(response, "ERROR\r") == 0)
			WLAN_State_InitAddressSetting = 900;
		break;

	case (70): /*ModuleIPAddress*/
		XBeeCommandSend(AddressingSetting->ModuleIPAddress, response, 500);
		osDelay(200);
		if (strcmp(response, "OK\r") == 0) {
			WLAN_State_InitAddressSetting = 1000;
		} else if (strcmp(response, "ERROR\r") == 0)
			WLAN_State_InitAddressSetting = 900;
		break;

	case (900): /*Error state*/
		commandModeFlag = False;
		WLAN_State_InitNetworkSetting = 0;
		return -1;
		break;

	case (1000): /*Successfully finished*/

		/*save and exit*/
		TON_Start[1] = True; /*start XBee_AT_Start() timeout timer*/

		WLAN_State_End = 0;

		if (XBee_AT_SaveAndExit() == 1)
			WLAN_State_InitAddressSetting = 0;
		else {
			WLAN_State_InitAddressSetting = 900;
			break;
		}

		TON_Start[1] = False; /*stop XBee_AT_Start() timeout timer*/

		return 1;
		break;

	default:
		break;
	}
}

/**
 * @brief  With this function you can send an AT command,
 * 		and checking response.
 * @param  cmd: AT command
 * 		res: response of AT command
 * 		timeout: Response deadline
 * @retval Anything can be
 */
int XBeeCommandSend(char *cmd, char *res, int timeout) {
	timingTimeout = timeout;
	UART_NewDataToInterprate = False;
	//toDo: undo cause of testing usart rxPosition = 0;
	/*reset rxBuffer*/
	for (int i = 0; i < sizeof(rxBuffer); i++)
		rxBuffer[i] = 0;

	UART_WriteTextLine(cmd);

	if (DebugMode) {
		//Debug(cmd);
	}

	memset(res, 0, sizeof(res));

	//while(UART_NewDataToInterprate != True && timingTimeout != 0);
	UART_NewDataToInterprate = False;
	//toDo: undo cause of testing usart  rxPosition = 0;
	for (int i = 0; UART_ReceicedData[i] != '\r' && (i < 50); i++)
		res[i] = UART_ReceicedData[i];

	return 0;
}

/**
 * put the module in AT Command mode
 * if there where any communication problem in order to start AT Mode , this function tries to solve the problem
 * @return 1-Module is in AT Mode		-1-starting at mode failed
 */
int XBee_AT_Start(void) {
	uint8_t repeats = 0;
	while (WLAN_State_Start != 1000 || WLAN_State_Start != 900 || !TON_Out[0]) {
		switch (WLAN_State_Start) {
		case (0): /*sending starter*/

			do {
				osDelay(1000);
				XBeeCommandSend("+++", response, 500);
				osDelay(200);
				repeats++;
				if(repeats > 5)
				{
					error_XBee_NoResponse = True;
					break;
				}
			} while (strcmp(response, "OK\r") != 0);

			if (strcmp(response, "OK\r") == 0) {
				WLAN_State_Start = 1000;
				commandModeFlag = True;
				repeats = 0;
			} else
				WLAN_State_Start = 10;
			break;

		case (10): /*check: if already in command mode*/
			XBeeCommandSend("ATCN\r", response, 500);
			osDelay(200);
			if (strcmp(response, "OK\r") == 0) {
				WLAN_State_Start = 1000;
			} else
				WLAN_State_Start = 20;
			break;

		case (20): /*check: if CR would be attached*/
			if (commandModeFlag == True) {
				commandModeFlag = False;

				XBeeCommandSend("+++", response, 500);
				osDelay(200);
				if (strcmp(response, "OK\r") == 0) {
					WLAN_State_Start = 1000;
					commandModeFlag = True;
				} else
					WLAN_State_Start = 900;
			} else
				WLAN_State_Start = 900;

			break;

		case (900): /*Error state*/
			//commandModeFlag = False;
			WLAN_State_Start = 0;
			error_XBee_NoResponse = True;
			return -1;
			break;

		case (1000): /*Successfully finished*/
			//commandModeFlag = False;
			error_XBee_NoResponse = False;
			WLAN_State_Start = 0;
			return 1;
			break;

		default:
			break;
		}
	}
}

/**
 * save parameters and exit from at mode
 * @return 1-succussfully saved and exit	-1-fail to save and exit
 */
int XBee_AT_SaveAndExit(void) {
	while (WLAN_State_End != 1000 || WLAN_State_End != 900 || !TON_Out[1]) {
		switch (WLAN_State_End) {
		case (0): /*check if module is in at command mode if not enter the command mode*/

			commandModeFlag = False;

			XBeeCommandSend("+++", response, 500);
			osDelay(200);
			if (strcmp(response, "OK\r") == 0) {
				WLAN_State_End = 10;
				commandModeFlag = True;
			} else
				WLAN_State_End = 900;

			break;

		case (10): /*make changes persistent*/

			XBeeCommandSend("ATWR\r", response, 500);
			osDelay(200);
			if (strcmp(response, "OK\r") == 0) {
				WLAN_State_End = 20;
				commandModeFlag = True;
			} else
				WLAN_State_End = 900;

			break;

		case (20): /*apply changes*/

			XBeeCommandSend("ATAC\r", response, 500);
			osDelay(200);
			if (strcmp(response, "OK\r") == 0) {
				WLAN_State_End = 30;
				commandModeFlag = True;
			} else
				WLAN_State_End = 900;

			break;

		case (30): /*exit command mode*/

			XBeeCommandSend("ATCN\r", response, 500);
			osDelay(200);
			if (strcmp(response, "OK\r") == 0) {
				WLAN_State_End = 1000;
				commandModeFlag = True;
			} else
				WLAN_State_End = 900;

			break;

		case (900): /*Error state*/
			commandModeFlag = False;
			error_XBee_NoResponse = True;
			WLAN_State_End = 0;
			return -1;
			break;

		case (1000): /*Successfully finished*/
			commandModeFlag = False;
			WLAN_State_End = 0;
			return 1;
			break;

		default:
			break;
		}

	}
}

/**
 * sending live signal
 */
void XBee_SendLiveSignal() {
	UART_WriteTextLine("AGV live signal\n\r");
	error_NoAccesPoint = False;
	error_XBee_NoResponse = False;
}

void FC_CalculateSegmentProgress()
{
	uint32_t segProcess = 0;
	uint32_t segLength = atoi(currentSegmentLength);

    if (agv_Segment_End_reached)	//reset Segmentprogress
    {
    	//reset DistanceMovedByMotor
    	ResetLeftSegmentProgessData = True;
    	ResetRightSegmentProgessData = True;
    	agv_Segment_End_reached = False;
    	LeftMotorArray [Dist_Arr][0] = 0;
    	RightMotorArray [Dist_Arr][0] = 0;
    }

    DistanceMovedByRobot = (LeftMotorArray [Dist_Arr][0] + RightMotorArray [Dist_Arr][0] )/ 2;
	segProcess = ((DistanceMovedByRobot * 100)/segLength)*100;

	if(segProcess > 9999)
	{
		segProcess = 9999;
	}

    for (int i = 3; i >= 0; --i, segProcess /= 10)
    {
    	fc_telegram_segmentprocess[i] = (segProcess % 10) + '0';
    }
}

short FC_GetCRC(uint8_t data[], int start, int length) // CRC calculation
	{
		int crc = 0xFFFF; // initial value
		int polynomial = 0x1021;

		for (int i = 0; i < length; i++)
		{
			uint8_t b = data[start + i];
			for (int k = 0; k < 8; k++)
			{
				boolean bit = ((b >> (7 - k) & 1) == 1);
				boolean c15 = ((crc >> 15 & 1) == 1);
				crc <<= 1;

				if (c15 ^ bit)
					crc ^= polynomial;
			}
		}

		return (short) crc;
	}

void FC_Transmission_Out(FC_Dataset dataset, boolean sendData)
{
	//fc_sendData[] = "#TCV00001D000100010500020550006100A8833F0000100000!";
	//fc_sendDataPosition = 0 ;

	switch (dataset)
	{
		case 0:
			fc_sendDataPosition = 1;
			FC_FillDataset(fc_telegram_agv_id);
			break;
		case 1:
			fc_sendDataPosition = 9;
			FC_FillDataset(fc_telegram_type);
			break;
		case 2:
			fc_sendDataPosition = 10;
			FC_FillDataset(fc_telegram_counter);
			break;
		case 3:
			fc_sendDataPosition = 12;
			FC_FillDataset(fc_telegram_version);
			break;
		case 4:
			fc_sendDataPosition = 14;
			FC_FillDataset(fc_telegram_reserve_a);
			break;
		case 5:
			fc_sendDataPosition = 16;
			FC_FillDataset(fc_telegram_reserve_b);
			break;
		case 6:
			fc_sendDataPosition = 18;
			FC_FillDataset(fc_telegram_max_speed);
			break;
		case 7:
			fc_sendDataPosition = 22;
			FC_FillDataset(fc_telegram_current_speed);
			break;
		case 8:
			fc_sendDataPosition = 26;
			FC_FillDataset(fc_telegram_battery);
			break;
		case 9:
			fc_sendDataPosition = 30;
			FC_FillDataset(fc_telegram_rfid);
			break;
		case 10:
			fc_sendDataPosition = 40;
			FC_FillDataset(fc_telegram_segmentprocess);
			break;
		case 11:
			fc_sendDataPosition = 44;
			FC_FillDataset(fc_telegram_isidle);
			break;
	}

	if (sendData)
	{
		int crcValue = (int)FC_GetCRC(fc_sendData, 0, 44);
		char numberArray[8];
		sprintf(numberArray,"%ld", crcValue);

		UART_WriteTextLine(fc_sendData);
		UART_WriteTextLine(numberArray);
		UART_WriteTextLine("]!\n\r");
		UART_ResetRxBuffer();
	}
}

void FC_FillDataset(char data[])
{
	for(int i = 0 ; i < strlen(data); i++)
	{
		fc_sendData[fc_sendDataPosition] = data[i];
		fc_sendDataPosition++;
	}
}

void FC_Order_Send() {
	fc_telegram_counter[1] = telegramCounter + '0';

	if(agv_Segment_End_reached)
	{
		fc_telegram_segmentprocess[0] = '0';
		fc_telegram_segmentprocess[1] = '0';
		fc_telegram_segmentprocess[2] = '0';
		fc_telegram_segmentprocess[3] = '0';
	}

	FC_Transmission_Out(FC_Telegram_agv_id, False);
	FC_Transmission_Out(FC_Telegram_type, False);
	FC_Transmission_Out(FC_Telegram_counter, False);
	FC_Transmission_Out(FC_Telegram_version, False);
	FC_Transmission_Out(FC_Telegram_reserve_a, False);
	FC_Transmission_Out(FC_Telegram_reserve_b, False);
	FC_Transmission_Out(FC_Telegram_max_speed, False);
	FC_Transmission_Out(FC_Telegram_current_speed, False);
	FC_Transmission_Out(FC_Telegram_battery, False);
	FC_Transmission_Out(FC_Telegram_rfid, False);
	FC_Transmission_Out(FC_Telegram_segmentprocess, False);
	FC_Transmission_Out(FC_Telegram_isidle, True);


	if(telegramCounter >= 9)
	{
		telegramCounter = 0;
	}
	else
	{
		telegramCounter++;
	}
}
/*UART Functions*/
/**
 * @brief  In this function, a character is sent.
 * @param  None
 * @retval None
 */
void UART_SendByte(char data) {
	HAL_UART_Transmit(&huart1, data, 1, 100);
}

/**
 * @brief  In this function, a text is sent.
 * @param  None
 * @retval None
 */
void UART_WriteTextLine(uint8_t *pBuffer) {
	HAL_UART_Transmit(&huart1, pBuffer, strlen(pBuffer), 10);
}

/**
 * copy the data from uart rx into local buffer and wirte the rest of buffer with 0s
 */
void UART_CopyDataToLocalBuffer_1() {
	/*write data on the local buffer*/
	for (int i = 0; i < rxPosition; i++)
		UART_ReceicedData[i] = rxBuffer[i];

	/*write the rest of local buffer with 0*/
	for (int i = rxPosition; i < UART_ReceicedData_Size; i++)
		UART_ReceicedData[i] = 0;
}

/**
 * reset uart buffer and set the pointer back to the start of array
 */
void UART_ResetRxBuffer() {
	for (int i = 0; i < rxBufferSize; i++)
		rxBuffer[i] = 0;

	rxPosition = 0;
}

/**
 * interpret the received data
 */
void UART_InterpretReceivedData() {
	if (strcmp(UART_ReceicedData, "Start\n") == 0) {
		//LEDSet();
		CMD_Start = True;

		//reset the function just in case of the receiving right command
		UART_NewDataToInterprate = False;
	}

	if (strcmp(UART_ReceicedData, "Stop\n") == 0) {
		//LEDReset();
		CMD_Start = False;

		UART_NewDataToInterprate = False;
	}

}

/**
 * callback of uart
 * @param UartHandle
 * HAL_UART_RxCpltCallback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
	//check buffer over flow : reset the buffer , old data would be lost
	if (rxPosition >= rxBufferSize) {
		rxPosition = 0;
		rxBuffer[rxPosition] = UART_ReceicedByte[0];
	}

	//if received the end delimiter then write the received data into a buffer and set a data received variable
	if ((UART_ReceicedByte[0] == EndDelimiter)
			|| (UART_ReceicedByte[0] == EndDelimiter1)) //received the end delimiter
			{
		//copy the end delimiter also into the received data and increase the buffer index
		rxBuffer[rxPosition] = UART_ReceicedByte[0];
		rxPosition++;

		UART_CopyDataToLocalBuffer_1();
		UART_NewDataToInterprate = True;
		UART_ResetRxBuffer();
	} else //received use data
	{
		rxBuffer[rxPosition] = UART_ReceicedByte[0];
		rxPosition++;
	}
}

/**************Testing**************************/
/**
 * all the test funcitons
 */

/**
 * test the sending command of uart
 * all sent command should be received by xbee module
 */
void Test001_UART_SendCommand() {
	UART_WriteTextLine("+++");
	osDelay(1000);

	UART_WriteTextLine("AT");
	osDelay(1000);
}

/**
 * test the CAN network
 * sending some delibrate data to each driver to see if CAN works properly
 */
void Test002_CAN_SendingTest() {
	if (ButtomPushed) {
		Motor_SpeedCommand(CANDeviceID_MotorDriver_Right, tDirection_R,
				tSpeed_R);
		Motor_SpeedCommand(CANDeviceID_MotorDriver_Left, tDirection_L,
				tSpeed_L);
	}
}

void Test003_Communication_Uart() {
	XBeeCommandSend("+++", response, 1500);
}

void Test004_Communication_ATStart() {
	XBee_AT_Start();
}

/*
 * testing the CheckCRC function
 * toTest the function: this array has a valid CRC Check
 * 						change one of the bytes of array and the result should be False
 * */
void Test005_AICP_CheckCRC() {
	uint8_t DeviationCoefficient[10] = { 0x00, 0x16, 0x33, 0x44, 0x30, 0x02,
			0x00, 0x00, 0x2c, 0xcd };

	boolean result = AICP_CheckCRC(DeviationCoefficient);

}

/**
 * move with obstacle detection
 */
void Test006_IBN1_MoveWithObstacleDetection() {

	if (ButtomPushed) {
		MotorDriver_Start = !MotorDriver_Start;		//starting the motor driver
		osDelay(300);
	}

	MoveStartCondition = MotorDriver_Start	& !ObstacleDetection_Detected_WithDelay;

	if (MoveStartCondition) {
		Motor_SpeedCommand(CANDeviceID_MotorDriver_Right, tDirection_R,	tSpeed_R);
		Motor_SpeedCommand(CANDeviceID_MotorDriver_Left, tDirection_L, tSpeed_L);
		tMotorStatusFlag = True;
		osDelay(50);
	} else {
		Motor_SpeedCommand(CANDeviceID_MotorDriver_Right, tDirection_R, 0);
		Motor_SpeedCommand(CANDeviceID_MotorDriver_Left, tDirection_L, 0);
		tMotorStatusFlag = False;
		osDelay(50);
	}

}

void Test007_DataSendToRFIDModul() {
	uint8_t dataToSend[2];
	dataToSend[0] = 0xEE; //function Code
	dataToSend[1] = 0xFF;	//some data
	//making the header
	TxHeader.DLC = 3;		//setting data length
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = CANDeviceID_RFID;	//setting the device can id

	CAN_Transmit(&TxHeader, dataToSend);	//sending the can packet
}

void Test008_GetDistance(){
	//distance moved is the avg of distance moved by the individual motors
	DistanceMovedByRobot= (LeftMotorArray [SegmentProgessData][0] + RightMotorArray [SegmentProgessData][0] )/ 2;
/*//Uncomment to check the reset segement progess data it resets after the robot has reached 3000 mm
 * 	if (DistanceMovedByRobot > 3000){
		ResetRightSegmentProgessData = True;
		ResetLeftSegmentProgessData = True;
	}*/

}

void Test009_DataSendToObstacleModul() {
	uint8_t dataToSend[2];
	dataToSend[0] = 0xEE; //function Code
	dataToSend[1] = 0xFF;	//some data
	//making the header
	TxHeader.DLC = 3;		//setting data length
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = CANDeviceID_BarrierSensor_Head;	//setting the device can id

	CAN_Transmit(&TxHeader, dataToSend);	//sending the can packet
}

/**************Utility**************************/

/**
 * get the state of Push Bottom
 * @return 1: pushed  , 0: not pushed
 */
boolean PushButtom_GetState() {
	return HAL_GPIO_ReadPin(USER_BT_GPIO_Port, USER_BT_Pin) == KEY_PRESSED;
}
//using Stm32 button to give input and generate task on fleet controller
void Send_DataToTc (void){
	if (ButtomPushed){
//		FC_Order_Send();

		fc_telegram_reserve_a[0] = '9';
		fc_telegram_reserve_a[1] = '9';
		fc_telegram_reserve_b[0] = '9';
		fc_telegram_reserve_b[1] = '9';
	}
		else
		{fc_telegram_reserve_a[0] = '0';
		fc_telegram_reserve_a[1] = '0';
		fc_telegram_reserve_b[0] = '0';
		fc_telegram_reserve_b[1] = '0';
		}
}
///**
//  * @brief  Send the debug string to uart2
//  * @param  None
//  * @retval None
//  */
//void Debug(char * pBuffer)
//{
//	if(DebugEnable)
//		HAL_UART_Transmit(&huart2, pBuffer , strlen(pBuffer) , 10);
//}

/**
 * all the tasks that should be done in the first cycle
 */
void FirstCycle() {

	Timer_CalculatePTs();

	IsFirstCycle = False;
}

/**************LED**************************/
void ExcuteSchema()
{
	switch (ExcuteSchema_State)
	{
	case (0):	//changing the state of LED
			switch (LED_Port_State)
			{
			case LEDSignalPort_MCU:	LEDState(LEDSignal_SchemaToExcute.LEDState[LEDSchema_Index]); break;
			case LEDSignalPort_Error: ErrorLEDState(LEDSignal_SchemaToExcute.LEDState[LEDSchema_Index]); break;
			case LEDSignalPort_Status: StatusLEDState(LEDSignal_SchemaToExcute.LEDState[LEDSchema_Index]); break;
			}
		ExcuteSchema_State++;
		break;
	case (1):	//excute delay on this schema state
		LEDSchema_DelayCounter++;	//increasing schema counter

		if (LEDSchema_DelayCounter >= LEDSignal_SchemaToExcute.BlinkDelay[LEDSchema_Index])//changing to the next schema index
				{
			LEDSchema_Index++;
			LEDSchema_DelayCounter = 0;
			ExcuteSchema_State++;
		}
		break;
	case (2):	//check if the schema is finished?
		if (LEDSchema_Index >= LEDSignal_SchemaToExcute.MaxState)//after finishing the schema
				{
			LEDSchema_Index = 0;
			LEDSchema_Done = True;
			ExcuteSchema_State = 4;
		} else
			//if the schema wasnt finished, start from state 0
			ExcuteSchema_State = 0;
		break;
	case (4):	//after excuting the schema, setting the mode to Idle
		if (LEDSchema_Done) {
			LED_State = LED_NextState;//set the next state as the current excutable state
			LED_NextState = 0; 		//reset the next state
			LEDSchema_Done = False;
		}
		ExcuteSchema_State = 0;	//reset to the state machine
		break;
	default:
		ExcuteSchema_State = 0;
		break;
	}
}

/**
 * Changing the State of LED
 */
void LEDState(boolean LEDState) {
	HAL_GPIO_WritePin(LED_MCU_GPIO_Port, LED_MCU_Pin, LEDState);
}

void ErrorLEDState(boolean LEDState) {
	HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, !LEDState);
}

void StatusLEDState(boolean LEDState)
{
	HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, LEDState);
}

//execute FC Orders
void FC_Controller()
{
	TC_SignalIsFreshCheck = HAL_GetTick();

	if (FC_Order[1] == 'r' && FC_Order[2] == 'd' && FC_Order[3] == 'y')
	{
		TC_Is_Rdy_For_Transmission = True;
		FC_Order_Send();
	}
	else
	{
	int crcCheck = (int) FC_GetCRC(FC_Order, 0, 36);
	char numberArray[5];
	sprintf(numberArray, "%ld", crcCheck);

	if (numberArray[0] == FC_Order[37] &&
		numberArray[1] == FC_Order[38] &&
		numberArray[2] == FC_Order[39] &&
		numberArray[3] == FC_Order[40])
	{
		for (int i = 0; i < 6; i++)
		{
			currentSegmentLength[i] = (char) FC_Order[i + 14];//reading current Segment length
		}
		FC_CalculateSegmentProgress();

		for (int i = 20; i < 30; i++)	// getting destination RFID.
		{
			fc_telegram_destination_rfid[i - 20] = FC_Order[i];
		}

		if (!maintenanceMode && startPositionReached)
		{
			if (FC_Order[34] == '0')// getting actioncode. 0 = is job started, -1 no job for this agv
			{
				fc_telegram_isidle[0] = '0';
			}
			else
			{
				fc_telegram_isidle[0] = '1';
			}

			if (fc_telegram_isidle[0] == '0')// stop at segment destination if agv has a running job
			{
				agv_destination_reached = True;

				for (int i = 0; i < 10; i++)
				{
					if (fc_telegram_rfid[i]	!= fc_telegram_destination_rfid[i])
					{
						agv_destination_reached = False;
					}
				}

				if (agv_destination_reached)
				{
					MotorDriver_Start = False;
				}
				else
				{
					MotorDriver_Start = True;
				}
			}
			else
			{
				MotorDriver_Start = False;
			}
		}
	}
	else
	{
		HAL_GPIO_TogglePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin);
	}
	}
}

void fillingErrorQueue()
{
	(error_undervolt) ? (LEDErrorQueue[0] = 4) : (LEDErrorQueue[0] = 0);
	(error_CAN_error) ? (LEDErrorQueue[1] = 5) : (LEDErrorQueue[1] = 0);
	(error_ModulesNotFound) ? (LEDErrorQueue[2] = 6) : (LEDErrorQueue[2] = 0);
	(error_CorruptData) ? (LEDErrorQueue[3] = 7) : (LEDErrorQueue[3] = 0);
	(error_NoAccesPoint) ? (LEDErrorQueue[4] = 8) : (LEDErrorQueue[4] = 0);
	(error_XBee_NoResponse) ? (LEDErrorQueue[5] = 9) : (LEDErrorQueue[5] = 0);
	(maintenanceMode) ? (LEDErrorQueue[6] = 10) : (LEDErrorQueue[6] = 0);
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
 {
	/* USER CODE BEGIN 5 */
	osTimerStart(Timer_LinescannerHandle, 1500);
//	osTimerStart(Timer_MotormoduleHandle, 1000);
	/* Infinite loop */
	for (;;)
	{
	/*HAL_GetTick() gives the counter value that is increasing at the rate of 1ms for the point the microcontroller is turned ON.
	Current_Ticks stores the tick value at this particular instance.
	LineScanLiveCheck/ ObstacleDetectionLiveCheck/ RightMotorLiveCheck/ LeftMotorLiveCheck also stores the tick values and
	these values are updated respectively when the corresponding CAN packet is recieved from the modules.
	If the DIff of Counter_Ticks and the respective "Checks" increase  the specified limits (in ms). it registers that there is a problem with the Module

	 */
		Current_Ticks =  HAL_GetTick();
		if((Current_Ticks - LineScanLiveCheck) > 300){
			error_LineScan_CAN = True;
		}else error_LineScan_CAN =False;

		if((Current_Ticks - ObstacleDetectionLiveCheck) > 300){
			error_ObstacleDetectionModule = True;
		}else error_ObstacleDetectionModule =False;

		if((Current_Ticks - RightMotorLiveCheck) > 2000 ){
			error_Motordriver_Right = True;
		}else error_Motordriver_Right = False;

		if((Current_Ticks - LeftMotorLiveCheck) > 2000 ){
			error_Motordriver_Left = True;
		}else error_Motordriver_Left = False;

		if((Current_Ticks - TC_SignalIsFreshCheck) > 1000 )
		{
			if (XBee_AddressingSetting_Initialized	&& TC_Is_Rdy_For_Transmission)
			{
				HAL_GPIO_WritePin(XBEE_RESET_GPIO_Port, XBEE_RESET_Pin, False);
				osDelay(50);
				TC_Is_Rdy_For_Transmission = False;
				HAL_GPIO_WritePin(XBEE_RESET_GPIO_Port, XBEE_RESET_Pin, True);
			}
		}

		if(((Current_Ticks - CAN_TxRate) > 997) && (CorrectPacket == True) ){
			LightControllerTxData[0] = MainModule_ON;
			//
			SendData( CANDeviceID_LightController,  LightControllerTxData);
			CAN_TxRate = Current_Ticks;
		} else if (((Current_Ticks - CAN_TxRate) > 223) && (CorrectPacket == False) ){
			LightControllerTxData[0] = MainModule_ON;
			SendData( CANDeviceID_LightController,  LightControllerTxData);
			CAN_TxRate = Current_Ticks;
		}

		if((Current_Ticks - LightControllerLiveCheck) > 2000 ){
			error_LightController = True;
		}else error_LightController = False;


		if((/*error_LineScan_CAN || error_ObstacleDetectionModule)||(error_Motordriver_Left || error_Motordriver_Right ||*/ error_LightController))
		{
			HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin,GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
		}

			ButtomPushed = PushButtom_GetState();
			Send_DataToTc();
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_LineScannerTask_Main */
/**
 * @brief Function implementing the LineScannerTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LineScannerTask_Main */
void LineScannerTask_Main(void *argument)
{
  /* USER CODE BEGIN LineScannerTask_Main */
	/* Infinite loop */
	for (;;) {

		if(LineScanner_LostLine)
		{
			fc_telegram_reserve_b[1] = '1';
		}
		else
		{
			fc_telegram_reserve_b[1] = '0';
		}

		//reset the marker by finding a line
		if ((Lines.NumberFoundLines > 0) ){
			/*
			 * write the value in a memory
			 * why we need this memory? in the case of losting the line, the value of Line[0] would be 0 and the LostLine is not activated cause it has a timer
			 * in this case the robot try to tilt to one side which is not desireable.
			 */
			mLine = Lines.Line[0];

			LineScanner_LostLine = False;
			TON_Start[3] = False;
		}
		/*		  //Start the timer when curve is detected.
		 if (CurveDetected && (MightBeOscillating == 0) & !TON_Start[5]){
		 TON_Start[5] = True;
		 }
		 if ((TON_Out[5] && (Oscillator_count <= 0 )) && (straightlineCounter >0)){
		 Curve_present = True;
		 pt_eight_curve = True;
		 }
		 if( (Curve_present & (MightBeOscillating == 1) ) ){
		 Curve_present = False;
		 pt_eight_curve = False;
		 }*/

		//start the timer if the robot lost the Lines
		if (((Lines.NumberFoundLines == 0)||(error_LineScan_CAN  == True)) & !TON_Start[3])
			TON_Start[3] = True;

		//set the marker after time elapsed
		if (TON_Out[3])
			LineScanner_LostLine = True;

		osDelay(3);

	}
  /* USER CODE END LineScannerTask_Main */
}

/* USER CODE BEGIN Header_ObstacleDetectionTask_Main */
/**
 * @brief Function implementing the ObstacleDetecti thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ObstacleDetectionTask_Main */
void ObstacleDetectionTask_Main(void *argument)
{
  /* USER CODE BEGIN ObstacleDetectionTask_Main */
	/* Infinite loop */
	for (;;) {
		ObstacleDetection_Detected = ObstacleDetection_ProcessSensorSignalsSignals();
		ObstacleDetection_Detected_WithDelay = ObstacleDetection_ResumeDelay(ObstacleDetection_Detected);

		if(ObstacleDetection_Detected_WithDelay)
		{
			fc_telegram_reserve_a[1] = '1';
		}
		else
		{
			fc_telegram_reserve_a[1] = '0';
		}
		osDelay(3);
	}
  /* USER CODE END ObstacleDetectionTask_Main */
}

/* USER CODE BEGIN Header_CommunicationTask_Main */
/**
 * @brief Function implementing the CommunicationTa thread.
 * initiate and start the communication over WLAN
 * @param argument: Not used
 * @retval None
 */
/**
 *
 */
/* USER CODE END Header_CommunicationTask_Main */
void CommunicationTask_Main(void *argument)
{
  /* USER CODE BEGIN CommunicationTask_Main */
	/* Infinite loop */

	//set the Reset of XBEE Module to 1 in order to set the Module to functional state.
	HAL_GPIO_WritePin(XBEE_RESET_GPIO_Port, XBEE_RESET_Pin, True);

	for (;;) {
		/*initializing the module*/
		if (!XBee_NetworkSetting_Initialized)
		{
			if (XBeeInit_NetworkSetting(&WLAN_NetworkSetting) == 1) /*successful initialization*/
			{
				XBee_NetworkSetting_Initialized = True;
			}
		}

		/*initializing the addressing setting*/
		if (!XBee_AddressingSetting_Initialized	&& XBee_NetworkSetting_Initialized) /*after initializin the network start initing the addresseing setting*/
				{
			if (XBeeInit_AddressingSetting(&WLAN_AddressingSetting) == 1)
			{
				XBee_AddressingSetting_Initialized = True;
				TON_Start[2] = False;
			}
		}

		/**sending live signal**/
		Counter_LiveSignal++;
		if (Counter_LiveSignal >= LiveSignal_CounterLimit)
		{
			Flag_SendLiveSignal = True;
			Counter_LiveSignal = 0;
		}

		if (Flag_SendLiveSignal)
		{
			Flag_SendLiveSignal = False;
			XBee_SendLiveSignal();
			//FC_Order_Send();
		}

		//		/**receiving new data to interpret**/
		//		if(UART_NewDataToInterprate)
		//			UART_InterpretReceivedData();

		osDelay(10);
	}
  /* USER CODE END CommunicationTask_Main */
}

/* USER CODE BEGIN Header_AICPTask_Main */
/**
 * @brief Function implementing the AICPTask thread.
 * @param argument: Not used
 * @retval None
 * process received AICP Packets
 * check the packet header and find the start of packet
 * check the CRC Code
 * extract data from a packet
 * initiate and start the communication over wlan
 * Reference: read the "Diagram/AICP-DataFlow" to fully understand the data flow of this state machine
 * toDo:
 * initiate the RF Module throw AT Commands
 */

/* USER CODE END Header_AICPTask_Main */
void AICPTask_Main(void *argument)
{
  /* USER CODE BEGIN AICPTask_Main */
	/* Infinite loop */
	for (;;) {
		AICP_StateMachine();

		osDelay(2);

	}
  /* USER CODE END AICPTask_Main */
}

/* USER CODE BEGIN Header_TimerTask_Main */
/**
 * @brief Function implementing the TimerTask thread.
 * @param argument: Not used
 * @retval None
 * these timers do not stop the processor like a for or while loop
 * these timers are not accurate and can be used as delays
 * these timers should be adjusted to be accure
 * important: in order to use timers have the following tipps in mind
 * 		-active the time by setting the "MAX_ActiveTimer".
 * 			by activating more time, the cpu needs more time for calculations and would be slower.
 * 		-wirte the timer discription in the timer table at the begining of code.
 */
/* USER CODE END Header_TimerTask_Main */
void TimerTask_Main(void *argument)
{
  /* USER CODE BEGIN TimerTask_Main */
	/* Infinite loop */
	for (;;) {
		for (int i = 0; i < Timer_Max; i++)
			Timer(i);

		//decreasing the timer
		if (timingTimeout != 0) {
			timingTimeout--;
		}

		osDelay(1);
	}
  /* USER CODE END TimerTask_Main */
}

/* USER CODE BEGIN Header_TestingTask_Main */
/**
 * @brief Function implementing the TestingTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_TestingTask_Main */
void TestingTask_Main(void *argument)
{
  /* USER CODE BEGIN TestingTask_Main */
	/* Infinite loop */
	for (;;) {
		//Test001_UART_SendCommand();

		//Test002_CAN_SendingTest();

		//Test003_Communication_Uart();

		//Test004_Communication_ATStart();

		//Test005_AICP_CheckCRC();

		//Test006_IBN1_MoveWithObstacleDetection();

		//Test007_DataSendToRFIDModul();

		//Test009_DataSendToObstacleModul();
		timetester = timetester+ 1;


		osDelay(1);
	}
	/* USER CODE END TestingTask_Main */
}

/* USER CODE BEGIN Header_movetask_2 */
/**
 * @brief Function implementing the MoveTask_2 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_movetask_2 */
void movetask_2_Main(void *argument) {
	/* USER CODE BEGIN movetask_2 */
	/* Infinite loop */
	for (;;) {
		//int a = 0;
		if (ButtomPushed && maintenanceMode)
		{
			MotorDriver_Start = !MotorDriver_Start;	//starting the motor driver
			if (Move_S6Called == False)Move_S6Called = True;
			osDelay(1);
		}

		if(!maintenanceMode && fc_telegram_isidle[0] == '0' && !agv_destination_reached)
		{
			MotorDriver_Start = True;	//starting the motor driver
			if (Move_S6Called == False)Move_S6Called = True;
			osDelay(1);
		}
		else
		{
			if (!maintenanceMode && startPositionReached)
			{
				MotorDriver_Start = False;
			}
		}
		//		Lines.NumberFoundLines> 0
		//		MoveStartCondition = ((MotorDriver_Start && !LineScanner_LostLine ) || CMD_Start)
		//								& ! ObstacleDetection_Detected_WithDelay;

		if (!maintenanceMode)
		{
			if (locationFound)
			{
				DesiredSpeed = 0.15;
				if (!startPositionReached)
				{
					startPositionReached = True;
					MotorDriver_Start = False;
				}
			}
			else
			{
				DesiredSpeed = 0.1;
				MotorDriver_Start = True;
			}
		}

		MoveStartCondition = (((MotorDriver_Start && !LineScanner_LostLine)) & ! ObstacleDetection_Detected_WithDelay)/**/;

		if(error_Motordriver_Left || error_Motordriver_Right)
		{
			//HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin,GPIO_PIN_RESET);
			MoveStartCondition = False;
			fc_telegram_version[1] = '1';
		}
		else
		{
			//HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin,GPIO_PIN_SET);
			fc_telegram_version[1] = '0';
		}

		lineCounter++;

		//Move_S5(MoveStartCondition,mLine);

		Test008_GetDistance();
		Move_S6(MoveStartCondition, mLine , False);

		uint32_t currspd = 0;
		currspd = DesiredSpeed * 100;

	    for (int i = 3; i >= 0; --i, currspd /= 10)
	    {
	    	fc_telegram_current_speed[i] = (currspd % 10) + '0';
	    }
		osDelay(17);
	}
  /* USER CODE END TestingTask_Main */
}

/* USER CODE BEGIN Header_LEDTask_Main */
/**
 * @brief Function implementing the LEDTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LEDTask_Main */
void LEDTask_Main(void *argument)
{
  /* USER CODE BEGIN LEDTask_Main */
	/* Infinite loop */
	for (;;)
	{
		switch (LED_State)
		{
		case LEDSignalState_Idle:
			LEDSignal_SchemaToExcute = LEDSignalSchema_Idle; //the Idle-Signal Schema would be written into the executable schema
			break;
		case LEDSignalState_Error:
			LEDSignal_SchemaToExcute = LEDSignalSchema_Error;
			break;
		case LEDSignalState_CANRecieved:
			LEDSignal_SchemaToExcute = LEDSignalSchema_CANReceived;
			break;
		case LEDSignalState_Calibrate:
			LEDSignal_SchemaToExcute = LEDSignalSchema_Calibrate;
			break;
		case LEDSignalState_undevoltage:
			LEDSignal_SchemaToExcute = LEDSignalSchema_undevoltage;
			break;
		case LEDSignalState_CAN_Error:
			LEDSignal_SchemaToExcute = LEDSignalSchema_CAN_Error;
			break;
		case LEDSignalState_No_Modules_Found:
			LEDSignal_SchemaToExcute = LEDSignalSchema_No_Modules_Found;
			break;
		case LEDSignalState_Got_Corrupt_Data:
			LEDSignal_SchemaToExcute = LEDSignalSchema_Got_Corrupt_Data;
			break;
		case LEDSignalState_AccesPoint_Not_Reachable:
			LEDSignal_SchemaToExcute = LEDSignalSchema_AccesPoint_Not_Reachable;
			break;
		case LEDSignalState_XBEE_NoResponse:
			LEDSignal_SchemaToExcute = LEDSignalSchema_XBEE_NoResponse;
			break;
		case LEDSignalState_Maintenence_Mode:
			LEDSignal_SchemaToExcute = LEDSignalSchema_Maintenence_Mode;
			break;
		default:
			LED_State = LEDSignalState_Idle;
			break;
		}
		if (LED_State < 4)
		{
			LED_Port_State = LEDSignalPort_MCU;
			ExcuteSchema(LED_Port_State);
		}
		else if (LED_State < 10)
		{
			LED_Port_State = LEDSignalPort_Error;
			ExcuteSchema(LED_Port_State);
		}
		else
		{
			LED_Port_State = LEDSignalPort_Status;
			ExcuteSchema(LED_Port_State);
		}
		osDelay(20);

		fillingErrorQueue();

		if(LED_NextState == 0)
		{
			if(LEDErrorCounter < 7)
			{
				LED_NextState = LEDErrorQueue[LEDErrorCounter];
				LEDErrorCounter++;
			}
			else
			{
				LEDErrorCounter = 0;
			}
		}
	}
  /* USER CODE END LEDTask_Main */
}

/* timeoutLinescanner function */
void timeoutLinescanner(void *argument)
{
  /* USER CODE BEGIN timeoutLinescanner */
	if (LC_Recieved_Ack == LightControllerTxData[2]) {
		CorrectPacket = True;
		error_LightController = False;
	} else {
		CorrectPacket = False;
		error_LightController = True;
	}
	LightControllerCounter = LightControllerCounter +1;
	LightControllerTxData[2] = LightControllerCounter;


	/* USER CODE END timeoutLinescanner */
}

/* timeoutMotormodule function */
void timeoutMotormodule(void *argument)
{
  /* USER CODE BEGIN timeoutMotormodule */

// CAnnot use this approach as if the communication is restored asa this timer task is executed then status will only be updaed when this timer task is ecexuted again.
//	if (RightMotorLive == True){
//		HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
//		error_Motordriver_Right = False;
//
//	}else {
//		HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin,GPIO_PIN_RESET);
//		error_Motordriver_Right = True;
//	}
//
//	if (LeftMotorLive == True){
//		HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
//		error_Motordriver_Left = False;
//
//	}else {
//		HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin,GPIO_PIN_RESET);
//		error_Motordriver_Left = True;
//	}
//
//	RightMotorLive = False;
//	LeftMotorLive = False;

	if(error_Motordriver_Left || error_Motordriver_Right)
	{
		//HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin,GPIO_PIN_RESET);
	}

  /* USER CODE END timeoutMotormodule */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
