/*
 * ac210_can.c
 *
 *  Created on: 6/05/2017
 *      Author: Murray
 */

//#include "board.h"
#include <string.h>
#include <stdlib.h>

#include "ac210_global.h"
#include "comms.h"
#include "param.h"

//char DebugString[256];
//#define MHDEBUGOUT(...)		{sprintf(DebugString,__VA_ARGS__);Board_UARTPutSTR(DebugString);}
// Received CAN
#define MHDEBUGOUT			PRINTF			// For now

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define CAN_CTRL_NO         1
#if (CAN_CTRL_NO == 0)
#define LPC_CAN             (LPC_CAN1)
#else
#define LPC_CAN             (LPC_CAN2)
#endif
#define AF_LUT_USED         1
#if AF_LUT_USED
//#define FULL_CAN_AF_USED    1
#endif
#define CAN_TX_MSG_STD_ID (0x200)
#define CAN_TX_MSG_REMOTE_STD_ID (0x300)
#define CAN_TX_MSG_EXT_ID (0x10000200)
#define CAN_RX_MSG_ID (0x100)

#ifdef MH_CONVERT_SERIAL
void FormatSerialOutput(void)
{
    uint16_t crc;
    char *bps,*bpd;

    TxMsgBuf[0] = 0x55;       // 2 header bytes
    TxMsgBuf[1] = 0xaa;

    bps = (char *) &CanRxMsg.ID;
    bpd = (char *)(TxMsgBuf + 2);
    bpd[0] = bps[3];      // convert to bigendian
    bpd[1] = bps[2];
    bpd[2] = bps[1];
    bpd[3] = bps[0];
    memcpy(TxMsgBuf+6,CanRxMsg.Data,8);
    crc = DoCRC (0, TxMsgBuf, 14);
    TxMsgBuf[14] = (crc >> 8);        // make sure CRC bigendian
    TxMsgBuf[15] = (crc & 0xff);
//    memcpy(SvMsgBuf,TxMsgBuf,IMAX);
}
//--------------------------------------------------------------------
void SerialOutput1(void)
{
    FormatSerialOutput();
	Chip_UART_SendRB(LPC_UART2, &tx2ring, TxMsgBuf, IMAX);
}
#endif
//---------------------------------------------------------------------------------
#ifdef MH_CAN_RB
//void OutputDebug(void);
CAN_MSG_T CanRxMsg,CanTxMsg;
//---------------------------------------------------------------------------------
#define CBUFFSIZE   8
typedef struct
{
    int head;
    int tail;
    CAN_MSG_T msg[CBUFFSIZE];
} CanRingBuf_t;
//CanRingBuf_t Can1RB,Can2RB;
CanRingBuf_t Can2RB;
//-----------------------------------------------------------------------------
void CanRingBuffError(const char *emsg,CanRingBuf_t *crb)
{
    char *msg = (char *)"Can:Unknown";
//    if(crb == &Can1RB) msg = (char *)"Can1";
    if(crb == &Can2RB) msg = (char *)"Can2";
    MHDEBUGOUT("%s: %s\n\r",emsg, msg);
//    OutputDebug();
//    exit(1);
}
//-----------------------------------------------------------------------------
void CanAddRingMsg(CanRingBuf_t *crb,CAN_MSG_T *cmsg)
{
    int head = crb->head;
    crb->msg[head] = *cmsg;
//    head = (head+1) & CBUFFMASK;
    head = (head+1) % CBUFFSIZE;
    if(head != crb->tail) {
        crb->head = head;
        return;
    }
// Buffer is overflowing if we drop through
    CanRingBuffError("CanAddRingMsg:Buffer overflow",crb);
}
//-----------------------------------------------------------------------------
void CanGetRingMsg(CanRingBuf_t *crb,CAN_MSG_T *cmsg)
{
    int tail = crb->tail;
    if(crb->head == tail) {
        CanRingBuffError("CanGetRingMsg: Buffer empty",crb);
    }
    *cmsg = crb->msg[tail];
//    crb->tail = (tail+1) & CBUFFMASK;
    crb->tail = (tail+1) % CBUFFSIZE;
}
//------------------------------------------------------------------------------
#ifdef MH_XXX
void CanGetRingMsgWait(CanRingBuf_t *crb,CAN_MSG_T *cmsg)
{
    while(crb->head == crb->tail) wait_us(10);
    CanGetRingMsg(crb,cmsg);
}
#endif
//------------------------------------------------------------------------------
int CanRingMsgReady(CanRingBuf_t *crb)
{
    if(crb->head == crb->tail) return false;
    return true;
}
#endif
/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

#if AF_LUT_USED
// CanAero ID defines
#define CA_RPM_A		500
#define CA_FUELFLOW_A	524
#define CA_MAP_A		528
#define CA_RPM_B		564
#define CA_FUELFLOW_B	588
#define CA_MAP_B		592
#define	CA_THROTTLE_A	692
#define CA_THROTTLE_B	696


// Note: CAN IDs must be in ascending order
CAN_STD_ID_ENTRY_T SFFSection[] = {
//	{CAN_CTRL_NO, 0, CA_RPM_A},
//	{CAN_CTRL_NO, 0, CA_FUELFLOW_A},
//	{CAN_CTRL_NO, 0, CA_MAP_A},
//	{CAN_CTRL_NO, 0, CA_RPM_B},
//	{CAN_CTRL_NO, 0, CA_FUELFLOW_B},
//	{CAN_CTRL_NO, 0, CA_MAP_B},
	{CAN_CTRL_NO, 0, CA_THROTTLE_A},
	{CAN_CTRL_NO, 0, CA_THROTTLE_B},
};
CANAF_LUT_T AFSections = {
	NULL, 0,
	SFFSection, sizeof(SFFSection) / sizeof(CAN_STD_ID_ENTRY_T),
};
#endif /*AF_LUT_USED*/

/*****************************************************************************
 * Private functions
 ****************************************************************************/
/* Print error */
static int Can_debug=0;
int CAN_error_code;
int CAN_error_count;
static void PrintCANErrorInfo(uint32_t Status)
{
//	if(Can_debug == 0) return;
#define MH_PRINT_CAN_ERRORS
#ifdef MH_PRINT_CAN_ERRORS

	CAN_error_code = 0;
	if((Status & 0xFF) == 0) return;	// MHH:28/05/2025

	if (Status & CAN_ICR_EI) {
		CAN_error_code = 1;
//		PRINTF("CAN:Error Warning!\r\n");
	}
	if (Status & CAN_ICR_DOI) {
		CAN_error_code = 2;
//		PRINTF("CAN:Data Overrun!\r\n");
	}
	if (Status & CAN_ICR_EPI) {
		CAN_error_code = 3;
//		PRINTF("CAN:Error Passive!\r\n");
	}
	if (Status & CAN_ICR_ALI) {
		CAN_error_code = 4;
//		PRINTF("CAN:Arbitration lost in the bit: %d(th)\r\n", CAN_ICR_ALCBIT_VAL(Status));
	}
	if (Status & CAN_ICR_BEI) {

//		PRINTF("CAN:Bus error !!!\r\n");

		if (Status & CAN_ICR_ERRDIR_RECEIVE) {
			CAN_error_code = 5;
//			PRINTF("\t CAN:Error Direction: Transmitting\r\n");
		}
		else {
			CAN_error_code = 6;
//			PRINTF("\t CAN:Error Direction: Receiving\r\n");
		}

//		PRINTF("\t CAN:Error Location: 0x%2x\r\n", CAN_ICR_ERRBIT_VAL(Status));
//		PRINTF("\t CAN:Error Type: 0x%1x\r\n", CAN_ICR_ERRC_VAL(Status));
	}
#endif
}

/* Print CAN Message */
void PrintCANMsg(CAN_MSG_T *pMsg)
{
	uint8_t i;
	MHDEBUGOUT("\t**************************\r\n");
	MHDEBUGOUT("\tMessage Information: \r\n");
	MHDEBUGOUT("\tMessage Type: ");
	if (pMsg->ID & CAN_EXTEND_ID_USAGE) {
		MHDEBUGOUT(" Extend ID Message");
	}
	else {
		MHDEBUGOUT(" Standard ID Message");
	}
	if (pMsg->Type & CAN_REMOTE_MSG) {
		MHDEBUGOUT(", Remote Message");
	}
	MHDEBUGOUT("\r\n");
	MHDEBUGOUT("\tMessage ID :0x%x\r\n", (pMsg->ID & (~CAN_EXTEND_ID_USAGE)));
	MHDEBUGOUT("\tMessage Data :");
	for (i = 0; i < pMsg->DLC; i++)
		MHDEBUGOUT("%x ", pMsg->Data[i]);
	MHDEBUGOUT("\r\n\t**************************\r\n");
}

#ifdef MH_REPLY_REMOTE
/* Reply remote message received */
static void ReplyRemoteMessage(CAN_MSG_T *pRcvMsg)
{
	CAN_MSG_T SendMsgBuf;
	CAN_BUFFER_ID_T   TxBuf;
	uint8_t i;
	SendMsgBuf.ID  = pRcvMsg->ID;
	SendMsgBuf.DLC = pRcvMsg->DLC;
	SendMsgBuf.Type = 0;
	for (i = 0; i < pRcvMsg->DLC; i++)
		SendMsgBuf.Data[i] = '0' + i;
	TxBuf = Chip_CAN_GetFreeTxBuf(LPC_CAN);
	Chip_CAN_Send(LPC_CAN, TxBuf, &SendMsgBuf);
	MHDEBUGOUT("Message Replied!!!\r\n");
	PrintCANMsg(&SendMsgBuf);
}
#endif

#ifdef MH_REPLY_NORMAL

/* Reply message received */
static void ReplyNormalMessage(CAN_MSG_T *pRcvMsg)
{
	CAN_MSG_T SendMsgBuf = *pRcvMsg;
	CAN_BUFFER_ID_T   TxBuf;
	SendMsgBuf.ID = CAN_TX_MSG_STD_ID;
	TxBuf = Chip_CAN_GetFreeTxBuf(LPC_CAN);
	Chip_CAN_Send(LPC_CAN, TxBuf, &SendMsgBuf);
	MHDEBUGOUT("Message Replied!!!\r\n");
	PrintCANMsg(&SendMsgBuf);
}
#endif

/* Print entries in AF LUT */
void PrintAFLUT(void)
{
	uint16_t i, num;
	CAN_STD_ID_ENTRY_T StdEntry;
	/* Standard ID Table */

	if(Can_debug < 2) return;
	MHDEBUGOUT("\tIndividual Standard ID Table: \r\n");
	num = Chip_CAN_GetEntriesNum(LPC_CANAF, LPC_CANAF_RAM, CANAF_RAM_SFF_SEC);
	for (i = 0; i < num; i++) {
		Chip_CAN_ReadSTDEntry(LPC_CANAF, LPC_CANAF_RAM, i, &StdEntry);
		MHDEBUGOUT("\t\t%d: Controller ID: %d, ID: 0x%x, Dis: %1d\r\n",
				 i, StdEntry.CtrlNo, StdEntry.ID_11, StdEntry.Disable);
	}
}

/* Setup AF LUT */
#ifdef MH_XXX
static void SetupAFLUT(void)
{
//	MHDEBUGOUT("Setup AF LUT... \r\n");
	Chip_CAN_SetAFLUT(LPC_CANAF, LPC_CANAF_RAM, &AFSections);
	//PrintAFLUT();
}
#endif

/*****************************************************************************
 * Public functions
 ****************************************************************************/
/**
 * @brief CAN Message Object Structure
 */
#ifdef MH_DEFINE_CAN_MESSAGE
typedef struct						/*!< Message structure */
{
	uint32_t ID;					/*!< Message Identifier. If 30th-bit is set, this is 29-bit ID, othewise 11-bit ID */
	uint32_t Type;					/*!< Message Type. which can include: - CAN_REMOTE_MSG type*/
	uint32_t DLC;					/*!< Message Data Length: 0~8 */
	uint8_t  Data[CAN_MSG_MAX_DATA_LEN];/*!< Message Data */
} CAN_MSG_T;


memcpy((char *)&CA,(char *)Rec.msg.data,8);
LoadCheck_F4();
CanThrottle = CA_F4;

CA_F4 = OldPotThrottle;
Store_F4();         // Stores in Bigendian in CA.Data
memcpy(Rec.msg.data+4,CA.Data,4);       // Copy into record
#endif

#define UCHAR unsigned char
static struct CANAero_struct {
    UCHAR NodeID;
    UCHAR DataType;
    UCHAR ServiceCode;
    UCHAR MessageCode;
    UCHAR Data[4];
    } CA;

static float CA_F4;
static UCHAR BigEndianChar[8];
#ifdef MH_CAN_STORE
void Store_F4()
{

    float *fp;

// Reverse of load.

    fp = (float *)BigEndianChar;
    *fp = CA_F4;

    CA.Data[3] = BigEndianChar[0];
    CA.Data[2] = BigEndianChar[1];
    CA.Data[1] = BigEndianChar[2];
    CA.Data[0] = BigEndianChar[3];
}
#endif
//static int CAN_throttle_A_val;
static int CAN_throttle_A_cnt;
//static int CAN_throttle_B_val;
static int CAN_throttle_B_cnt;
static int CAN_throttle;
static int CAN_unknown_id_cnt;
static int CAN_unknown_data_type;

static CAN_MSG_T CAN_rcv_msg;
static CAN_MSG_T CAN_snd_msg;
CAN_MSG_T CAN_rcv_command_msg;
CAN_MSG_T CAN_snd_status_msg;
//------------------------------------------------------------------------------------------
static void Load_F4()
{

    float *fp;

// Not sure if I have to reverse bytes for FP.

    BigEndianChar[0] = CA.Data[3];
    BigEndianChar[1] = CA.Data[2];
    BigEndianChar[2] = CA.Data[1];
    BigEndianChar[3] = CA.Data[0];
    fp = (float *)BigEndianChar;
    CA_F4 = *fp;
}
//------------------------------------------------------------------------------------------
bool LoadCheck_F4()
{
    if(CA.DataType != 2) {
    	CAN_unknown_data_type++;
//    	MHDEBUGOUT("Unexpected DataType: %d, should be 2\n\r",CA.DataType);
    	return false;
    }
    Load_F4();
    return true;
}
//------------------------------------------------------------------------------------------
void CAN_process_rotax_message(void)
{
	switch(CAN_rcv_msg.ID)
	{
	default:
		CAN_unknown_id_cnt++;
		return;

	case CA_THROTTLE_A:
		CAN_throttle_A_cnt++;
		break;

	case CA_THROTTLE_B:
		CAN_throttle_B_cnt++;
		break;
	}
	memcpy((char *)&CA,(char *)CAN_rcv_msg.Data,8);
	if(LoadCheck_F4())
	{
		CAN_throttle = CA_F4;
	}
}
//------------------------------------------------------------------------------------------
int CanInterrupts;
bool CAN_new_command=false;
void CAN_process_rc_message(void)
{
//	if(CanInterrupts % 10 == 0)
	{
//		PRINTF("id:%d\r\n",CAN_rcv_msg.ID);
/*
 *  We may get messages from other propellers, not just commands.
 *  Need a way to save commands and ignore responses. Simplest to do by can ID. Commands in 500 to 600 range
 *  Responses in 700 to 800 range. That way all controllers can ignore responses from other controllers.
 *  Each response CAN id will need to include the propeller number, otherwise we will have duplicate ids.
 */
/*
 *       CAN_transmit_rec.ID = CAN_ID_MOVE_FINE;
      CAN_transmit_rec.Data[0] = CAN_hub_transmit_id;
      int tenths = int.Parse(tb_move_fine_tenths.Text);
      CAN_transmit_rec.Data[1] = (byte)tenths;
      CAN_transmit_rec.Data[2] = CAN_motor_speed;
      CAN_transmit_rec.DLC = 3;   // Data length = 3
 */
		if(CAN_rcv_msg.ID >= 500 && CAN_rcv_msg.ID < 600)
		{
			uint8_t prop_num = CAN_rcv_msg.Data[0];
			if(prop_num == 0 || prop_num == ps.parms[RC_PROPNUM])
			{
				CAN_rcv_command_msg = CAN_rcv_msg;		// Not sure if this will work, may need to memcpy.
				CAN_new_command = true;
			}
		}

	}
}
//------------------------------------------------------------------------------------------
void CAN_convert_to_serial(void);
static void CAN_process_message(void)
{
	switch(CAN_mode)
	{
	case CAN_MODE_ROTAX:
		CAN_process_rotax_message();
		break;
	case CAN_MODE_RC:
		CAN_process_rc_message();
		break;
	case CAN_MODE_SERIAL:
		CAN_convert_to_serial();
		break;
	}
}
//------------------------------------------------------------------------------------------
void CAN_IRQHandler(void)
{
	uint32_t IntStatus;
//	CAN_MSG_T CAN_rcv_msg;


	CanInterrupts++;
//	if((CanInterrupts % 1000) == 0) MHDEBUGOUT(" %d",CanInterrupts);
//	 Board_UARTPutSTR("X");
	IntStatus = Chip_CAN_GetIntStatus(LPC_CAN);

	PrintCANErrorInfo(IntStatus);

	if(CAN_error_code)
	{
		CAN_error_count++;
		PRINTF("E=%d\r\n",CAN_error_code);
	}
	/* New Message came */
	if (IntStatus & CAN_ICR_RI) {
		if(Chip_CAN_Receive(LPC_CAN, &CAN_rcv_msg))
		{
			CAN_process_message();
		}

//		CanAddRingMsg(&Can2RB,&CAN_rcv_msg);

// Note: Could toggle LED

// Note: We will need to know which CAN controller msg came from.
// Is that info within CAN_rcv_msg? - No, but probably can get from interrupt status
// which is specific to a CAN controller.

//		MHDEBUGOUT("Message Received!!!\r\n");
//		PrintCANMsg(&CAN_rcv_msg);

//#define MH_REPLY
#ifdef MH_REPLY
		if (CAN_rcv_msg.Type & CAN_REMOTE_MSG) {
			ReplyRemoteMessage(&CAN_rcv_msg);
		}
		else {
			ReplyNormalMessage(&CAN_rcv_msg);
		}
#endif
	}
}
//------------------------------------------------------------------------------
#ifdef MH_CAN_RB
static void SerialOutput1(void)
{
	static int CAN_cnt;
	CAN_cnt++;
	if(CAN_cnt % 50 == 0)
	{
		PRINTF("CAN_cnt:%d, CAN_throttle:%d\r\n",CAN_cnt,CAN_throttle);
	}
}
//------------------------------------------------------------------------------
void AC210_CheckForCanMsg(void)
{
    while(CanRingMsgReady(&Can2RB))		// Could put a count to make sure doesn't get stuck here.
    {
//    	My_LED_Toggle(1);
    	CanGetRingMsg(&Can2RB,&CanRxMsg);
//    	My_LED_Toggle(3);
        SerialOutput1();
    }
}
#else
//------------------------------------------------------------------------------
void Display_can_throttle(void)
{
	static int CAN_cnt;
	CAN_cnt++;
	if(CAN_cnt % 50 == 0)
	{
		PRINTF("CAN_throttle:%d\r\n",CAN_throttle);
	}
}
#endif
//------------------------------------------------------------------------------
static bool CAN_init=false;
int CAN_mode;
void AC210_CAN_Init(int baudrate,int can_mode)
{
	CAN_init = true;
	if(CAN_mode != CAN_MODE_SERIAL)
	{
		CAN_mode = can_mode;
	}
//	Chip_CAN_DeInit(LPC_CAN);		// MHH:28/05/2025 Test ???

	AC210_Aux2_CAN_Init();
//	return ; // DEBUG!!!

	Chip_CAN_Init(LPC_CAN, LPC_CANAF, LPC_CANAF_RAM);
//	PRINTF("baudrate=%d\r\n",baudrate);
	Chip_CAN_SetBitRate(LPC_CAN, baudrate);
//	int status = Chip_CAN_SetBitRate(LPC_CAN, baudrate);
//	PRINTF("status=%d\r\n",status);
//	Chip_CAN_SetBitRate(LPC_CAN, 125000);
	Chip_CAN_EnableInt(LPC_CAN, CAN_IER_BITMASK);

//	SetupAFLUT();
	if(can_mode == CAN_MODE_ROTAX)
	{
//		PRINTF("CAN Rotax\r\n");
		Chip_CAN_SetAFLUT(LPC_CANAF, LPC_CANAF_RAM, &AFSections);
		Chip_CAN_SetAFMode(LPC_CANAF, CAN_AF_NORMAL_MODE);
	}
	else
	{
//		PRINTF("CAN RC\r\n");
		Chip_CAN_SetAFMode(LPC_CANAF, CAN_AF_BYBASS_MODE);	// Note: 'bypass' mis-spelt in header
	}

//	ChangeAFLUT();
//	PrintAFLUT();

//	Chip_CAN_SetAFMode(LPC_CANAF, CAN_AF_BYBASS_MODE);	// Note: 'bypass' mis-spelt in header
	NVIC_EnableIRQ(CAN_IRQn);
//	My_LED_Init();
//	My_LED_Set(1,1);
}
//------------------------------------------------------------------------------
typedef struct
{
	WORD chan_A_rate;
	WORD chan_B_rate;
	WORD tot_rate;
} CAN_stat_td;

CAN_stat_td CAN_stat;

WORD AC210_can_get_throttle(void)
{
//	static ULONG CAN_time_in_secs;

	if(CAN_init == false)
	{
		AC210_CAN_Init(125000,CAN_MODE_ROTAX);
		return 0;
	}

// Keep rate info in case there is a problem with CAN

	if(timerTick == 0)
//	if(CAN_time_in_secs != TimeInSeconds)
	{
//		CAN_time_in_secs = TimeInSeconds;
		CAN_stat.chan_A_rate = CAN_throttle_A_cnt;
		CAN_stat.chan_B_rate = CAN_throttle_B_cnt;
		CAN_stat.tot_rate = CAN_throttle_A_cnt + CAN_throttle_B_cnt;
		CAN_throttle_A_cnt = 0;
		CAN_throttle_B_cnt = 0;
	}

	if(CAN_stat.tot_rate <= 2)
	{
		return AUTO_THROTTLE_TIMEOUT;
	}
	//	Display_can_throttle();
	return CAN_throttle;
}
//====================================================================================================================
// CAN/Serial code
/*
CAN_data_to_send[0] = (byte)'C';    // Serial prefix
CAN_data_to_send[1] = (byte)'X';
CAN_data_to_send[2] = (byte)(CAN_transmit_rec.ID >> 8); // Bigendian
CAN_data_to_send[3] = (byte)(CAN_transmit_rec.ID & 0xff); // Bigendian
CAN_data_to_send[4] = (byte)(CAN_transmit_rec.DLC & 0xf); // Only
 *
 */
TIMER_t CAN_timer;
int CAN_send_timeouts;
int CAN_send_count=0;
int CAN_Send(void)
{
	CAN_BUFFER_ID_T   TxBuf;

	CAN_send_count++;
//	PRINTF("CAN_send_count:%d\r\n",CAN_send_count);	// MHH:29/05/2025. Debug only !!!

	for(;;)
	{
		TxBuf = Chip_CAN_GetFreeTxBuf(LPC_CAN);
		if(TxBuf < 3) break;
	}
	Chip_CAN_Send(LPC_CAN, TxBuf, &CAN_snd_msg);
	/*
	 * Note: For following line to work, there must be at least one other CAN node!!!!!
	 */
	Timer_reset(&CAN_timer);
	while ((Chip_CAN_GetStatus(LPC_CAN) & CAN_SR_TCS(TxBuf)) == 0)
	{
		int msecs = Timer_read_ms(&CAN_timer);
		if(msecs > 20)
		{
			CAN_send_timeouts++;
//			Chip_CAN_SetCmd(LPC_CAN, CAN_CMR_STB(TxBuf) | CAN_CMR_AT);	// Try aborting transmission
			return 0;
		}
	};
	return 1;
}
void CAN_send_init(void)
{

	int baudrate = ps.parms[RC_BAUDRATE];
	if(baudrate == 0)  baudrate = 125;
	baudrate *= 1000;
	AC210_CAN_Init(baudrate,CAN_MODE_RC);		// May need to differentiate from CANAero. Eg not setup FLUT


//	AC210_CAN_Init(125000,CAN_MODE_RC);
	Timer_start(&CAN_timer);
}
extern BYTE codeEntered;
int CANProcessCommand(char *rxbuff,int rxcount)
{
	if(CAN_init == false)
	{
		CAN_send_init();
	}

	CAN_snd_msg.ID  = rxbuff[2] << 8 | rxbuff[3];

	if(CAN_snd_msg.ID == 20000)	// Finished with CAN format?
	{
		S_Pkt.Mode = SERIAL_MODE_ASCII;
		S_Pkt.Pkt_Head_0 = 254;
		CAN_mode = CAN_MODE_RC;			// So we stop processing CAN data from AC300 controllers
		// Perhaps should send equivalent of RC_S=0 ??
		AC210_PC_ChangeBaud(19200);		// In case we were at 115.2k
		codeEntered = 0;				// Same as ATBYE
		return 2;		// So we can exit loop
	}

	CAN_snd_msg.DLC = rxbuff[4];
	CAN_snd_msg.Type = 0;		// Not CAN_REMOTE_MSG
	for(int i=0;i<CAN_snd_msg.DLC;i++)
	{
		CAN_snd_msg.Data[i] = rxbuff[5+i];
	}
	if(CAN_Send() == 0)
    {
 //       PRINTF("?CAN_Send fail\r\n");
		CAN_send_init();
		return 1;
    }
	return 0;
}
void CAN_send_status(void)
{
	CAN_snd_msg = CAN_snd_status_msg;
	if(CAN_Send() == 0)
    {
 //       PRINTF("?CAN_Send fail\r\n");
		CAN_send_init();
//		return 1;
    }

}
uint8_t CAN_txbuff[20];
void CAN_convert_to_serial(void)		// Here if we are converting CAN to serial
{
//	if(CAN_rcv_msg.ID < 700 || CAN_rcv_msg.ID >= 800) return;	// Only interested in CAN responses from AC300 controllers
	/*
	 * We want to see all CAN messages that we did not send
	 */

	CAN_txbuff[0] = 'C';
	CAN_txbuff[1] = 'R';
	CAN_txbuff[2] = CAN_rcv_msg.ID >> 8;
	CAN_txbuff[3] = CAN_rcv_msg.ID & 255;
	CAN_txbuff[4] = (uint8_t)CAN_rcv_msg.DLC;
	for(int i=0;i<CAN_rcv_msg.DLC;i++)
	{
		CAN_txbuff[5+i] = CAN_rcv_msg.Data[i];
	}
	int len = 5 + CAN_rcv_msg.DLC;
	for(int i=0;i<len;i++)		// Now send as serial message
	{
		PC_putc(CAN_txbuff[i]);
	}
}
