/*
 * junk.c
 *
 *  Created on: 19/03/2023
 *      Author: OEM
 */


#ifdef MH_JUNK

#ifdef CHIP_UART_MACROS
#define UART_WRITEABLE(puart)	(Chip_UART_ReadLineStatus(puart) & UART_LSR_THRE)
#define UART_PUTC(puart,c)		Chip_UART_SendByte(puart,c)
#define UART_READABLE(puart)	(Chip_UART_ReadLineStatus(puart) & UART_LSR_RDR)
#define UART_GETC(puart)		Chip_UART_ReadByte(puart)
#else
#define UART_WRITEABLE(puart)	(puart->FIFOLVL/256 < 16)
#define UART_NOT_WRITEABLE(puart)	(puart->FIFOLVL/256 == 16)
#define UART_PUTC(puart,c)		puart->THR = c
#define UART_READABLE(puart)	(puart->LSR & UART_LSR_RDR)
#define UART_GETC(puart)		(puart->RBR)
#endif



int UartFalseInts;
int UartSurpriseInts;
int UartBytesReceived;
int UartBytesTransmitted;
int LsrRDR,LsrTHR,LsrBoth;

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define SERIALBUFFIN
#define SBUFFSIZE_OUT_LARGE   2048	// When sending to AC200Diagnostics needs to handle large buffers
#define SBUFFSIZE_OUT   512		// MHH:20/08/2019
#define SBUFFSIZE_IN	512
//---------------------------------------------------------------------------------
typedef struct
{
    volatile int head;		// MHH: 15/03/2018. Added "volatile" as had a problem with mbed compiler serial(Encoder7) without volatile
    volatile int tail;
    volatile int busy;       // Only for output so no conflict with IRQ
    int buffsize;
    uint8_t *pbuff;
//    uint8_t buff[SBUFFSIZE_OUT];
} UartRingBuf_t;

uint8_t Uart0_buff_out[SBUFFSIZE_OUT_LARGE];
//uint8_t Uart0_buff_in[SBUFFSIZE_IN]; This is not being used because LPC_UART0 is sending input bytes to rxBuf

uint8_t Uart2_buff_out[SBUFFSIZE_OUT];
uint8_t Uart2_buff_in[SBUFFSIZE_IN];

uint8_t Uart3_buff_out[SBUFFSIZE_OUT];
uint8_t Uart3_buff_in[SBUFFSIZE_IN];


//UartRingBuf_t Uart1InputRB,Uart1OutputRB;
//UartRingBuf_t Uart2InputRB,Uart2OutputRB;
//UartRingBuf_t Uart0InputRB,Uart0OutputRB;
UartRingBuf_t Uart0OutputRB,Uart0InputRB;
UartRingBuf_t Uart2OutputRB,Uart2InputRB;
UartRingBuf_t Uart3OutputRB,Uart3InputRB;



//-------------------------------------------------------------------------------
#ifdef MH_DISPLAY_RB
static void DisplayUartRingBufferData(int iu,const char *name,UartRingBuf_t *urb)
{
    printf("%s Buffer (%d):\n\r",name,iu);
    printf("  Head: %d\n\r",urb->head);
    printf("  Tail: %d\n\r",urb->tail);
    printf("  Buffsize: %d\r\n",urb->buffsize);
    int buffsize = urb->buffsize;
    uint8_t *buff = urb->pbuff;
    for(int i=0;i<buffsize;i++){
        if(i%16 == 0) printf("\r\n%3d ",i);
        printf(" %02x",buff[i]);
    }
    printf("\n\r");
}
#endif
//-----------------------------------------------------------------------------
#ifndef MH_TRY_CHIP_RB


static void UartRingBuffError(const char *emsg,UartRingBuf_t *urb)
{
    char *msg = (char *)"Uart:Unknown";
    char amsg[100];
    if(urb == &Uart0InputRB) msg = (char *)"Uart0Input";
    if(urb == &Uart0OutputRB) msg = (char *)"Uart0Output";
    sprintf(amsg,"UartRingBuffError:%s: %s\n\r",emsg, msg);
    Abort(AC210_SRC_COMMS+10,amsg);
}
//-----------------------------------------------------------------------------
static int UartRingBytesReady(UartRingBuf_t *urb)
{

    int head = urb->head;
    int tail = urb->tail;
    int buffsize = urb->buffsize;
    if(head < tail) head+= buffsize;
    return head-tail;
}
//-----------------------------------------------------------------------------
static int UartRingSpaceAvail(UartRingBuf_t *urb)
{
	return (urb->buffsize - UartRingBytesReady(urb));
}
#endif
//----------------------------------------------------------------------------------
#ifdef MH_COPY_FROM_RB
static void UartCopyFromRingBuffer(UartRingBuf_t *urb,uint8_t *ubp,int ilen)
{
    int head = urb->head;
    int tail = urb->tail;
    int buffsize = urb->buffsize;
    if(head < tail) head+= buffsize;
    int bytes_ready = head-tail;

    if(ilen > bytes_ready) {
        UartRingBuffError("UartCopyFromRingBuffer:Not enough bytes ready",urb);
    }
// May need to break copy into 2 parts if it wraps around buffer

    int len1 = buffsize - tail;
    if(len1 > ilen) len1 = ilen;
    memcpy(ubp,urb->pbuff+tail,len1);
    int len2 = ilen - len1;     // len2 is remainder, if any
    if(len2) {
        memcpy(ubp+len1,urb->pbuff,len2);
    }
//    urb->tail = (tail+ilen) & SBUFFMASK;
    urb->tail = (tail+ilen) % buffsize;
}
#endif


//-----------------------------------------------------------------------------
void AC210_uart0_wait(void)
{
#ifdef MH_TRY_CHIP_RB
	while(RingBuffer_IsEmpty(&U0_rxring) == false);
#else
	while(Uart0OutputRB.tail != Uart0OutputRB.head);	// Wait for ISR to catch up
#endif
}
//-----------------------------------------------------------------------------
void AC210_uart3_wait(void)
{
#ifdef MH_TRY_CHIP_RB
	while(RingBuffer_IsEmpty(&U3_rxring) == false);
#else
    while(Uart3OutputRB.tail != Uart3OutputRB.head);	// Wait for ISR to catch up
#endif
}
//-----------------------------------------------------------------------------
#ifndef MH_TRY_CHIP_RB
static void UartAddRingByte(UartRingBuf_t *urb,uint8_t uc)
{
    int head = urb->head;
    urb->pbuff[head] = uc;
    head = (head+1) % urb->buffsize;

#ifdef MH_OLD_RING_ADD
    if(head != urb->tail)
    {
    //    if(head != urb->tail) {
        urb->head = head;
        return;
    }
// If we drop through then buffer is full.
    UartRingBuffError("UartAddRingByte:Buffer full",urb);
#else
    while(head == urb->tail);	// Wait for ISR to catch up. Could put count in here?
    // Note: 16/11/2022. If this routine is ever called from a high priority ISR then it can hang, as UART ISR not called.
    // Alternative is to wait for UART to be ready to transmit, then send one byte from tail and update tail before updating head.
    // Need to be sure that UART ISR does not interfere by disabling interrupts for this UART first. Or set "busy" flag.
    urb->head = head;
#endif
}
//-----------------------------------------------------------------------------
uint32_t rx_Time_last_us;		// SIG60 routines need to know when last byte received
uint32_t rx_Time_start_us;
static void Uart_ISR_AddRingByte(UartRingBuf_t *urb,uint8_t uc)
{
#ifdef MH_RX_TIMER
	uint32_t time_us = us_ticker_read();
	if((time_us - rx_Time_last_us) > 10000)	// 10 ms
	{
		rx_Time_start_us = time_us;
	}
	rx_Time_last_us = time_us;
#endif
	int head = urb->head;
    urb->pbuff[head] = uc;
    head = (head+1) % urb->buffsize;

    if(head != urb->tail)
    {
    //    if(head != urb->tail) {
        urb->head = head;
        return;
    }

    // If we drop through then buffer is full.
//    UartRingBuffError("UartAddRingByte:Buffer full",urb);
}
#endif


#ifndef MH_TRY_CHIP_RB

void rxProcessChar(BYTE ch);
extern bool Sig100_hub_link;
static void UartInputIRQRingBuffer(UartRingBuf_t *urb,LPC_USART_T *pUART)
{
    int rcnt = 0;
//    while(pUART->readable()) {
    while(UART_READABLE(pUART)) {
//        uint8_t c = pUART->getc();
        uint8_t c = UART_GETC(pUART);
// Note: Could get all bytes ready and add in 1 go
        UartBytesReceived++;
        if(pUART == LPC_UART0)	// Special case
        {
        	rxProcessChar(c);		// In comms.c
        }
        else
        {
        	if(pUART == LPC_UART3 && Sig100_hub_link)
        	{
            	PC_putc((char) c);	// MHH:15/12/2022 Special logic to allow data to "pass through AC200 for Flash write routines in HubProgrammer
        	}
        	else
        	{
            	Uart_ISR_AddRingByte(urb,c);	// MHH:18/12/2017
        	}
        }
        rcnt++;
//    	if(rcnt == 8) return;	// Try batches of 8??
    }
}
//-----------------------------------------------------------------------------
static int UartGetRingByte(UartRingBuf_t *urb)
{
    int tail = urb->tail;
    if(urb->head != tail) {
        int c = urb->pbuff[tail];
//        urb->tail = (tail+1) & SBUFFMASK;
        urb->tail = (tail+1) % urb->buffsize;
        return c;
    }
// If we drop through then nothing available
    return -1;
}
//==========================================================================
static void UartOutputRingBuffer(UartRingBuf_t *urb,LPC_USART_T *pUART)
{
    int tail = urb->tail;
    while(urb->head != tail) {
//        if(pUART->writeable()== false) return;
        if(UART_WRITEABLE(pUART) == false) return;
        uint8_t c = urb->pbuff[tail];
//        pUART->putc(c);
        UART_PUTC(pUART,c);
        UartBytesTransmitted++;
        tail = (tail+1) % urb->buffsize;    // Note: tail is used in while test!
        urb->tail = tail;
    }
}
//-------------------------------------------------------------------------
#ifdef MH_UART_SPEEDTEST
int UartTimeCount;
int UartTimeCount2;
void UartTestSpeed(LPC_USART_T *pUART)
{
	int i;
	int c = 'a';
	uint32_t t1,t2,t3;

	t1 = us_ticker_read();
	for(i=0;i<16;i++) {
		while(UART_WRITEABLE(pUART) == false) UartTimeCount++;
        UART_PUTC(pUART,c);
	}
	t2 = us_ticker_read();
	for(i=0;i<1024;i++) {
		while(UART_WRITEABLE(pUART) == false) UartTimeCount2++;
        UART_PUTC(pUART,c);
	}
	t3 = us_ticker_read();
	printf("UartTestSpeed1: usecs/16 = %d\n\r",(t2-t1));
	printf("UartTestSpeed2: usecs/16 = %d\n\r",(t3-t2)/64);
	printf("UartTimeCount: %d\n\r",UartTimeCount);
//	exit(0);
}
#endif
//-------------------------------------------------------------------------
static void UartOutputIRQRingBuffer(UartRingBuf_t *urb,LPC_USART_T *pUART)
{
    if(urb->busy) return;
    if(urb->head == urb->tail) return;  // Anything in buffer?
    UartOutputRingBuffer(urb,pUART);      // yes, output
}
//-------------------------------------------------------------------------
static void UartPutcRingBuffer(UartRingBuf_t *urb,LPC_USART_T *pUART,uint8_t uc)
{
	/* Don't let UART transmit ring buffer change in the UART IRQ handler */
	Chip_UART_IntDisable(pUART, UART_IER_THREINT);	// MHH:15/03/2023

	UartAddRingByte(urb,uc);
    urb->busy = true;
    UartOutputRingBuffer(urb,pUART);
    urb->busy = false;


	/* Enable UART transmit interrupt */
	Chip_UART_IntEnable(pUART, UART_IER_THREINT);	// MHH:15/03/2023

}
//-------------------------------------------------------------------------
static bool UartReadable(UartRingBuf_t *urb)
{
    return (urb->head != urb->tail);
}

//----------------------------------------------------------------------------------
#ifdef MH_EEE
void UartOutputString(UartRingBuf_t *urb,LPC_USART_T *pUART,uint8_t* pString)
{
// Not meant to be efficient, just want to see if it works....
	int i,maxspace;
	uint8_t uc;

	maxspace = UartRingSpaceAvail(urb);
	for(i=0;i<maxspace;i++) {
		uc = pString[i];
		UartAddRingByte(urb,uc);
		if(uc == 0) {
			break;
		}
	}
    urb->busy = true;
    UartOutputRingBuffer(urb,pUART);
    urb->busy = false;
}
#endif
//----------------------------------------------------------------------------------
#ifndef MH_TRY_CHIP_RB
void UartCopyToRingBuffer(UartRingBuf_t *urb,LPC_USART_T *pUART,const uint8_t *ubp,int ilen)
{


	int head = urb->head;
    int tail = urb->tail;
    int buffsize = urb->buffsize;
    if(head >= tail) tail += buffsize;
    int bytes_avail = tail - head;
    while(ilen >= bytes_avail)	// MHH:21/12/2016. QuickFix to stop buffer overflow
    {
        head = urb->head;
        tail = urb->tail;
        if(head >= tail) tail += buffsize;
        bytes_avail = tail - head;
    }
    if(ilen >= bytes_avail) {
        UartRingBuffError("UartCopyToRingBuffer:Buffer full",urb);
    }
// May need to break copy into 2 parts if it wraps around buffer

    int len1 = buffsize - head;
    if(len1 > ilen) len1 = ilen;
    memcpy(urb->pbuff+head,ubp,len1);
    int len2 = ilen - len1;     // len2 is remainder, if any
    if(len2) {
        memcpy(urb->pbuff,ubp+len1,len2);
    }
//    urb->head = (head+ilen) & SBUFFMASK;
    urb->head = (head+ilen) % buffsize;
    urb->busy = true;
    UartOutputRingBuffer(urb,pUART);
    urb->busy = false;
}
#endif
//------------------------------------------------------------------------------------------------
#ifdef MH_DEFINE_UART2
int IntRLS,IntRDA,IntCTI,IntTHR,IntXXX;
int LsrRDR,LsrTHR,LSRBoth;
void UART2_IRQHandler(void)
{
	uint32_t IIR = LPC_UART2->IIR;
	uint32_t LSR = LPC_UART2->LSR;
	if(IIR & 1) {
		UartFalseInts++;
		if(LSR & (1 | (1<<5))) UartSurpriseInts++;
//		printf("UART2_IRQHandler: No interrupt\n\r");
//		exit(1);
	}
	uint32_t Intld = ((IIR >> 1) & 0b111);
	switch(Intld) {
	case 0b011:
		IntRLS++;
		break;
	case 0b010:
		IntRDA++;
		break;
	case 0b110:
		IntCTI++;
		break;
	case 0b001:
		IntTHR++;
		break;
	default:
		IntXXX++;
		break;
	}
	if((LSR & 1) && (LSR & (1<<5))) LsrBoth++;
//	if(LSR & 1) {
	if(Intld == 2) {	// Try only servicing int if interrupt RDA signalled (to get 8 in 1 go)
	    LsrRDR++;
		UartInputIRQRingBuffer(&Uart1InputRB,LPC_UART2);
	}
	if(UART_WRITEABLE(LPC_UART2)) {
//	if(LSR & (1<<5)) {
		LsrTHR++;
		UartOutputIRQRingBuffer(&Uart1OutputRB,LPC_UART2);
	}
}






//	UART_ClearInterrupts(LPC_UART3);
#define MH_OLD_CLEAR_ERRORS
#ifdef MH_OLD_CLEAR_ERRORS
uint32_t IIR = LPC_UART2->IIR;
UART2_LSR = LPC_UART2->LSR;
if(IIR & 1)		// No interrupt?
{
	NVIC_ClearPendingIRQ(UART2_IRQn);
}
uint32_t int_id = ((IIR >> 1) & 7);

if((LPC_UART2->LSR & 1) == 0)
{
	if(int_id == 6)		// Character timeout?
	{
		UART2_RBR = LPC_UART2->RBR;		// Clear cause of interrupt
	}
}
#else	// MHH:04/07/2018
UART_ClearInterrupts(LPC_UART2);
#endif
UartInputIRQRingBuffer(&Uart2InputRB,LPC_UART2);
UartOutputIRQRingBuffer(&Uart2OutputRB,LPC_UART2);
#endif
#endif


//	UART_ClearInterrupts(LPC_UART3);
#define MH_OLD_CLEAR_ERRORS
#ifdef MH_OLD_CLEAR_ERRORS
	uint32_t IIR = LPC_UART3->IIR;
	UART3_LSR = LPC_UART3->LSR;
	if(IIR & 1)		// No interrupt?
	{
		NVIC_ClearPendingIRQ(UART3_IRQn);
	}
	uint32_t int_id = ((IIR >> 1) & 7);

	if((LPC_UART3->LSR & 1) == 0)
	{
		if(int_id == 6)		// Character timeout?
		{
			UART3_RBR = LPC_UART3->RBR;		// Clear cause of interrupt
		}
	}
/*
	if((IIR >> 1) == 3)	// Receive Line Status?
	{
		uint32_t LSR = LPC_UART3->LSR;
		if((LSR & 0x1f) != 0)	// Any receive errors?
		{
		    LPC_UART3->FCR |= (UART_FCR_RX_RS | UART_FCR_TX_RS);	// MHH:03/07/2018 Reset FIFO RX and TX
		}
		return;
	}
*/
#else	// MHH:04/07/2018
	UART_ClearInterrupts(LPC_UART3);
#endif
	UartInputIRQRingBuffer(&Uart3InputRB,LPC_UART3);
    UartOutputIRQRingBuffer(&Uart3OutputRB,LPC_UART3);


#define MH_DEFINE_UART3
#ifdef MH_DEFINE_UART3
uint32_t UART3_LSR;
uint32_t UART3_RBR;
uint32_t UART3_IIR;
void UART3_IRQHandler(void)
{
#ifdef MH_TRY_CHIP_RB
	Chip_UART_IRQRBHandler(LPC_UART3, &U3_rxring, &U3_txring);
#else
#endif
}

//------------------------------------------------------------------------------------------------
#define LSR_RXFE				(1<<7)
#define FCR_RX_FIFO_RESET		(1<<1)

#ifdef MH_OLD_CLEAR_ERRORS
uint32_t UART_ClearInterrupts(LPC_USART_T *pUART)
{
	uint32_t IIR = pUART->IIR;
	uint32_t LSR = pUART->LSR;

	if(LSR & LSR_RXFE)		// MHH:16/03/2018 FIFO RX Framing error?
	{
		pUART->FCR |= FCR_RX_FIFO_RESET;	// then reset
	}
	return (IIR+LSR);		// Nonsensical, but want to force read of both registers to clear pending interrupts
}
#else	// MHH:04/07/2018

void UART_ClearInterrupts(LPC_USART_T *pUART)
{
	uint32_t IIR = pUART->IIR;
	uint32_t LSR = pUART->LSR;
	if(IIR & 1)
	{

	}
	if((IIR & 7) == 6)	// Receive Line Status?
	{
//		uint32_t LSR = pUART->LSR;

		if((LSR & 0x1f) != 0)	// Any receive errors?
		{
//			pUART->FCR |= (UART_FCR_RX_RS | UART_FCR_TX_RS);	// MHH:03/07/2018 Reset FIFO RX and TX
			pUART->FCR |= UART_FCR_RX_RS;	// MHH:04/07/2018 Reset FIFO RX
		}
	}
}

//	UART_ClearInterrupts(LPC_UART0);
	uint32_t UART0_IIR = LPC_UART0->IIR;
	UART0_LSR = LPC_UART0->LSR;
//	if(IIR & 1)		// No interrupt?	// MHH:05/12/2022. Saw example of IIR with bit 0 = 0, but under debug it showed interrupt pending!!
	{
		NVIC_ClearPendingIRQ(UART0_IRQn);
	}
    UartInputIRQRingBuffer(&Uart0InputRB,LPC_UART0);
    UartOutputIRQRingBuffer(&Uart0OutputRB,LPC_UART0);
#endif
#endif

    int Uart3_count;
    uint8_t Uart3_ch;
    void Try_reading_from_uart3(void)
    {
    	while (Chip_UART_ReadLineStatus(LPC_UART3) & UART_LSR_RDR)	// MHH:17/03/2023. Modify to allow linking...
    	{
    		uint8_t ch = Chip_UART_ReadByte(LPC_UART3);
    		Uart3_ch = ch;
    		Uart3_count++;
    	}
    }

    //#define MH_UART3_AND_SIG60
    #ifdef MH_UART3_AND_SIG60	// MHH:24/11/2020. May have had problem because of initial UART3 baud = 115,200, SIG60 = 19200.
    	/* Enable receive data and line status interrupt  AND TRANSMIT!!!*/
    	if(n == 3)		// MHH: 03/07/2018. UART3 causing AC210 to hang when connected to active SIG60 but not params not set for positioning.
    	{
    		Chip_UART_IntEnable(lpc_uart, UART_IER_THREINT);	// Transmit only interrupt to start with for AUX serial
    	}
    	else	// MHH:21/08/2019. Here for UART0 and UART2. Note: May only use RX for UART2.
    	{
    		Chip_UART_IntEnable(lpc_uart, (UART_IER_RBRINT | UART_IER_RLSINT | UART_IER_THREINT));
    	}
    #else
    //	Chip_UART_IntEnable(lpc_uart, (UART_IER_RBRINT));
    	Chip_UART_IntEnable(lpc_uart, (UART_IER_RBRINT | UART_IER_RLSINT));
    //	Chip_UART_IntEnable(lpc_uart, (UART_IER_RBRINT | UART_IER_RLSINT | UART_IER_THREINT));
    #endif

    	Uart3OutputRB.tail = Uart3OutputRB.head = 0;
    	Uart3InputRB.tail = Uart3InputRB.head = 0;
    	Uart0OutputRB.buffsize = SBUFFSIZE_OUT_LARGE;
    	Uart0OutputRB.pbuff = Uart0_buff_out;

    	Uart0InputRB.buffsize = SBUFFSIZE_IN;
    	//	Uart0InputRB.pbuff = Uart0_buff_in;	// This is not being used because LPC_UART0 sending bytes to rxBuf

    	Uart2OutputRB.buffsize = SBUFFSIZE_OUT;
    	Uart2OutputRB.pbuff = Uart2_buff_out;

    	Uart2InputRB.buffsize = SBUFFSIZE_IN;
    	Uart2InputRB.pbuff = Uart2_buff_in;

    	Uart3OutputRB.buffsize = SBUFFSIZE_OUT;
    	Uart3OutputRB.pbuff = Uart3_buff_out;

    	Uart3InputRB.buffsize = SBUFFSIZE_IN;
    	Uart3InputRB.pbuff = Uart3_buff_in;
    	//----------------------------------------------------------------------
    	#ifdef MH_P_GET_SERIAL		// Not needed at present
    	BYTE p_GetSerial(BYTE *prxCount,char *rxBuf)
    	{
    	// Copy to rxBuf until we get a newline character, or no more
    	// If we do get a new line character, return 1, else 0
    		BYTE rxcnt = *prxCount;
    		int c;

    	#ifdef XOAR_VERSION
    		if(S_Pkt.Mode != SERIAL_MODE_ASCII) {
    			return Serial_GetBinary(prxCount,rxBuf);
    		}
    	#endif

    		while(true) {
    			c = UartGetRingByte(&Uart0InputRB);
    			if(c < 0) break;	// Anything in buffer?

    			if(rxcnt >= RX_BUF_SIZE-1) {
    				Abort("p_GetSerial:Overflow");
    			}
    	#ifdef XOAR_VERSION
    	// Note: We could generalise the escape character and logic if we end up with more than one other remote mode.
    			if(c == S_Pkt.Pkt_Head_0) {
    				if(RemoteCommsType == REMOTE_COMMS_XOAR) {
    					rxcnt = 0;		// Start of pkt
    					rxBuf[rxcnt++] = c;
    					S_Pkt.Mode = SERIAL_MODE_XOAR;
    					S_Pkt.Pkt_TimeoutVal = 3;	// Decremented every cycle
    					S_Pkt.Mode_TimeoutVal = 150;	// 150 cycles = 3 secs
    					return Serial_GetBinary(prxCount,rxBuf);
    				}
    			}
    	#endif
    			rxBuf[rxcnt++] = c;
    			if(c == 10) break;	// LF?
    		}
    		*prxCount = rxcnt;		// Update ptr
    		if(c == 10){
    			rxBuf[rxcnt+1] = 0;		// null terminate
    			printf("<< %s\r",rxBuf);
    			return 1;
    		}
    		return 0;
    	}
    	#endif
    	//void UartCopyToRingBuffer(UartRingBuf_t *urb,LPC_USART_T *pUART,const uint8_t *ubp,int ilen)
        UartCopyToRingBuffer(&Uart0OutputRB,LPC_UART0,(uint8_t *)buf,(int)numChar);
        Debug_SendStringLen(buf,numChar);
        //-------------------------------------------------------------------------
        #ifdef MH_AAA
        float fBladeAngle(float fPos)
        {
        // fPos is the distance in mm (+ or -) from the ANGLE_TOP position

            float result,radians,degrees;

            float sine_val = (fPos/CAM_RADIUS);

        #ifdef MH_XXX		// MHH:07/02/2023
            if(sine_val <= -1.0)
            {
            	return -90.0;		// Maximum negative value
            }
            if(sine_val >= 1.0)
            {
            	return 90.0;		// maximum positive value
            }
        #else
            if(sine_val < -1.0) sine_val = -1;
            if(sine_val > 1.0) sine_val = 1;
        #endif
            radians = asinf(sine_val);
            degrees = radians * 180.0 / PI;
            result = ANGLE_TOP + degrees;
            return result;
        }
        //-------------------------------------------------------------------------
        float fBladePos(float fdegrees)     // Reverse of fBladeAngle
        {
        // fPos is the distance in mm (+ or -) from the ANGLE_TOP position

            float radians,degrees;
            float fpos,pos_over_radius;

            degrees = fdegrees - ANGLE_TOP;     // relative to ANGLE_TOP
            radians = degrees * PI / 180.0;
        //    pos_over_radius = sin(radians);
            pos_over_radius = sinf(radians);
            fpos = pos_over_radius * CAM_RADIUS;
            return fpos;
        }
        //-------------------------------------------------------------------------
        float Encoder_GetBladeAngle(int encoder_pos_from_fine)
        {
            float screw_pos,pos;
            float blade_angle;
            float motor_pos;
        //    float rv;
            int motor_gear_ratio;

        //    int iangle;
        //    int encoder_pos;     // As the ISR could change Encoder_Pos while we are printing.

        //    encoder_pos    = Encoder_Pos;
        //    encoder_pos -= Encoder_zero_offset;         // strip away zero offset
            int encoder_pos = encoder_pos_from_fine + hwf.PO_fine_offset;
            motor_pos = (float)encoder_pos / ENCODER_INCREMENTS;
            motor_gear_ratio = getParameter(PO_GEAR_RATIO);
            screw_pos = motor_pos / motor_gear_ratio;
            pos       = screw_pos * MM_PER_TURN;
            blade_angle = fBladeAngle(pos);
            return blade_angle;
        // To 1 dec place
        /*

            iangle = (int) (fabs(blade_angle) * 10 + 0.5);
            rv = iangle;	// Convert from integer
            rv /= 10;
            if(blade_angle < 0) rv = -rv;
            return rv;
        */
        }
        #endif
#ifdef MH_AAA
//------------------------------------------------------------------------------
int Enc_GetBladeAngle(int encoder_pos)
{
	float f_angle = Encoder_GetBladeAngle(encoder_pos);
	f_angle *= 10;		// convert to integer with an implied decimal place
//	int angle = (int)f_angle;		// Could have added 0.5
	int angle = iround(f_angle);
	return angle;
}
#endif
//------------------------------------------------------------------------------
#ifdef MH_AAA
//float fEnc_fine_stop_angle=15;
#define ENC_DEF_FINE_STOP	15
#define ENC_DEF_COARSE_STOP	35

int Enc_pos_from_angle(float angle)
{
    float screw_pos,pos;
    float motor_pos;
    float encoder_pos;
    int enc_pos;
    int motor_gear_ratio;

    pos  = fBladePos(angle);

    // Now to translate mm to turns of motor

	screw_pos = pos / MM_PER_TURN;
    motor_gear_ratio = getParameter(PO_GEAR_RATIO);
	motor_pos = screw_pos * motor_gear_ratio;
	encoder_pos = motor_pos * ENCODER_INCREMENTS;
	enc_pos = iround(encoder_pos);
//	enc_pos = (int)(encoder_pos + 0.5);
// Because when SIG60 sends P=0 for fine stop, we want it adjusted to enc_pos
	return enc_pos;
}
#endif
//------------------------------------------------------------------------------
#ifdef MH_AAA
void Enc_Angle_Init(void)
{
    int enc_pos;


//    hwf.PO_fine_offset = -1;		// Test!!
    if(hwf.PO_fine_offset == -1)		// First time?
    {
    	enc_pos = Enc_pos_from_angle(ENC_DEF_FINE_STOP);
    // Because when SIG60 sends P=0 for fine stop, we want it adjusted to enc_pos
    //	hwf.PO_adjust_pos = -enc_pos;
    	hwf.PO_fine_offset = enc_pos;
    	enc_pos = Enc_pos_from_angle(ENC_DEF_COARSE_STOP);
    	hwf.PO_coarse_stop = enc_pos - hwf.PO_fine_offset;
//    	hwf.PO_coarse_stop = enc_pos;
    	POS_write_param();		// Save default
    }
}
#endif
#ifdef MH_AAA
#define ENCODER_INCREMENTS			2		// 2 increments per turn of motor
//#define ENCODER_INCREMENTS			1		// MHH:07/07/2018. increments per turn of motor

#define TEETH_PER_INCH    8.0
#define CAM_RADIUS       26.0         // mm
#define MM_PER_INCH      25.4

#define ANGLE_TOP               30.0         // degrees

#define MM_PER_TURN   (MM_PER_INCH/TEETH_PER_INCH)
#endif

#ifdef MH_CCC
	case CAL_START:
		PRINTF("Calibrate:Moving to FINE stop,pos:%d\r\n",Encoder_Pos);
		Hub_calibrate_state = CAL_FINE_STOP;

	case CAL_FINE_STOP:
		// First go to fine stop....
		if(currentState & S_STOP_FINE)
		{
			Hub_calibrate_state = CAL_COARSE_HARDSTOP;
			Cal_fine_pos = Encoder_Pos;
			Cal_current_count = 0;
			PRINTF("Calibrate:Reached FINE stop, moving to COARSE hard stop,pos:%d\r\n",Encoder_Pos);
			PRINTF("CAL:f=%d\r\n",Cal_fine_pos);	// Was 'X' but want to use that for finish
		}
		Hub_return_pos();
		return;
#else
	case CAL_START:
		PRINTF("Calibrate:Moving to COARSE hard stop,pos:%d\r\n",Encoder_Pos);
		Cal_fine_ms_stop_found = false;
		Hub_calibrate_state = CAL_COARSE_HARDSTOP;
		Cal_current_count = 0;
		Hub_return_pos();
		return;

#endif

#ifdef MH_XXX
	case CAL_COARSE_HARDSTOP_FINE:
		if(Cal_current_count++ > 100)
		{
			PRINTF("Calibrate:Moving to COARSE hard stop again,pos:%d,Cal_cur:%d,cur:%d\r\n",Encoder_Pos,Cal_current,current);
			Hub_calibrate_state = CAL_COARSE_HARDSTOP_SLOW;
			Cal_current_count = 0;
		}
		return;

	case CAL_COARSE_HARDSTOP_SLOW:
		current = scaledValue (A_MOTOR_CURRENT);
		if(Cal_current_count < 50)	// Give a sec to start
		{
			Cal_current_count++;
			Cal_current = current;
		}
		current = scaledValue (A_MOTOR_CURRENT);
		if(current > (Cal_current*3)/2)		// Arbitrary - could be 50 % more
		{
			Cal_coarse_hard_pos = Encoder_Pos;
			Hub_calibrate_state = CAL_FINE_HARDSTOP;
			Cal_current_count = 0;
			PRINTF("Calibrate:Reached COARSE hard stop, moving to FINE hard stop,pos:%d,Cal_cur:%d,cur:%d\r\n",Encoder_Pos,Cal_current,current);
		}
		return;
#endif
#ifdef MH_XXX
	case CAL_FINE_HARDSTOP_COARSE:
		if(Cal_current_count++ > 100)
		{
			PRINTF("Calibrate:Moving to FINE hard stop again,pos:%d,Cal_cur:%d,cur:%d\r\n",Encoder_Pos,Cal_current,current);
			Hub_calibrate_state = CAL_FINE_HARDSTOP_SLOW;
			Cal_current_count = 0;
		}
		return;

	case CAL_FINE_HARDSTOP_SLOW:
		current = scaledValue (A_MOTOR_CURRENT);
		if(Cal_current_count < 50)	// Give a sec to start
		{
			Cal_current_count++;
			Cal_current = current;
		}
		current = scaledValue (A_MOTOR_CURRENT);
		if(current > (Cal_current*3)/2)		// Arbitrary - could be 50 % more
		{
			Cal_fine_hard_pos = Encoder_Pos;
			Hub_calibrate_state = CAL_RETURN_FINE_STOP;
			PRINTF("Calibrate:Reached FINE hard stop, moving COARSE,pos:%d,Cal_cur:%d,cur:%d\r\n",Encoder_Pos,Cal_current,current);
		}
		return;
#endif
		//---------------------------------------------------------------------------------------------------
		#ifdef MH_DDD
		void Sig100_send_init_j(void)
		{
			Sig100_send_pkt[0] = '!';	// Tell hub we want Enc_data  (128 bytes)
			Sig100_send_pkt[1] = 'j';	// Initial data
			Sig100_send_pkt[2] = 'J';	// This is a big request, make sure pkt not corrupt

			Sig100_checksum_and_send_pkt(SIG100_j_PKT_LEN);

			Sig100_verify_j_pkt();

			return;

		}
		#endif
		//----------------------------------------------------------------------------
		#ifdef MH_DDD
		#define SIG100_i_PKT_LEN 5

		void Sig100_send_init_i(void)
		{
			Sig100_send_pkt[0] = '!';	// Tell hub some new arguments coming
			Sig100_send_pkt[1] = 'i';	// Initial data

			Put_int2(Sig100_send_pkt+2,Stat_Rec.run_number);

			Sig100_checksum_and_send_pkt(SIG100_i_PKT_LEN);

			Sig100_verify_i_pkt();
		}
		#endif
#ifdef MH_AAA
struct
{
	uint8_t ud_stops;
	uint8_t pole_pairs;
	int16_t fine_stop_offset;
	int16_t coarse_stop;
	int16_t feather_stop;
	int16_t reverse_stop;
	uint16_t motor_gear_ratio;
	uint16_t hub_id;
	uint8_t hub_software_version;	// May need a hardware version??
} Sig100_hub;


int Sig100_pos_from_angle(float angle)
{
    float screw_pos,pos;
    float motor_pos;
    float encoder_pos;
    int enc_pos;
    float motor_gear_ratio;

    pos  = fBladePos(angle);

 // OK, so the pos returned is in mm and is relative to 30 degrees, which is when the cam is at the top.
 // So, if the angle was 30 degrees, it should return zero
 // The encoder_pos returned is the number of turns of the motor * how many times the position counter is incremented per turn. It will be one
 // for a single pole pair.

    // Now to translate mm to turns of motor

	screw_pos = pos / MM_PER_TURN;
    motor_gear_ratio = (float)getParameter(BL_MOTOR_RATIO);
    motor_gear_ratio /= 10;	// Implied decimal point
	motor_pos = screw_pos * motor_gear_ratio;

	WORD pole_pairs = getParameter(BL_MOTOR_POLE_PAIRS);
	float increments = (float) pole_pairs;
	if(increments == 0) increments = 1;
	encoder_pos = motor_pos * increments;
	enc_pos = iround(encoder_pos);
	return enc_pos;
}
//---------------------------------------------------------------------------------------------------------------------
float Sig100_GetBladeAngle(int encoder_pos_from_fine)
{
    float screw_pos,pos;
    float blade_angle;
    float motor_pos;
    float motor_gear_ratio;

    int encoder_pos = encoder_pos_from_fine + Sig100_hub.fine_stop_offset;
    WORD pole_pairs = getParameter(BL_MOTOR_POLE_PAIRS);
    if(pole_pairs == 0) pole_pairs = 1;	// default
    motor_pos = (float)encoder_pos / pole_pairs;
    motor_gear_ratio = getParameter(BL_MOTOR_RATIO);	// We were going to add an implied decimal point....
    motor_gear_ratio /= 10;								// and we did...
    screw_pos = motor_pos / motor_gear_ratio;
    pos       = screw_pos * MM_PER_TURN;
    blade_angle = fBladeAngle(pos);
//    blade_angle += 0.05;		// So will round to closest decimal point
    return blade_angle;
}
//------------------------------------------------------------------------------
int16_t Sig100_format_stop_offset(int16_t stop_angle)
{
// Note stop angle zero may be valid for reverse

	float f_stop_angle = (float) stop_angle;
	f_stop_angle /= 10;	//  Because implied decimal place
	int ac200_pos = Sig100_pos_from_angle(f_stop_angle);
	int16_t hub_pos = ac200_pos - Sig100_hub.fine_stop_offset;	// Relative to fine stop
	return hub_pos;
}


//------------------------------------------------------------------------------

void Sig100_Enc_Angle_Init(void)
{
	if(getParameter (BL_ENABLED) != 1) return;

	WORD ud_hub_stops = getParameter(BL_HUB_STOPS);

	Sig100_hub.ud_stops = (uint8_t)ud_hub_stops;
	Sig100_hub.pole_pairs = (uint8_t) getParameter(BL_MOTOR_POLE_PAIRS);
	Sig100_hub.motor_gear_ratio = (uint16_t) getParameter(BL_MOTOR_RATIO);
	WORD w_fine_stop = getParameter (BL_FINE_STOP);	// Get measured angle for fine stop with one implied decimal place
	float fine_stop = (float) w_fine_stop;
	fine_stop /= 10;	// Because implied decimal place
	Sig100_hub.fine_stop_offset = Sig100_pos_from_angle(fine_stop);

	Sig100_hub.coarse_stop = 0;
	Sig100_hub.feather_stop = 0;
	Sig100_hub.reverse_stop = 0;

	int16_t stop_angle = 0;
	if(ud_hub_stops & HUB_UD_COARSE_STOP)	// MHH:05/12/2022. Think we want this either way.
	{
		stop_angle = (int16_t)getParameter (BL_COARSE_STOP);	// This may be negative for reverse
		Sig100_hub.coarse_stop = Sig100_format_stop_offset(stop_angle);
	}
	if(ud_hub_stops & HUB_UD_FEATHER_STOP)
	{
		stop_angle = (int16_t)getParameter (BL_FEATHER_STOP);	// This may be negative for reverse
		Sig100_hub.feather_stop = Sig100_format_stop_offset(stop_angle);
	}
	if(ud_hub_stops & HUB_UD_REVERSE_STOP)
	{
		stop_angle = (int16_t)getParameter (BL_REVERSE_STOP);	// This may be negative for reverse
		Sig100_hub.reverse_stop = Sig100_format_stop_offset(stop_angle);
	}
#ifdef MH_OLD_LOGIC
	if(w_coarse_stop == 0) return;	// Possibly should abort if user defined coarse stop

	float coarse_stop = (float) w_coarse_stop;
	coarse_stop /= 10;	//  Because implied decimal place
	int pos = Sig100_pos_from_angle(coarse_stop);
	Sig100_pos.coarse_stop = pos - Sig100_pos.fine_stop_offset;	// Relative to fine stop
#endif
}
#endif

//-----------------------------------------------------------------------------------------------------------------------
#ifdef MH_SIG100_NO_ISR
void Sig100_HDC_Set(bool bval)
{
	HDC_write(bval);
}

int Sig100_UARTGetChar(void)
{
	int r0= SIG100_getc_timeout(SIG100_TIMEOUT_MILLISECS);
	return r0;
}
//bool Sig100_init_done=false;
int Sig100_register[8];
//#define SIG60_DELAY_MS	5
#endif
#ifdef MH_AAA
typedef enum
{
	SEND_PKT_I=0,
	SEND_PKT_J,
	SEND_PKT_K,
	FINISHED,
}Hub_handshake_status;
#endif
#ifdef MH_YYY
bool Sig100_verify_I_pkt(void)
{

	if(SIG100_checksum_input_pkt(SIG100_I_PKT_LEN) == false)
	{
		return false;
	}
	for(int i=1;i<10;i++)	// Check what was received is same as sent
	{
		uint8_t cr = SIG100_input_packet[i];
		uint8_t cs = Sig100_send_pkt[i];
		if(cr != cs)
		{
			return false;
		}
	}
	Sig100_init_status = 1;	// Now for pkt J
	return true;
}
#endif
#ifdef MH_AAA
bool Sig100_verify_i_pkt(void)
{

	if(SIG100_checksum_input_pkt(11) == false)
	{
		return false;
	}
	WORD hub_id = Get_word(SIG100_input_packet+2);
	WORD update_count = Get_word(SIG100_input_packet+4);
	WORD total_errors = Get_word(SIG100_input_packet+6);
	WORD hub_software_version = Get_word(SIG100_input_packet+8);

	WORD param_hub_id = getParameter (BL_HUB_ID);
	WORD param_update_count = getParameter (BL_UPDATE_COUNT);
	WORD param_total_errors = getParameter (BL_TOTAL_ERRORS);
	WORD param_hub_software_version = getParameter(BL_HUB_SOFTWARE_VERSION);
	if(param_hub_software_version != hub_software_version)
	{
		writeParameter (BL_HUB_SOFTWARE_VERSION, hub_software_version);
	}

//#define MH_FORCE_INI_J
#ifdef MH_FORCE_INI_J
	Sig100_init_status = 2; // MHH:DEBUG!!!
	return true;
#endif
	if(hub_id == param_hub_id && update_count == param_update_count)
	{
		if(Sig100_load_Enc_data_from_page_zero())
		{
			if(total_errors == param_total_errors)// Do we need to get error data?
			{
				Sig100_init_status = 3;	// Hopefully normal path
				return true;
			}

			Sig100_init_status = 1;	// Will need to load error data only
			return true;

		}
	}
	Sig100_init_status = 2;	// Looks like we need a full Enc_data update from Hub...
	return true;
}
#endif
//----------------------------------------------------------------------------
#ifdef MH_SIG100_NO_ISR
void SIG100_test_send_receive(void)
{
	int ci;
	Sig100_UARTPutChar('?');
	for(int i=0;i<10;i++)	// Receive "0123456789"
	{
		ci = Sig100_UARTGetChar();
		if(ci > 0) PC_putc((char)ci);
		if(ci != i + '0')
		{
			PC_putc('!');
			SIG100_error(1);
		}
	}
//	wait_ms(1);
	for(int i=0;i<10;i++)	// Send "abcdefghij"
	{
		char c ='a' + i;
		Sig100_UARTPutChar(c);
	}
	ci = Sig100_UARTGetChar();
	if(ci > 0) PC_putc((char)ci);
	if(ci != 'x')
	{
		SIG100_error(2);
	}
}
#endif
//---------------------------------------------------------------------------------------------
#ifdef MH_SIG100_NO_ISR
/* Gets a character from the UART, returns EOF if no character is ready */
int SIG100_UARTGetChar(void)
{
	while ((Chip_UART_ReadLineStatus(SIG100_UART) & UART_LSR_RDR) == 0);	// Wait until byte available

	return Chip_UART_ReadByte(SIG100_UART);
}
//-------------------------------------------------------------------------------
int SIG100_getc_timeout(uint32_t millisecs)
{

	uint32_t finish_us = us_ticker_read() + millisecs * 1000;
	for(;;)
	{
		if(SIG100_UARTReadable())	// byte available?
		{
			return SIG100_UARTGetChar();
		}
		if(us_ticker_read() >= finish_us)
		{
			return -1;
		}
	}
}
#endif
#ifdef MH_AAA
bool Sig100_verify_J_pkt(void)
{
	if(SIG100_checksum_input_pkt(SIG100_J_PKT_LEN) == false)
	{
		return false;
	}

	if((Sig100_hub.ud_stops & 2) == 0) // Not a user defined coarse stop
	{
		WORD hub_coarse_stop = Get_word(SIG100_input_packet + 2);
		Sig100_hub.coarse_stop = hub_coarse_stop;
		float angle = Sig100_GetBladeAngle(hub_coarse_stop);
		WORD coarse_stop_angle = (WORD)(angle * 10);	// implied dec point
		if(coarse_stop_angle != getParameter(BL_COARSE_STOP))	// Just so AC200User will know where coarse stop is
		{
			writeParameter (BL_COARSE_STOP,coarse_stop_angle);
			PRINTF("Coarse Stop:%d\r\n",coarse_stop_angle)
		}
	}
	Sig100_hub.hub_id = Get_word(SIG100_input_packet + 4);
	Sig100_hub.hub_software_version = SIG100_input_packet[6];
	AC200Enc_version = Sig100_hub.hub_software_version;	// For remote.c
	if(Sig100_hub.hub_software_version != getParameter(BL_HUB_SOFTWARE_VERSION))	// Update parameters as may need to know this in diagnostics
	{
		writeParameter (BL_HUB_SOFTWARE_VERSION,Sig100_hub.hub_software_version);
	}

	Sig100_init_status = 2;	// Ini complete
//	PC_PRINTF("P:%c ",header);
	return true;
}
#endif
//--------------------------------------------------------------------------------------------
#ifdef MH_AAA
bool Sig100_verify_init_pkts(void)
{
	// Here if first character is '>'

	char type = SIG100_input_packet[1];

	switch(type)
	{
#ifdef MH_YYY
	case 'I':
		return Sig100_verify_I_pkt();

	case 'J':
		return Sig100_verify_J_pkt();
#endif
	case 'i':
		return Sig100_verify_i_pkt();


//	case 'j':
//		return Sig100_verify_j_pkt();



	default:
		return false;
	}

}
#endif

#ifdef MH_DDD
bool SIG100_check_packet(void)
{
	SIG100_packet_ix = 0;	// Assume that entire packet is waiting.
	for(;;)
	{
//		wait_ms(1);
/*		int ch = SIG100_UARTGetChar_nowait();
		if(ch == EOF)	// Expecting 8 bytes
		{
			break;
		}
*/

		if(SIG100_UART_READABLE() == false)
		{
			break;
		}
		int ch = SIG100_UART_GETC();
		if(SIG100_packet_ix == 0)
		{
//			PC_putc((char) ch);	// Debug!!
			if(ch != '_' && ch != '&' && ch != '#')
			{
				continue;
			}
		}
		if(SIG100_packet_ix < SIG100_PACKET_MAX)
		{
			SIG100_input_packet[SIG100_packet_ix++] = ch;	// append ch
		}
	}
	if(SIG100_packet_ix == 0)	// Anything in packet?
	{
		Sig100_debug(1);
		return false;
	}
	uint8_t header = SIG100_input_packet[0];
//	PC_putc(header);	// Debug!!


	switch (header)
	{
	case '_':	// Normal, just includes stop bit status flags and maybe other flags, plus voltage
		if(SIG100_packet_ix < SIG100_SMALL_PKT_LEN) // Allow it to be greater than 3, in case spurious
		{
			Sig100_debug(2);
			return false;
		}
		if(SIG100_checksum_input_pkt(SIG100_SMALL_PKT_LEN) == false)
		{
			Sig100_debug(3);
			return false;
		}
		break;


	case '&':	// This packet includes position, voltage and temperature
		if(SIG100_packet_ix < SIG100_LARGE_PKT_LEN) // Allow it to be greater than 8, in case spurious bytes
		{
			Sig100_debug(4);
			return false;
		}
		if(SIG100_checksum_input_pkt(SIG100_LARGE_PKT_LEN) == false)
		{
			Sig100_debug(5);
			return false;
		}
		break;

	case '#':	// Special error pkt
		Sig100_process_error_pkt();
		break;

#ifdef MH_AAA
	case '>':	// Special response from init only. If we want to append data, will need a new header
		if(Sig100_verify_init_pkts() == false)
		{
			Sig100_debug(6);
			return false;
		}
		return true;
#endif
	default:	// Should never get here
		Sig100_debug(6);
		return false;

	}
//	PC_PUTC(header);
	uint8_t stop_flags = SIG100_input_packet[1];	// Common position for short and long pkt.
	if(Hub_calibrate_state == CAL_FINE_HARDSTOP)
	{
		Cal_flags = stop_flags;
	}
	ADC_adjusted_voltage = SIG100_input_packet[2];
	stop_flags <<= 4;	// To be consistent with existing logic
	if(stop_flags & S_STOP_REVERSE)
	{
		stop_flags &= ~S_STOP_FINE;		// Mask out fine stop if reverse on
	}

	if(stop_flags & S_STOP_FEATHER)
	{
		stop_flags &= ~S_STOP_COARSE;		// Mask out coarse stop if feather on
	}
	else
	{
		if(operatingMode() == MANUAL)
		{
//			if(Sig100_hub.ud_stops & HUB_UD_FEATHER_STOP && dState != MD_BETA)	// Feather enabled?
			if(Enc_data.user_defined_stops & HUB_UD_FEATHER_STOP && dState != MD_BETA)	// Feather enabled?
			{
				BYTE mankeys = manualKeys();
				if((mankeys & MANUAL_KEY_FEATHER) && (mankeys & MANUAL_KEY_COARSE))
				{
					stop_flags &= ~S_STOP_COARSE;		// Mask out coarse stop if feathering
				}
			}
//			if(Sig100_hub.ud_stops & HUB_UD_REVERSE_STOP)	// Reverse enabled?
			if(Enc_data.user_defined_stops & HUB_UD_REVERSE_STOP)	// Reverse enabled?
			{
				if(dState == MD_BETA || dState == MD_BETA_EXIT)
				{
//					if(SIG100_beta_mode_on())
					{
//						stop_flags &= ~S_STOP_FINE;
						if(stop_flags & S_STOP_REVERSE)	// For Beta reverse, if REVERSE_STOP on then REVERSE_STOP off, FEATHER_STOP on
						{
							stop_flags &= ~(S_STOP_REVERSE);
							stop_flags |= S_STOP_FEATHER;
						}
					}
				}
			}
		}
	}
	resetStateBits (300,S_STOP_FINE | S_STOP_COARSE | S_STOP_FEATHER | S_STOP_REVERSE);	// Turn stop bit flags off.
	setStateBits(300,(WORD)stop_flags);


	if(header == '_')
	{
		return true;
	}

	if(header == '&')
	{
		ADC_adjusted_temp = SIG100_input_packet[3];
		int16_t pos16 = SIG100_input_packet[5] | (SIG100_input_packet[6] << 8);
		Encoder_Pos = pos16;
		return true;
	}

	// If we drop through then we have an extended pkt.
	// Format:
	// 0 = header
	// 1 = flags
	// 2 = volts
	// 3 = type. eg I = Initial
	// 4 = data 1
	// 5 = data 2
	// 6 = data 3
	// 7 = checksum


	// No extended packets yet. Ini handled differently.

	Sig100_debug(7);
	return false;

}
#else
#endif

#ifdef MH_BBB

			if(total_errors == param_total_errors)// Do we need to get error data?
			{
				Sig100_init_status = 3;	// Hopefully normal path
				return true;
			}

			Sig100_init_status = 1;	// Will need to load error data only
			return true;
#endif
#ifdef MH_YYY
void Sig100_send_init_I(void)
{
	Sig100_send_pkt[0] = '!';	// Tell hub some new arguments coming
	Sig100_send_pkt[1] = 'I';	// Initial data
	Sig100_send_pkt[2] = Sig100_hub.ud_stops;			// This is the most important one.
	Sig100_send_pkt[3] = Sig100_hub.pole_pairs;

	Put_int2(Sig100_send_pkt+4,Sig100_hub.coarse_stop);
	Put_int2(Sig100_send_pkt+6,Sig100_hub.feather_stop);
	Put_int2(Sig100_send_pkt+8,Sig100_hub.reverse_stop);
	Put_int2(Sig100_send_pkt+10,Sig100_hub.motor_gear_ratio);

	Sig100_send_pkt[12] = 0;		// Reserved for future

	Sig100_checksum_output_pkt(SIG100_I_PKT_LEN);
	Sig100_send_output_pkt(SIG100_I_PKT_LEN);
}
#endif
//----------------------------------------------------------------------------
#ifdef MH_AAA
#define SIG100_i_PKT_LEN 5
void Sig100_send_init_i(void)
{
	Sig100_send_pkt[0] = '!';	// Tell hub some new arguments coming
	Sig100_send_pkt[1] = 'i';	// Initial data

	Put_int2(Sig100_send_pkt+2,Stat_Rec.run_number);

	Sig100_checksum_and_send_pkt(SIG100_i_PKT_LEN);
}
#endif

#ifdef MH_DDD
//---------------------------------------------------------------------------------------------------
void Sig100_send_init_j(void)
{
	Sig100_send_pkt[0] = '!';	// Tell hub we want Enc_data  (128 bytes)
	Sig100_send_pkt[1] = 'j';	// Initial data
	Sig100_send_pkt[2] = 'J';	// This is a big request, make sure pkt not corrupt

	Sig100_checksum_and_send_pkt(SIG100_j_PKT_LEN);

	Sig100_verify_j_pkt();

	return;

}
//---------------------------------------------------------------------------------------------------
#define SIG100_k_PKT_LEN 4

//---------------------------------------------------------------------------------------------------
void Sig100_send_init_k(void)
{
	Sig100_send_pkt[0] = '!';	// Tell hub we want Enc error_data  (64 bytes)
	Sig100_send_pkt[1] = 'k';	// Initial data
	Sig100_send_pkt[2] = 'K';	// This is a big request, make sure pkt not corrupt

	Sig100_checksum_and_send_pkt(SIG100_j_PKT_LEN);

	Sig100_verify_k_pkt();

	return;

}

//----------------------------------------------------------------------------
#define SIG100_i_PKT_LEN 5
void Sig100_send_init_i(void)
{
	Sig100_send_pkt[0] = '!';	// Tell hub some new arguments coming
	Sig100_send_pkt[1] = 'i';	// Initial data

	Put_int2(Sig100_send_pkt+2,Stat_Rec.run_number);

	Sig100_checksum_and_send_pkt(SIG100_i_PKT_LEN);

	Sig100_verify_i_pkt();
}
//----------------------------------------------------------------------------
void SIG100_send_init(void)
{
	switch(Sig100_init_status)
	{
	case 0:
//		Sig100_send_init_I();
		Sig100_send_init_i();
		return;
#ifdef MH_BBB
	case 1:
		Sig100_send_init_k();
		return;
#endif
	case 2:
//		Sig100_send_init_J();
		Sig100_send_init_j();
		return;

	default:
		return;		// Should never get here
	}
}
#endif
#ifdef MH_DDD

//		Sig100_display = true;
		Sig100_send_pkt[0] = '!';
		Sig100_send_pkt[1] = 'X';
		Sig100_send_pkt[2] = (uint8_t)Hub_calibrate_state;
		Sig100_checksum_output_pkt(4);
		Sig100_send_output_pkt(4);
		return;
#endif
#ifdef MH_DDD
				SIG100_send_position_request(c);
#endif
#ifdef MH_DDD
	Sig100_UARTPutChar(c);	// Once a tick for testing
#else
	SIG100_send_command_pkt(c);
#endif
	//			wait_ms(100);
	//			Sig100_clear_hub_serial();

	#ifdef MH_DDD
				strcpy((char *)Sig100_send_pkt,"!CMode=1");
				Sig100_send_output_pkt(8);
	#endif
#ifdef MH_DDD
	for(int i=0;i<4;i++)
	{
		c = Aux_getc_timeout(100);		// Allow a tenth of a second
		if(c < 0) break;
		Hub_header[count++] = (uint8_t)c;
	}
	if(count != 4) return false;		// Not even 4 bytes
	if(Hub_header[0] != '>') return false;
	if(Hub_header[1] != 'j') return false;
	if(Hub_header[2] != '{') return false;
	int data_count = Hub_header[3];
#endif
#ifdef MH_AAA

   uint8_t *param_start_p = (uint8_t *)&ps;
   uint8_t *param_hub_p = (uint8_t *)(&ps.parms[BL_HUB_ID]);
   uint8_t *param_hub_end_p = (uint8_t *)(&ps.parms[BL_MOTOR_POLE_PAIRS]);
   pos = param_hub_p - param_start_p;
   int len = param_hub_end_p - param_hub_p + 2;

   p_eeprom_i2c_pos(pos);
	p_eeprom_i2c_write(param_hub_p,len);		// 9 parameters, although software version will be updated on handshake
#endif
#ifdef MH_AAA
    ps.parms[BL_TOTAL_ERRORS] = Enc_data.total_errors;
    ps.parms[BL_ERR_COUNT]    = Enc_data.error.count;
    ps.parms[BL_ERR_LAST]     = Enc_data.error.code;
    ps.parms[BL_ERR_RUN]     = Enc_data.error.ac200_run;

    ps.parms[BL_ABORT_COUNT]    = Enc_data.abort.count;
    ps.parms[BL_ABORT_LAST]     = Enc_data.abort.code;
    ps.parms[BL_ABORT_RUN]     = Enc_data.abort.ac200_run;

    Sig100_update_param_range(BL_TOTAL_ERRORS,BL_ABORT_RUN);
#endif
#ifdef MH_YYY
typedef struct
{
	uint32_t magic;				// 0.
	uint16_t hub_id;			// 4.
	uint16_t update_count;		// 6. Incremented by AC200HubCalibrator only
	uint16_t modified_date;		// 8.
	uint16_t creation_date;		// 10.

	uint16_t cal_tolerance;		// 12. In Ecount units, equivalent to +/- 0.05 mm, provided by Calibrator. Could be unit8_t
	uint16_t cal_motor_ratio;	// 14. Test hub uses 294.6
	uint16_t cal_cam_length;	// 16. In tenths of a mm, usually 26.0
	int16_t cal_blade_offset;	// 18. In degrees, usually 30.0
	uint16_t cal_leadscrew_mm;	// 20. In mm, 3 implied decimal places. Default is 3.175 (25.4/8)

	uint8_t cal_pole_pairs;		// 22. Could be uint8_t
	uint8_t user_defined_stops;	// 23. Bit flags of user defined stops (coarse,feather and reverse)

	int16_t ud_coarse_stop;		// 24. User defined coarse stop
	int16_t ud_feather_stop;	// 26.
	int16_t ud_reverse_stop;	// 28.

	int16_t cal_fine_stop;		// 30.
	int16_t cal_ref_open;		// 32.
	int16_t cal_ref_closed;		// 34.

	int16_t cal_coarse_hard_stop;	// 36.
	int16_t cal_fine_hard_stop;		// 38.
	uint16_t cal_tdc_to_coarse;		// 40. In mm, one decimal place, usually 22.0 mm
	uint16_t cal_tdc_to_fine;		// 42. In mm, one decimal place, calculated by Calibrator

// Following can be calculated, but so long as there is room...

	int16_t fine_stop_angle;		// 44. This is set by Calibrator based on micro-switch stop, others are user defined
	int16_t coarse_stop_angle;		// 46.
	int16_t feather_stop_angle;		// 48.
	int16_t reverse_stop_angle;		// 50.
	int16_t ref_open_angle;			// 52.
	int16_t ref_closed_angle;		// 54. related to cal_ref_open and cal_ref_closed
	int16_t cal_pos;				// 56. Returned from calibrator
	uint16_t fill_a;				// 58.

	uint32_t crc32;					// 60.
									// 64.Should fit in 1 flash page
	uint32_t magic3;				// 64.0.When just loading from this area
	uint16_t ac200_run;				// 68.4. Use for error reporting
	uint16_t hub_run;				// 70.6
	uint16_t rpm_starts;			// 72.8
	uint16_t hub_software_version;	// 74.10

	int16_t pos;					// 76.12. These 2 updated together, so we know if we have an rpm restart (if motor was going when hub mcu stopped)
	uint16_t rpm;					// 78.14.
	// Note: Copy until up to rpm from hub, as maintaining data below in struct is handled by ac200
	// We may not need any of the data below in the hub. Have decided that it is simpler and more robust to send any error data from the hub to ac200
	// rather than trying to store, as the storing may be causing the error.

	uint16_t total_errors;			// 80.16.

	Enc_err_td abort;				// 82.18.
	Enc_err_td watchdog;			// 92.28.
	Enc_err_td error;				// 102.38.
									// 112.48.
	uint16_t fill_b[8];				// 112.48
									// 128.64.
}Enc_data_td;
Enc_data_td Enc_data;
#else
typedef struct
{
	uint16_t code;			// 0.
	uint8_t command;		// 2.
	uint8_t fill;			// 3
	uint16_t ac200_run;		// 4.
	uint16_t count;			// 6. of this code, if multiple occurrences of error.
//	uint16_t from;			// 8. where it was called from
	uint16_t total;			// 8.
} Enc_err_td;				// 10.
typedef struct
{
	uint32_t magic;				// 0.
	uint16_t hub_id;			// 4.
	uint16_t update_count;		// 6. Incremented by AC200HubCalibrator only
	uint16_t modified_date;		// 8.
	uint16_t creation_date;		// 10.

	uint16_t cal_tolerance;		// 12. In Ecount units, equivalent to +/- 0.05 mm, provided by Calibrator. Could be unit8_t
	uint16_t cal_motor_ratio;	// 14. Test hub uses 294.6
	uint16_t cal_cam_length;	// 16. In tenths of a mm, usually 26.0
	int16_t cal_blade_offset;	// 18. In degrees, usually 30.0
	uint16_t cal_leadscrew_mm;	// 20. In mm, 3 implied decimal places. Default is 3.175 (25.4/8)

	uint8_t cal_pole_pairs;		// 22. Could be uint8_t
	uint8_t user_defined_stops;	// 23. Bit flags of user defined stops (coarse,feather and reverse)

	int16_t ud_coarse_stop;		// 24. User defined coarse stop
	int16_t ud_feather_stop;	// 26.
	int16_t ud_reverse_stop;	// 28.

	int16_t cal_fine_stop;		// 30.
	int16_t cal_coarse_hard_stop;	// 32.
	int16_t cal_fine_hard_stop;		// 34.
	int16_t ref_max;				// 36.Could be 1 byte. These fields build picture of reference stops
	int16_t ref_start[3];			// 38.
	int16_t ref_end[3];				// 44.
	int16_t ref_main;				// 50.
	int16_t fill[4];

	int16_t cal_pos;				// 58. Returned from calibrator, could be moved out
	uint32_t crc32;					// 60.
									// 64.Should fit in 1 flash page


/*

	uint16_t cal_tdc_to_coarse;		// 40. In mm, one decimal place, usually 22.0 mm
	uint16_t cal_tdc_to_fine;		// 42. In mm, one decimal place, calculated by Calibrator

// Following can be calculated, but so long as there is room...

	int16_t fine_stop_angle;		// 44. This is set by Calibrator based on micro-switch stop, others are user defined
	int16_t coarse_stop_angle;		// 46.
	int16_t feather_stop_angle;		// 48.
	int16_t reverse_stop_angle;		// 50.
	int16_t cal_pos;				// 56. Returned from calibrator
	uint16_t fill_a;				// 58.
*/
	uint32_t magic3;				// 64.0.When just loading from this area
	uint16_t ac200_run;				// 68.4. Use for error reporting
	uint16_t hub_run;				// 70.6
	uint16_t rpm_starts;			// 72.8
	uint16_t hub_software_version;	// 74.10

	int16_t pos;					// 76.12. These 2 updated together, so we know if we have an rpm restart (if motor was going when hub mcu stopped)
	uint16_t rpm;					// 78.14.
	// Note: Copy until up to rpm from hub, as maintaining data below in struct is handled by ac200
	// We may not need any of the data below in the hub. Have decided that it is simpler and more robust to send any error data from the hub to ac200
	// rather than trying to store, as the storing may be causing the error.

	uint16_t total_errors;			// 80.16.

	Enc_err_td abort;				// 82.18.
	Enc_err_td watchdog;			// 92.28.
	Enc_err_td error;				// 102.38.
									// 112.48.
	uint16_t fill_b[8];				// 112.48
									// 128.64.

}Enc_data_td;
Enc_data_td Enc_data;

#endif
#ifdef MH_SIG100_NO_ISR
//--------------------------------------------------------------------------------------------
bool SIG100_UARTReadable(void)
{
	bool readable = ((Chip_UART_ReadLineStatus(SIG100_UART) & UART_LSR_RDR) != 0);
	return readable;
}
#endif
//---------------------------------------------------------------------------------------------
/* Gets a character from the UART, returns EOF if no character is ready */
#ifdef MH_UNUSED
int SIG100_UARTGetChar(void)
{
	while ((Chip_UART_ReadLineStatus(SIG100_UART) & UART_LSR_RDR) == 0);	// Wait until byte available

	return Chip_UART_ReadByte(SIG100_UART);
}
#endif
//---------------------------------------------------------------------------------------------
/* Gets a character from the UART, returns EOF if no character is ready */
#ifdef MH_SIG100_NO_ISR
int SIG100_UARTGetChar_nowait(void)
{
	if((Chip_UART_ReadLineStatus(SIG100_UART) & UART_LSR_RDR) == 0) return EOF;

	return Chip_UART_ReadByte(SIG100_UART);
}
#endif
//--------------------------------------------------------------------------------------------
bool Sig100_verify_k_pkt(void)
{
	// OK, going to break the rules here and wait for a response.
	// Expect header of ">j"
	// Then "{ndddd...cccc}
	// where n = number of bytes expected, including crc32
	// d = data bytes
	// cccc = 4 byte crc32

// We should set a timeout, or read with a timeout. Not sure about going into sig100 read mode without finishing sending.
// Could check to see if anything left to send, and/or status uart
/*
 *
	Command_string[0] = '>';
	Command_string[1] = 'k';
	Command_string[2] = '{';
	Command_string[3] = 68;		// 64+4

	Send_cmd_pkt(4);

	uint8_t *data_p = (uint8_t *)&Enc_data;
	Sig100_send_binary_data(data_p,128);
	Sig100_UARTPutChar('}');	// Final character
 *
 */
	int count=0;
	int c;
	for(int i=0;i<4;i++)
	{
		c = Aux_getc_timeout(100);		// Allow a tenth of a second
		if(c < 0) break;
		Hub_header[count++] = (uint8_t)c;
	}
	if(count != 4) return false;		// Not even 4 bytes
	if(Hub_header[0] != '>') return false;
	if(Hub_header[1] != 'k') return false;
	if(Hub_header[2] != '{') return false;
	int data_count = Hub_header[3];
	if(data_count != 68) return false;

	count = 0;
	for(int i=0;i<data_count;i++)
	{
		c = Aux_getc_timeout(100);		// Allow a tenth of a second
		if(c < 0) break;
		Hub_data[count++] = (uint8_t)c;
	}
	c = Aux_getc_timeout(100);
	if(c != '}') return false;

	// Data seems in correct format. Now to check crc32 value
	int data_size = data_count - 4;
	uint32_t data_crc32 = Get_uint32(Hub_data + data_size);

	uint32_t crc32 = Wiki_CRC32(Hub_data,data_size);
	if(data_crc32 != crc32 ) return false;

	// Data looks OK, now to copy to Enc_data struct.

	 Sig100_copy_Hub_data_to_Enc_data_B(data_size);

	 //Now need to write this data to page zero of diagnostics page (first 256 bytes = params + stats).
	 // Write at position (256 * 3) = 768 as is unused.

	 int pos = 768+64;
	 p_eeprom_i2c_pos(pos);
	 uint8_t *bp = (uint8_t *)Hub_data;
	 p_eeprom_i2c_write(bp,data_size);		// Should be 64 bytes

//	 Sig100_update_brushless_error_params();
	Sig100_init_status = 3;		// All done!!
	return true;
}

void Sig100_copy_Hub_data_to_Enc_data_B(int data_len)
{
	uint8_t *dst_p = (uint8_t *)&Enc_data.magic3;		// Do we need & ?
	if(data_len > 64) data_len = 64;	// To be sure
	for(int i=0;i<data_len;i++)
	{
		*dst_p++ = Hub_data[i];
	}
}



#ifdef MH_XXX
// Now, if the same error code as last time, just increment the count, otherwise copy new details in
	if(Hub_err.code == enc_err_p->code)
	{
		enc_err_p->command = Hub_err.command;
		enc_err_p->ac200_run = Hub_err.ac200_run;
		enc_err_p->count++;
		enc_err_p->total++;
	}
	else
	{
		enc_err_p->code = Hub_err.code;
		enc_err_p->command = Hub_err.command;
		enc_err_p->ac200_run = Hub_err.ac200_run;
		enc_err_p->count = 1;
		enc_err_p->total++;
	}
#endif




	/*
	BYTE ignSpeed (ULONG *rpm)
	{
	#ifdef AC210_PORT
		Abort(SRC_RPM+10,"ignSpeed:Not coded");
		return 0;
	#else
		char buf[30];
	    BYTE rval = 0;

	    ignRPM = calculateIgnRPM ( rawIgnCount);
	    if (ignSpeedChanged)
	    {
	        ignSpeedChanged = 0;
	        // Debug call
	        if (debug & DEBUG_SPEED_INPUT)
	        {
	            sprintf (buf, "Ign RPM -> %lu.%d\r\n", ignRPM/10, ignRPM%10);
	            txDebug(buf);
	        }
	        rval = 1;
	    }
	    *rpm = ignRPM;
	    return (rval);
	#endif
	}
	*/

	// initialisation functions
	void initMagSpeed (void)
	{
	#ifndef AC210_PORT
		ULONG far *vect_ptr;

	    // mode register set up
	    MAG_CTR_MODE = 0x42;      // f8, falling edge to falling edge, period
	    // Set up ISR
	    vect_ptr = vector_table + MAG_INT_NO;
	    *vect_ptr = (ULONG)magInterrupt;
	    // enable the Interrupt, high priority
	    MAG_INT_REG = 0x06;
	    // start the counter
	    TBSR |= MAG_START_MASK;
	#endif
	}

	void initIgnSpeed (void)
	{
	#ifndef AC210_PORT
	    ULONG far *vect_ptr;

	    // mode register set up
	    IGN_CTR_MODE = 0x46;      // f8, rising edge to rising edge, period
	    // Set up ISR
	    vect_ptr = vector_table + IGN_INT_NO;
	    *vect_ptr = (ULONG)ignInterrupt;
	    // enable the Interrupt, high priority
	    IGN_INT_REG = 0x06;
	    // start the counter
	    TBSR |= IGN_START_MASK;
	#endif
	}
	#ifndef AC210_PORT
	// the ISR's
	void magInterrupt (void)
	{
	    ULONG tmp;

	    enableInterrupt();
	    if (MAG_CTR_MODE & 0x20)
	    {
	        // this is an overflow
	        magOverflow += 65536L;
	        // check for "0" RPM
	        if (magOverflow >= SPEED_ZERO)
	        {
	            magOverflow = 0;
	            rawMagCount = ZERO_PERIOD;
	            // stop and start again..
	            TBSR &= ~MAG_START_MASK;
	            MAG_CTR_MODE = 0x00;
	            MAG_CTR_MODE = 0x42;
	            TBSR |= MAG_START_MASK;
	        }
	    }
	    else
	    {
	        tmp = filterMagRPM (magOverflow + MAG_CTR_COUNT);
	        magOverflow=0;
	        if (tmp != rawMagCount)
	        {
	            magSpeedChanged = 1;
	            rawMagCount = tmp;
	        }
	    }
	    // reset overflow flag
	    MAG_CTR_MODE &= ~0x20;
	}

	void ignInterrupt (void)
	{
	    ULONG tmp;

	    enableInterrupt();
	    if (IGN_CTR_MODE & 0x20)
	    {
	        // this is an overflow
	        ignOverflow += 65536L;
	        // check for "0" RPM
	        if (ignOverflow >= SPEED_ZERO)
	        {
	            ignOverflow = 0;
	            rawIgnCount = ZERO_PERIOD;
	        }
	    }
	    else
	    {
	        tmp = filterIgnRPM (ignOverflow + IGN_CTR_COUNT);
	        ignOverflow = 0;
	        if (tmp != rawIgnCount)
	        {
	            ignSpeedChanged = 1;
	            rawMagCount = tmp;
	        }
	    }
	    // reset overflow flag
	    IGN_CTR_MODE &= ~0x20;
	}


	/*
	    This filters the raw count (period) according to the following

	    NEW PERIOD = ((FilterCoefficient * OLD PERIOD) + (100-FilterCoefficient)*NEW PERIOD)/100
	    FilterCoefficient is in %

	*/

	ULONG filterMagRPM (ULONG period)
	{
	    ULONG temp, fc;

	    // check for a missed pulse (increase of period by more than 80%)
	#if 0
	    if (rawMagCount != ZERO_PERIOD)
	    {
	        temp = (rawMagCount * 18)/10;
	        if (period > temp)
	        {
	            return (rawMagCount);
	        }
	    }
	#endif
	    fc = 90;		// MHH:22/04/17 hard code to 90.
	//    fc = (ULONG)getParameter(FC_MAG_SPEED);
	    temp = (rawMagCount * fc) + (period * (100 - fc));
	    temp /= 100;
	    return (temp);
	}

	/*  calulation of RPM from period is  done using doubles
	    to give extra accuracy through the scaling and filtering

	    RAW RPM = (2,000,000.0 * 6000.0)/count
	    (gives us 2 dp's (should be 2,000,000 * 60))

	    TRUE RPM = (RAW RPM * ScalingFactor) / 100
	    (ScalingFactor is in percent)

	    Result is then rounded and converted to a long

	    What we should have at the end is a filtered value accurate to 0.1RPM
	*/

	ULONG calculateMagRPM (ULONG period)
	{
	    ULONG  tlong;
	    double temp;


	    if (period == ZERO_PERIOD)
	    {
	        return (0L);
	    }

	    // raw RPM  * 100
	    temp = (2000000.0 * 6000.0)/(double)period;


	   	if(ac2_test_flag != AC2_TEST_ON)	// No scale for AC2_TEST
		{
		    temp = (temp * (double)getParameter(SF_MAG_SPEED))/100.0;
	 	}

	    // round and covert to ULONG
	    tlong = ((ULONG)temp + 5)/10;

	    return (tlong);
	}



	ULONG filterIgnRPM (ULONG period)
	{
	    ULONG temp, fc;

	    // check for a missed pulse (increase of period by more than 80%)
	#if 0
	    if (rawIgnCount != ZERO_PERIOD)
	    {
	        temp = (rawIgnCount * 18)/10;
	        if (period > temp)
	        {
	            return (rawIgnCount);
	        }
	    }
	#endif
	    fc = (ULONG)getParameter(FC_IGN_SPEED);
	    temp = (rawIgnCount * fc) + (period * (100 - fc));
	    temp /= 100;
	    return (temp);
	}


	ULONG calculateIgnRPM (ULONG period)
	{
	    ULONG  tlong;
	    double temp;

	    if (period == ZERO_PERIOD)
	    {
	        return (0L);
	    }

	    // raw RPM  * 100
	    temp = (2000000.0 * 6000.0)/(double)period;

	    // scaled
	    temp = (temp * (double)getParameter(SF_IGN_SPEED))/100.0;

	    // round and covert to ULONG
	    tlong = ((ULONG)temp + 5)/10;

	    return (tlong);
	}
	#endif
#ifdef MH_MAV
struct MagIntStruct
{
	uint32_t Timer_us;
//	uint32_t FilteredPeriod_us;
	uint32_t last_cnt;
	uint32_t cnt;
	uint32_t RPM;
//	uint32_t ecnt;
} MagInt;
#endif
#ifdef MH_MAV
//----------------------------------------------------------------------------------------
void AC210_InitMagTimerPin(void)
{
/* Configure GPIO interrupt pin as input */
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, GPIO_INTERRUPT_PORT, GPIO_INTERRUPT_PIN);

/* Configure the GPIO interrupt */
//	Chip_GPIOINT_SetIntFalling(LPC_GPIOINT, GPIO_INTERRUPT_PORT, 1 << GPIO_INTERRUPT_PIN);
	Chip_GPIOINT_SetIntRising(LPC_GPIOINT, GPIO_INTERRUPT_PORT, 1 << GPIO_INTERRUPT_PIN);
	Chip_GPIOINT_SetIntFalling(LPC_GPIOINT, GPIO_INTERRUPT_PORT, 0);	// Hopefully will clear falling flag??

/* Enable interrupt in the NVIC */
	NVIC_ClearPendingIRQ(GPIO_INTERRUPT_NVIC_NAME);
	NVIC_EnableIRQ(GPIO_INTERRUPT_NVIC_NAME);
}
#endif
#ifdef MH_FFF
	if(Sig100_hub_link)
	{
		AC200_SIG100_putchar(ch);
		if(ch == Sig100_escape[Sig100_escape_cnt])
		{
			if(Sig100_escape_cnt >= 7)
			{
				Sig100_hub_link = false;
				Sig100_escape_cnt = 0;
				PC_puts("OK\r\n");
			}
			else
			{
				Sig100_escape_cnt++;
			}
		}
		else
		{
			Sig100_escape_cnt = 0;
		}
		return;
	}
#endif
	//   uint16_t current = (uint16_t)(scaledValue (A_MOTOR_CURRENT))/10;	// MHH:22/01/2019
	//#ifdef AC210_PORT
	#ifdef MH_RPM_XXX	// MHH:01/11/2018. Not working on Martin's electric test rig.
	    if(Diags_motor_running)
	    {
	    	count_not_running = 0;
	    }
	    else
	    {
	    	count_not_running++;
	    	if(count_not_running == 50)
	    	{
	    		display_rpm = actualspeed;
	    	}

	    	// This logic to cut the saved actualspeed sample rate to 5 times a second if motor not running and max sample rate
	    	// Note: If in manual mode then setspeed = actualspeed.
	    	if(count_not_running > 50)
	    	{
	    		if((EEPROM_log > 1) || (count_not_running%10 == 0))		// Not max rate of 50 samples/sec  or 10th sample
	    		{
	    			// If not active then allow +/- 1% before recording change in rpm
	    			if(((actualspeed >= (display_rpm*99)/100) && (actualspeed <= (display_rpm*101)/100)) == false)
	        		{
	        			display_rpm = actualspeed;
	        		}
	    		}
				actualspeed = display_rpm;
				if(op_mode == MANUAL)
				{
					setspeed = actualspeed;
				}
	    	}
	    }
	#else
	    /* MHH:20/07/2022. With filter option in AC200Diagnostics, forget about idea of "display_rpm"
		if(op_mode == MANUAL)	// MHH:03/11/2018. Try using dState instead of motor running.
		{
			if(actualspeed >= last_manual_speed -1 && actualspeed <= last_manual_speed +1)
			{
				if(manual_unchanged_count++ > 5)
				{
					actualspeed = last_manual_speed;
				}
			}
			else
			{
				manual_unchanged_count = 0;
			}
			last_manual_speed = actualspeed;
			setspeed = actualspeed;
			count_not_running = 0;
		}
		else
		{
			if((dState == MD_IDLE) && ((currentState & (S_STOP_FINE | S_STOP_COARSE | S_STOP_FEATHER | S_STOP_REVERSE)) == 0) && (current == 0))
			{
		    	count_not_running++;
			}
			else
			{
				count_not_running = 0;
			}
		}
		if(count_not_running == NOT_ACTIVE_SECS)		// 2 secs (was 500)
		{
			display_rpm = actualspeed;
		}

		// This logic to cut the saved actualspeed sample rate to 5 times a second if motor not running and max sample rate
		// Note: If in manual mode then setspeed = actualspeed.
		if(count_not_running > NOT_ACTIVE_SECS)
		{
			if((EEPROM_log > 1) || (count_not_running%10 == 0))		// Not max rate of 50 samples/sec  or 10th sample
			{
				// If not active then allow +/- 1% before recording change in rpm
	//			if(((actualspeed >= (display_rpm*99)/100) && (actualspeed <= (display_rpm*101)/100)) == false)
				if(((actualspeed >= (display_rpm*998)/1000) && (actualspeed <= (display_rpm*1002)/1000)) == false)	// MHH:22/01/2019. 1% too big (1% 5000 = 50)
	    		{
	    			display_rpm = actualspeed;
	    		}
			}
			actualspeed = display_rpm;
		}
	*/
	#endif
	//#endif

	    //----------------------------------------------------------------------------------
	    #ifdef MH_TRY_DYNAMIC_CTL_RATE
	    int Ctl_dyn_rpm_per_tick_x10;
	    int Ctl_fine_rpm_per_tick_x10;
	    int Ctl_coarse_rpm_per_tick_x10;
	    #define CTL_RPM_TBL_MAX	100
	    int Ctl_rpm_per_tick_t[CTL_RPM_TBL_MAX];
	    int Ctl_tick_t[CTL_RPM_TBL_MAX];
	    int Ctl_debug_count;
	    void Ctl_debug(void)
	    {
	    	Ctl_debug_count++;
	    }
	    void Ctl_add_rpm_per_tick_tbl(void)
	    {
	    	int i;

	    	if(Ctl_count_ticks > 10000)
	    	{
	    		Ctl_debug();
	    	}

	    	for(i=CTL_RPM_TBL_MAX-2;i>=0;i--)	// Ripple move tables
	    	{
	    		Ctl_rpm_per_tick_t[i+1] = Ctl_rpm_per_tick_t[i];
	    		Ctl_tick_t[i+1] = Ctl_tick_t[i];
	    	}
	    	Ctl_rpm_per_tick_t[0] = Ctl_dyn_rpm_per_tick_x10;
	    	Ctl_tick_t[0] = Ctl_count_ticks;
	    }
	    //----------------------------------------------------------------------------------
	    int Ctl_mh_count;
	    void Ctl_calculate_rpm_per_tick_rate(void)
	    {
	    /*
	    	Ctl_history_total_rpm_change = Calculated RPM change in pipeline. +ve for FINE, negative for COARSE

	    	current_rpm = initial_rpm + (pwm_ticks * rpm_per_tick) - (pipeline_pwm_ticks * rpm_per_tick)
	    	or
	    	current_rpm = initial_rpm + rpm_per_tick * (pwm_ticks - pipeline_pwm_ticks)

	    	so:

	    	rpm_per_tick_x10 = (current_rpm - initial_rpm)*10*255 /(pwm_ticks - pipeline_pwm_ticks)

	    */
	    	return;

	    	if(cState == C_IDLE)
	    	{
	    		Ctl_mh_count = 0;
	    		return;
	    	}
	    	if(Ctl_mh_count++ == 0)
	    	{
	    		Ctl_pwm_ticks = lastSpeed;	// As this called after pwm table logic.
	    		Ctl_count_ticks = 0;
	    		Ctl_start_rpm = Get_RPM();
	    		return;
	    	}

	    	int current_rpm = Get_RPM();
	    	int rpm_change  =  current_rpm - Ctl_start_rpm;
	    	if(rpm_change == 0)
	    	{
	    		return;
	    	}
	    	int rpm_change_adjusted = rpm_change * 10 * 255;
	    	int pwm_used_ticks = Ctl_pwm_ticks - Ctl_pipeline_pwm_ticks;
	    	if(pwm_used_ticks == 0)
	    	{
	    		return;
	    	}
	    	if(pwm_used_ticks < 0)
	    	{
	    		Ctl_debug();
	    	}
	    	Ctl_dyn_rpm_per_tick_x10 = rpm_change_adjusted / pwm_used_ticks;
	    	sprintf(Ctl_pbuff,"Ticks:%d,Tot_pwm:%d,Pipe_pwm:%d,Used_pwm:%d,rpm_chg:%d,Dyn_ctl:%d\r\n",
	    					Ctl_count_ticks,Ctl_pwm_ticks,Ctl_pipeline_pwm_ticks,pwm_used_ticks,rpm_change,Ctl_dyn_rpm_per_tick_x10);
	    	txDebug(Ctl_pbuff);




	    	if(Ctl_dyn_rpm_per_tick_x10 == 0)
	    	{
	    		return;
	    	}
	    	if(Ctl_pwm_ticks > 0)		// Going FINE?
	    	{
	    		Ctl_fine_rpm_per_tick_x10 = Ctl_dyn_rpm_per_tick_x10;
	    	}
	    	else
	    	{
	    		Ctl_coarse_rpm_per_tick_x10 = Ctl_dyn_rpm_per_tick_x10;
	    	}
	    // Note:Could add to table.
	    	Ctl_add_rpm_per_tick_tbl();
	    }
	    #endif

#if MAP_VERSION
        // Check for an OverPressure if manifold pressure sensor enabled
        if ((manifoldPressureEnabled()) &&
            (mode != TAKEOFF) &&
            (mode != CLIMB) &&
            (mode != FEATHER))
        {
            mp = manifoldPressureCheck();
            if (mp == PRESSURE_LIMIT_EXCEEDED)
            {
                manifoldPressureExceeded = 1;
                // Drive Fine
                cState = C_FINER;
                currentControl = 255;
                return;
            }
            else if (mp == PRESSURE_IN_WARN)
            {
                // stop control in this band if we have returned from an exceeded limit
                if (manifoldPressureExceeded)
                {
                    Set_cState(C_IDLE);
                    currentControl = 0;
                    return;
                }
            }
            else
            {
                manifoldPressureExceeded = 0;
            }
        }
        else
        {
            manifoldPressureExceeded = 0;
        }
#endif
#ifdef MH_XXX
	if(Ctl_tune_count <= CTL_MAX_RPM_HISTORY) // Wait until table is full
	{
		return;
	}
	if(++Ctl_tune_timer > 15) // 3 seconds
	{
		if(remoteSetSpeed == getParameter(SP_TAKEOFF))	// going finer?
		{
#ifdef AC210_PORT
//			printf("Ctl_report_rate_change: changing to coarse");
#endif
			remoteSetSpeed = getParameter(SP_CRUISE);	// Now go other way
			Ctl_init_tune();	// reset totals
		}
		else
		{
#ifdef AC210_PORT
//			printf("Ctl_report_rate_change: finished tune");
#endif
			Remote_end_tune();
			AT_Ctl_tune = 0;		// Turn flag off
		// Could signal to diagnostics that have finished.
			txDebug(":A:END\r\n");
		}
		return;
	}

	Ctl_act_rpm_rate_change = Ctl_rpm_change_rate(&Ctl_rpm_hist);
	Ctl_act_rpm_rate_change_fine = MIN(Ctl_act_rpm_rate_change_fine,Ctl_act_rpm_rate_change);
	Ctl_act_rpm_rate_change_coarse = MAX(Ctl_act_rpm_rate_change_coarse,Ctl_act_rpm_rate_change);

	strcpy(Ctl_pbuff,Ctl_fmt_dec(Ctl_act_rpm_rate_change));
	strcat(Ctl_pbuff,Ctl_fmt_dec(Ctl_act_rpm_rate_change_fine));
	strcat(Ctl_pbuff,Ctl_fmt_dec(Ctl_act_rpm_rate_change_coarse));
	strcat(Ctl_pbuff,"\r\n");
	txDebug(Ctl_pbuff);
#endif
#if MAP_VERSION
        if (mode == MAP)
        {
            // Map control.. ignore set points and work to MAP function
            mp = manifoldPressureCheck();
            // doesn't get here for OverPressure, or if in warn range and has been OverPressure
            // for now, use on/off control
            switch (mp)
            {
            case PRESSURE_BELOW_MAP:
                cs = C_COARSER;
                control = 255;
                break;

            case PRESSURE_IN_MAP:
                cs = C_IDLE;
                control = 0;
                break;

            case PRESSURE_ABOVE_MAP:
            case PRESSURE_IN_WARN:
                cs = C_FINER;
                control = 255;
                break;
            }
        }
        else
#endif
#ifndef AC210_PORT
            /* New implementation of PID controller is as follows.
               The PID equation used is

               O(k) = O(k-1)
                      - PF[s(k) - s(k-1)]
                      + IF[e(k)]
                      - DF[s(k) - 2s(k-1) + s(k-2)]
               PF = proportional "gain"
               IF = Integral "gain"
               DF = differential "gain"

               (k) = this sample, (k-1) = prev sample, (k-2) = prev-prev sample
               s = speed
               e = error

               This version of the PID equation uses speed in the Prop and Diff bits
               in preference to error to avoid large disturbances when set point is
               changed.
               The output value (O(k)) is limited to +=255 at the end of the loop
               prior to being saved in the history.

             */

            /* current speed is already in rpmHistory[0] */
            err = setSpeed - rpmHistory[0];
            if (err > (long)deadband)
            {
                err -= deadband;
                cs = C_FINER;
            }
            else if (err < -(long)deadband)
            {
                err += deadband;
                cs = C_COARSER;
            }
            else
            {
                cs = C_IDLE;
            }

            lastErrorValue = err;


            for (cnt = CONTROL_HISTORY-1; cnt > 0 ; cnt--)
            {
                controlHistory[cnt] = controlHistory[cnt-1];
            }

            if (cs != C_IDLE)
            {
                // 3 dp because factors are 2dp and rpm is 1dp
                lastProportionalValue = propFactor * (rpmHistory[0] - rpmHistory[sticks]);
                lastIntegralValue = intFactor * err;
                lastDifferentialValue = diffFactor * (rpmHistory[0] - 2*rpmHistory[sticks] + rpmHistory[2*sticks]);

                controlHistory[0] = controlHistory[1] +
                                    ((lastIntegralValue - lastProportionalValue - lastDifferentialValue)/1000);
            }
            else
            {
                lastProportionalValue = 0;
                lastDifferentialValue = 0;
                lastIntegralValue = 0;

                controlHistory[0] = 0;
            }



            /* Limits */
            if (controlHistory[0] > 255L)
            {
                controlHistory[0] = 255L;
            }
            else if (controlHistory[0] < -255L)
            {
                controlHistory[0] = -255L;
            }

            if (debug & DEBUG_PI_CONTROL)
            {
                sprintf (controlDebugBuf, "CLP,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
                                        cs,
                                        rpmHistory[0],
                                        rpmHistory[sticks],
                                        rpmHistory[2*sticks],
                                        lastErrorValue,
                                        lastProportionalValue,
                                        lastIntegralValue,
                                        lastDifferentialValue,
                                        controlHistory[0]);
                txDebug (controlDebugBuf);

            }

            /*
               If we are driving Fine (ie below set speed) then only take note of
               -ve control values to drive the motor, +ve values are there for looks
               If we are driving Coarse (ie above set speed) then only take note of
               +ve control values to drive the motor
            */
            if (cs == C_FINER)
            {
                if (controlHistory[0] > 0)
                {
                    control = controlHistory[0];
                }
                else
                {
                    // do not drive.......
                    control = 0;
                    cs = C_IDLE;
                }
            }
            else if (cs == C_COARSER)
            {
                if (controlHistory[0] < 0)
                {
                    control = -controlHistory[0];
                }
                else
                {
                    // do not drive.......
                    control = 0;
                    cs = C_IDLE;
                }
            }
#endif
#if 0
        if (debug & DEBUG_PI_CONTROL)
        {
            if (controlMode == PROP_MODE)
            {
                sprintf (controlDebugBuf, "CLOOP,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
                                            rpmHistory[0],
                                            rpmHistory[1],
                                            lastErrorValue,
                                            lastProportionalValue,
                                            lastIntegralValue,
                                            lastDifferentialValue,
                                            controlHistory[0]);
            }
            else
            {
                sprintf (controlDebugBuf, "Control-> %d (%ld)(%ld)\r\n", cs, err, control);
            }
            txDebug (controlDebugBuf);
        }
#endif
#ifdef MH_OLD_CTL_LOGIC		// MHH:21/11/2020
	if(Ctl_rpm_per_tick_x10 == 0)
	{
    // Always update the rpmHistory
	    for (cnt = RPM_HISTORY-1; cnt > 0 ; cnt--)
    	{
        	rpmHistory[cnt] = rpmHistory[cnt-1];
    	}
    	rpmHistory[0] = actualSpeed;
	}
#endif
#ifdef MH_XXX
	uint8_t flags = SIG100_input_packet[1];	// Common position for short and long pkt.
//	uint8_t dir = (flags >> 5) & DIR_MASK;
	bool motor_enabled = ((flags & MOTOR_ENABLED_BIT) != 0);

	if(Sig100_motor_timer)
	{
//		bool pos_changed = ((flags & POS_CHANGE_BIT) != 0);
//		if(pos_changed)
		if(Encoder_Pos != Sig100_epos)
		{
			Sig100_motor_timer = false;
			int msecs = Timer_read_ms(&Motor_timer);
#ifdef MH_XXX
			if(Sig100_epos == Encoder_Pos)
			{
				DPRINTF("SIG100_check_flags:Sig100_epos=Encoder_Pos =%d\r\n",Encoder_Pos);
			}
#endif
//			DPRINTF("SIG100_check_flags:C=%d,msecs=%d\r\n",BL_command,msecs);
		}
#ifdef MH_XXX
		else
		{
			if(Sig100_epos != Encoder_Pos)
			{
				DPRINTF("SIG100_check_flags:Sig100_epos=%d.Encoder_Pos =%d\r\n",Sig100_epos,Encoder_Pos);
			}
		}
#endif
	}

	switch(BL_command)
	{
	case '.':		// Most common
		if(dir != DIR_STOPPED)
		{
			DPRINTF("Sig100_check_flags:C=.,dir=%d\r\n",dir);
		}
		if(motor_enabled)
		{
			DPRINTF("Sig100_check_flags:C=.,Motor enabled!\r\n",dir);
		}
		return;

	case '+':
		if((flags & 2) == 0)
		{
			if(dir != DIR_COARSE)
			{
				DPRINTF("Sig100_check_flags:C=+,dir=%d\r\n",dir);
			}
			if(motor_enabled==false)
			{
				DPRINTF("Sig100_check_flags:C=+,Motor NOT enabled!\r\n",dir);
			}
		}
		return;

	case '-':
		if((flags & 1) == 0)
		{
			if(dir != DIR_FINE)
			{
				DPRINTF("Sig100_check_flags:C=-,dir=%d\r\n",dir);
			}
			if(motor_enabled==false)
			{
				DPRINTF("Sig100_check_flags:C=-,Motor NOT enabled!\r\n",dir);
			}
		}
		return;
	}
#endif
#ifdef MH_XXX
void Hub_calibrate_check(void)
{
	WORD current=0;
	int hub_size,pos_from_chs,pos_offset_from_fine_ms;

	if(Hub_calibrate_state == CAL_IDLE)
	{
		return;
	}

	if(Hub_calibrate_checkval != 12345)
	{
		Abort( AC210_SRC_SIG100+10,"Hub_calibrate:checkval");
	}

	switch(Hub_calibrate_state)
	{
	default:
	case CAL_IDLE:
		Hub_calibrate_checkval = 0;
		return;
	case CAL_START:
		DPRINTF("Calibrate:Moving to COARSE hard stop,pos:%d\r\n",Encoder_Pos);
		Cal_fine_ms_stop_found = false;
		Hub_calibrate_state = CAL_COARSE_HARDSTOP;
		Cal_current_count = 0;
		Cal_flags_saved = 0;
		Hub_calibrate_last_ms_stop_pos=0;
		Hub_return_pos();
		return;

	case CAL_COARSE_HARDSTOP:
		current = scaledValue (A_MOTOR_CURRENT);
//		DPRINTF("curr1=%d\r\n",current);
#ifdef MH_XXX

		if(Cal_current_count < 50)	// Give a sec to start
		{
			Cal_current_count++;
			Cal_current = current;
			Cal_current_max = 0;
		}
		else
		{
			if(Cal_current_max == 0)
			{
				Cal_current_max = (Cal_current * 3);		// +50%
			}
			if(current > 2000)		// Arbitrary - could be 50 % more
			{
				Cal_coarse_hard_pos = Encoder_Pos;
//				Hub_calibrate_state = CAL_COARSE_HARDSTOP_FINE;
				Hub_calibrate_state = CAL_FINE_HARDSTOP;
				Cal_current_count = 0;
//				PRINTF("Calibrate:Reached COARSE hard stop, moving to FINE hard stop,pos:%d,Cal_cur:%d,cur:%d\r\n",Encoder_Pos,Cal_current,current);
				PRINTF("CAL:C=%d\r\n",Cal_coarse_hard_pos);
				DPRINTF("CHS:Cal_current_max:%d,current:%d\r\n",Cal_current_max,current);
			}
		}
#endif
		if(current > MAX_CURRENT)		// Arbitrary - could be 50 % more
		{
			Cal_coarse_hard_pos = Encoder_Pos;
			Hub_calibrate_save_enc_pos = 0;
//				Hub_calibrate_state = CAL_COARSE_HARDSTOP_FINE;
			Hub_calibrate_state = CAL_FINE_HARDSTOP;
			Cal_current_count = 0;
			Cal_hardstop_count = 0;
			Cal_hardstop_sent = false;
//				PRINTF("Calibrate:Reached COARSE hard stop, moving to FINE hard stop,pos:%d,Cal_cur:%d,cur:%d\r\n",Encoder_Pos,Cal_current,current);
//			PRINTF("CAL:C=%d\r\n",Cal_coarse_hard_pos);
//			DPRINTF("CHS:Cal_current_max:%d,current:%d\r\n",Cal_current_max,current);
		}

		Hub_return_pos();
		return;

	case CAL_FINE_HARDSTOP:
		if(Cal_hardstop_sent == false)
		{
			if(Cal_coarse_hard_pos - Encoder_Pos < 50)
			{
				if(Encoder_Pos == Hub_calibrate_save_enc_pos)
				{
					if(Cal_hardstop_count++ >= 5)
					{
						Cal_coarse_hard_pos = Encoder_Pos;
						Cal_hardstop_sent = true;
						PRINTF("CAL:C=%d\r\n",Cal_coarse_hard_pos);
					}
				}
				else
				{
					Hub_calibrate_save_enc_pos = Encoder_Pos;
					Cal_hardstop_count = 0;
				}
				DPRINTF("CAL:C=%d\r\n",Encoder_Pos);
			}
			else
			{
				DPRINTF("CAL_FINE_HARDSTOP:Did not find 5 same positions\r\n")
				PRINTF("CAL:C=%d\r\n",Cal_coarse_hard_pos);
				Cal_hardstop_sent = true;
			}
		}
		// Could test for coarse microswitch changes as we move fine and record. Note that we should do it on pin change to be accurate, not wait for Systick confirmation
		if(Cal_flags_saved != Cal_flags)
		{
			if(Hub_calibrate_last_ms_stop_pos != 0)
			{
				DPRINTF("Encoder_Pos=%d,ms_stop_pos=%d\r\n",Encoder_Pos,Hub_calibrate_last_ms_stop_pos);
				Encoder_Pos = Hub_calibrate_last_ms_stop_pos;
				Hub_calibrate_last_ms_stop_pos = 0;
			}

			if((Cal_flags_saved & 2) && ((Cal_flags & 2) == 0))
			{
				Cal_coarse_pos = Encoder_Pos;
				Cal_coarse_ms_offset = Cal_coarse_hard_pos - Encoder_Pos;	// Do we need to know this?

			}
			Cal_flags_saved = Cal_flags;

			//			PRINTF("Cal_flags:%d,pos:%d\r\n",Cal_flags,Encoder_Pos);
			PRINTF("CAL:S=%d,%d\r\n",Cal_flags,Encoder_Pos);
			if(Cal_flags & 1)
			{
				if(Cal_fine_ms_stop_found == false)
				{
					Cal_fine_ms_stop_found = true;
					Cal_fine_pos = Encoder_Pos;
					Cal_fine_ms_offset = Cal_coarse_hard_pos - Encoder_Pos;	// Do we need to know this?
				}
//				PRINTF("Fine MS offset:%d\r\n",Cal_coarse_hard_pos - Encoder_Pos);
			}
		}
		current = scaledValue (A_MOTOR_CURRENT);
//		DPRINTF("curr2=%d\r\n",current);

		if(Cal_current_count < 50)
		{
			Cal_current_count++;
		}
		else
		{
			if(current > MAX_CURRENT)		// Arbitrary - could be 50 % more
			{
				Cal_fine_hard_pos = Encoder_Pos;
				Hub_calibrate_save_enc_pos = 0;
				//				Hub_calibrate_state = CAL_FINE_HARDSTOP_COARSE;
				Hub_calibrate_state = CAL_RETURN_FINE_STOP;
				Cal_current_count = 0;
				Cal_hardstop_count = 0;
				Cal_hardstop_sent = false;
				//				PRINTF("Calibrate:Reached FINE hard stop, moving to FINE stop,pos:%d,Cal_cur:%d,cur:%d\r\n",Encoder_Pos,Cal_current,current);
//				PRINTF("CAL:F=%d\r\n",Cal_fine_hard_pos);
				//				PRINTF("Size:%d, Fine microswitch:%d\r\n",Cal_coarse_hard_pos - Encoder_Pos,Cal_fine_ms_offset);
//				DPRINTF("FHS:Cal_current_max:%d,current:%d\r\n",Cal_current_max,current);
			}

		}

		Hub_return_pos();
		return;

	case CAL_RETURN_FINE_STOP:
		if(Cal_hardstop_sent == false)
		{
			if(Encoder_Pos - Cal_fine_hard_pos < 50)
			{
				if(Encoder_Pos == Hub_calibrate_save_enc_pos)
				{
					if(Cal_hardstop_count++ >= 5)
					{
						Cal_fine_hard_pos = Encoder_Pos;
						Cal_hardstop_sent = true;
						PRINTF("CAL:F=%d\r\n",Cal_fine_hard_pos);
					}
				}
				else
				{
					Hub_calibrate_save_enc_pos = Encoder_Pos;
					Cal_hardstop_count = 0;
				}
				DPRINTF("CAL:F=%d\r\n",Encoder_Pos);
			}
			else
			{
				DPRINTF("CAL_RETURN_FINE_STOP:Did not find 5 same positions\r\n")
				PRINTF("CAL:F=%d\r\n",Cal_fine_hard_pos);
				Cal_hardstop_sent = true;
			}
		}

//		if(Encoder_Pos > Cal_fine_pos)
		if(Encoder_Pos < Cal_fine_pos)	// MHH:05/07/2023
		{
			Hub_calibrate_state = CAL_FINISH;
			Cal_last_pos = Encoder_Pos;
			Cal_current_count = 0;
			//			PRINTF("Calibrate:Reached FINE stop,pos:%d\r\n",Encoder_Pos);
		}
		Hub_return_pos();
		return;

	case CAL_FINISH:
		// May have some tidying and parameter setting, but for first cut just move coarse until passed fine stop
		if(Encoder_Pos == Cal_last_pos)
		{
			if(Cal_current_count++ > 10)
			{
				Hub_calibrate_state = CAL_IDLE;
				Hub_calibrate_checkval = 0;
//				PRINTF("Calibrate:Finished calibration,pos:%d\r\n",Encoder_Pos);
				PRINTF("CAL:X=%d\r\n",Encoder_Pos);

				hub_size = Cal_coarse_hard_pos - Cal_fine_hard_pos;
				pos_from_chs = Cal_coarse_hard_pos - Encoder_Pos;
				pos_offset_from_fine_ms = Encoder_Pos - Cal_fine_pos;
				DPRINTF("CAL:X=%d\r\n",Encoder_Pos);
				DPRINTF("pos_from_chs=%d\r\n",pos_from_chs);
				DPRINTF("hub_size    =%d\r\n",hub_size);
				DPRINTF("Coarse_hard_pos = %d\r\n",Cal_coarse_hard_pos);
				DPRINTF("Fine_hard_pos   = %d\r\n",Cal_fine_hard_pos);
				DPRINTF("Cal_fine_ms_offset  = %d\r\n",Cal_fine_ms_offset);
				DPRINTF("Cal_coarse_ms_offset= %d\r\n",Cal_coarse_ms_offset);
				DPRINTF("pos_offset from fine_ms = %d\r\n",pos_offset_from_fine_ms);
				Hub_return_position = true;
			}
		}
		else
		{
			Cal_current_count = 0;
			Cal_last_pos = Encoder_Pos;
		}
		Hub_return_pos();
		return;
	}
}
#endif
#endif



