/*
 * ac210_uuencode.c
 *
 *
 * Derived from mbed:mhlpcbootloader.cpp
 *
 *  Created on: 20/12/2016
 *      Author: Murray
 */

#include <stdio.h>
#include <string.h>

#ifdef AC210_PORT
#include "ac210_global.h"
#else
#include "global.h"
#endif
#include "uuencode.h"

int UU_Sum20;

uint8_t UU_fBuff[UU_FBUFF_SIZE];

uint8_t UU_iBuf[48];
uint8_t UU_iBuf2[48];
uint8_t UU_uBuf[64];
//uint8_t UU_uBuf2[64];

static far uint8_t *ibp;
static far uint8_t *ubp;

#define MASK6BITS   0x3f
static void encode3(void)
{
    uint32_t b3;
    uint8_t c;
    int i;
//    uint32_t b3_save;
//	char printbuf[50];

//    b3 = (ibp[0]<<16) | (ibp[1] << 8) | ibp[2];

	b3 = ibp[0];
	b3 <<= 8;
	b3 |= ibp[1];
	b3 <<= 8;
	b3 |= ibp[2];

//	b3_save = b3;

    for(i=3;i>=0;i--) {
        c = (b3 & MASK6BITS)+0x20;
        if(c == 0x20) c = 0x60; // null special case
        ubp[i] = c;
        b3 >>= 6;
    }

//    printf("Encode3: %c%c%c to %s\n\r",ibp[0],ibp[1],ibp[2],ubp);

#ifdef MH_XXX
	sprintf(printbuf,"Encode3:%2x,%2x,%2x,%lx,%2x,%2x,%2x,%2x\r\n",
			ibp[0],ibp[1],ibp[2],b3_save,ubp[0],ubp[1],ubp[2],ubp[3]);

	txDebug(printbuf);
#endif

    ibp +=3;
    ubp +=4;
}
//--------------------------------------------------------------------------------------------------
WORD UUEncodeLine(far uint8_t *fsrc,int len)
{
	int clen;
	int loops;
	int i;
	WORD ret_len;

//	uint8_t b1,b2;
//	char printbuf[20];


	ibp = UU_iBuf;
    ubp = UU_uBuf;

    clen = MIN(len,45);
    memcpy(UU_iBuf,fsrc,clen);

//#define MH_XXX
#ifdef MH_XXX
    for(i=0;i<8;i++)
    {
		b1 = UU_iBuf[i];
		b2 = fsrc[i];
		sprintf(printbuf,"%d:%02x,%02x\r\n",i,b1,b2);
		txDebug(printbuf);
	}
#endif

    *ubp++ = (uint8_t) (len+32);    // first byte = len +32

    if(len != 45)
    {
        if(len > 45)
        {
        	DebugAbort("UUEncodeLine:bad len");
            return 0;
        }
        UU_iBuf[len] = 0;		// Make sure last 2 bytes are zero
        UU_iBuf[len+1] = 0;
    }
    loops = (len+2)/3;
    for(i=0;i<loops;i++) { // for each 3 byte group

        encode3();
    }
//    *ubp++ = 0x0D;      // append <CR><LF><null>
//    *ubp++ = 0x0A;
    ret_len = (ubp - UU_uBuf);		// Ignore null for length
    *ubp++ = 0;
    return ret_len;
}
//--------------------------------------------------------------------------------------------------
void UUChecksumLine(int ilen)
{
    int i;
    far uint8_t *bp = UU_iBuf;
    for(i=0;i<ilen;i++) UU_Sum20 += *bp++;
}
//------------------------------------------------------------------------------------------------------
//extern uint8_t Rbuffer[64];           // Receive buffer. Note: possibly use UU_iBuf instead??
//--------------------------------------------------------------------------------------------------
static void decode3(void)
{
    long b3;
    int c;
    int i;
// Exact reverse of encode

    b3 = 0;
    for(i=0;i<4;i++) {
        c = ubp[i] - 0x20;
        if(c == 0x40) c = 0;    // null special case
        b3 <<= 6;
        b3 |= c;
    }

// We now have a 24 bit value which we will split into 3 8 bit values.

    ibp[2] = (b3 & 0xff);
    b3 >>= 8;
    ibp[1] = (b3 & 0xff);
    b3 >>= 8;
    ibp[0] = (b3 & 0xff);
    UU_Sum20 += (ibp[0] + ibp[1] + ibp[2]);
//    printf("Decode3: %2x %2x %2x [%c%c%c%c] (%d)\n\r",ibp[0],ibp[1],ibp[2],ubp[0],ubp[1],ubp[2],ubp[3],Sum20);

    ibp +=3;
    ubp +=4;
}
//----------------------------------------------------------------------------------------------------------------
int UUDecodeLine(void)
{
// Expect line to be multiples of 4 coded bytes which we will convert to 3 decoded bytes

// Note: Could just set ubp to point to Rbuffer, no need to strcpy

//	strcpy(UU_uBuf,pCodedLine);		// Assume already copied in for test

	int slen;
    int len;
	int loops;
	int i;

    ibp = UU_iBuf;
    ubp = UU_uBuf;

    slen = strlen((far char *)ubp);
    len = (*ubp++) - 32;
    if(len == 45) {
        if(slen != 61) {
//            printf("UUDecodeLine: len = %d, slen = %d\n\r",len,slen);
            DebugAbort("UUDecodeLine:1");
            return 0;
        }
    } else {
        if(len == 34) {
            if(slen != 49) {
//                printf("UUDecodeLine: len = %d, slen = %d\n\r",len,slen);
            }
        } else {
//            printf("UUdecodeLine: len (%d) <> 45 or 34\n\r",len);
            DebugAbort("UUDecodeLine:2");
            return 0;
        }
//        UU_iBuf[34] = 0;   // Ensure last 2 bytes null for 34 byte option.
//        UU_iBuf[35] = 0;
    }
    loops = (len+2)/3;
    for(i=0;i<loops;i++) { // for each 3 byte group
        decode3();
    }
    return len;
}
//----------------------------------------------------------------------------------------------------
#ifdef AC210_PORT
void UUCheckEncode(uint8_t *fsrc,int copylen)
{
    memcpy(UU_iBuf2,fsrc,copylen);	// save for compare
    UUEncodeLine(fsrc,copylen);		// iBuf -> uBuf
//    memcpy(UU_uBuf2,UU_uBuf,sizeof(UU_uBuf2));
    UUDecodeLine();
    for(int i=0;i<copylen;i++)
    {
    	uint8_t c1 = UU_iBuf[i];
    	uint8_t c2 = UU_iBuf2[i];
    	if(c1 != c2)
    	{
    		printf("Differ at %d, c1=%02x,c2=%02x\r\n",i,c1,c2);
    		DebugAbort("UUCheckEncode");
    	}
    }
}
#endif
