/*
 * AC210_uuencode.h
 *
 *  Created on: 20/12/2016
 *      Author: Murray
 */

#ifndef UUENCODE_H_
#define UUENCODE_H_

#ifdef AC210_PORT
#define UU_FBUFF_SIZE 		1024
#else
#define UU_FBUFF_SIZE 		512
#endif
extern int UU_Sum20;
extern uint8_t UU_fBuff[UU_FBUFF_SIZE];
extern uint8_t UU_iBuf[48];
extern uint8_t UU_uBuf[64];

void UUCheckEncode(far uint8_t *fsrc,int copylen);
WORD UUEncodeLine(far uint8_t *fsrc,int len);
void UUChecksumLine(int ilen);

#endif /* UUENCODE_H_ */
