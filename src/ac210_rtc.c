/*
 * ac210_rtc.c
 *
 *  Created on: 29/12/2018
 *      Author: Murray
 */

#include "ac210_global.h"
#include "ac210_rtc.h"
#include "param.h"

//--------------------------------------------------------------------------
uint8_t nn_from_bcd(uint8_t bcd)
{
	uint8_t nn = (bcd & 0xf) + (bcd >> 4) * 10;
	return nn;
}
//--------------------------------------------------------------------------
uint32_t Get_yymmdd_from_bcd(void)
{
	const uint8_t *bp = ds3231_buff_in;

	uint8_t dd = nn_from_bcd(bp[4]);
	uint8_t mo = nn_from_bcd(bp[5]);
	uint8_t yy = nn_from_bcd(bp[6]);
	uint32_t yymmdd = yy * 10000 + mo * 100 + dd;
	return yymmdd;
}
//--------------------------------------------------------------------------
uint32_t Get_hhmmss_from_bcd(void)
{
	const uint8_t *bp = ds3231_buff_in;

	uint8_t ss = nn_from_bcd(bp[0]);
	uint8_t mm = nn_from_bcd(bp[1]);
	uint8_t hh = nn_from_bcd(bp[2]);		// works if in 24 hour format
	uint32_t hhmmss = hh * 10000 + mm * 100 + ss;
	return hhmmss;
}
//--------------------------------------------------------------------------
uint8_t nn_to_bcd(uint8_t nn)
{
	uint8_t bcd = (nn % 10) + ((nn/10) << 4);		// to bcd
	return bcd;
}
//--------------------------------------------------------------------------
void Set_date_time(uint32_t yymmdd,uint32_t hhmmss)
{
	uint8_t buff_out[7];

	uint8_t *bp = buff_out;

	uint32_t w = yymmdd;
	uint8_t dd = w % 100;
	w /= 100;
	uint8_t mo = w % 100;
	w /= 100;
	uint8_t yy = w;

	w = hhmmss;
	uint8_t ss = w % 100;
	w /= 100;
	uint8_t mm = w % 100;
	w /= 100;
	uint8_t hh = w;

	bp[0] = nn_to_bcd(ss);
	bp[1] = nn_to_bcd(mm);
	bp[2] = nn_to_bcd(hh);
	bp[3] = 1;		// Day of week. Not interested as can calculate.

	bp[4] = nn_to_bcd(dd);
	bp[5] = nn_to_bcd(mo);
	bp[6] = nn_to_bcd(yy);

	DS3231_i2c_write(buff_out, 0, 7);
}
//--------------------------------------------------------------------------
//                             1  2  3  4  5  6  7  8  9 10 11 12
//                             J  F  M  A  M  J  J  A  S  O  N  D
const uint8_t Month_days[12]={31,28,31,30,31,30,31,31,30,31,30,31};
uint32_t Get_days_after_2000(uint32_t yymmdd)
{
	uint32_t w = yymmdd;

	uint8_t dd = w % 100;
	w /= 100;
	uint8_t mo = w % 100;
	w /= 100;
	uint8_t yy = w;

	uint8_t y = yy % 4;		// leap_year?

	uint32_t days = dd -1;
	for(int m=1;m<mo;m++)
	{
		uint8_t mdays = Month_days[m-1];
		if(m == 2 && y == 0) mdays = 29;
		days += mdays;
	}
	days += yy * 365;

	if(yy > 0)
	{
		int leap_years = 1;	// Because 2000 a leap year
		leap_years +=  (yy - 1)/4;	// so, add 1 at year 5-8, add 2 at 9-12 etc. Rest is handled in months routine
		days += leap_years;
	}

	return days;
}
//--------------------------------------------------------------------------
uint32_t Get_secs_from_hhmmss(uint32_t hhmmss)
{
	uint32_t w = hhmmss;
	uint8_t ss = w % 100;
	w /= 100;
	uint8_t mm = w % 100;
	w /= 100;
	uint8_t hh = w;

	uint32_t secs = ss + mm * 60 + hh * 3600;
	return secs;
}
//--------------------------------------------------------------------------
uint32_t Get_yymmdd_from_secs(uint32_t secs_since_2000)
{
	uint32_t days = secs_since_2000 / (24 * 3600);
	uint32_t leap_years = (days / (365 * 4 + 1));

	uint32_t leapdays_remainder = days % (365 * 4 + 1);

	uint32_t ld_rem = leapdays_remainder;
	int days_in_year = 366;		// First time only;
	int y;
	for(y=0;y<4;y++)
	{
		if(days_in_year > ld_rem) break;

		ld_rem -= days_in_year;
		days_in_year = 365;
	}
	uint32_t yy = leap_years * 4 + y;

	uint32_t yy_days = ld_rem + 1;
	int m;
	for(m=1;m<=12;m++)
	{
		uint8_t mdays = Month_days[m-1];
		if(y == 0 && m == 2) mdays = 29;		// leap year and month
		if(mdays >= yy_days) break;

		yy_days -= mdays;
	}
	uint8_t mo = m;
	uint8_t dd = yy_days;
	uint32_t yymmdd = yy * 10000 + mo * 100 + dd;

	return yymmdd;
}
//--------------------------------------------------------------------------
uint32_t Get_hhmmss_from_secs(uint32_t secs_since_2000)
{
	uint32_t day_secs = secs_since_2000 % (24 * 3600);
	uint32_t w = day_secs;
	uint32_t ss = w % 60;
	w /= 60;
	uint32_t mm = w%60;
	w /= 60;
	uint32_t hh = w;
	uint32_t hhmmss = hh * 10000 + mm * 100 + ss;
	return hhmmss;
}
//--------------------------------------------------------------------------
uint32_t Get_secs_since_2000_from_bcd(void)
{
    uint32_t yymmdd = Get_yymmdd_from_bcd();
    uint32_t hhmmss = Get_hhmmss_from_bcd();
    uint32_t days = Get_days_after_2000(yymmdd);
    uint32_t secs = Get_secs_from_hhmmss(hhmmss);
    secs += days * (24 * 3600);
    return secs;
}
//--------------------------------------------------------------------------
void RTC_Display_date_time(void)
{
	const uint8_t *bp = ds3231_buff_in;

	uint8_t ss = nn_from_bcd(bp[0]);
	uint8_t mm = nn_from_bcd(bp[1]);
	uint8_t hh = nn_from_bcd(bp[2]);		// works if in 24 hour format
//	printf("Time(hh:mm:ss): %02d:%02d%:%02d\r\n",hh,mm,ss);
//	uint8_t day_of_week = bp[3];
//	printf("day_of_week: %d\r\n",day_of_week);
	uint8_t dd = nn_from_bcd(bp[4]);
	uint8_t mo = nn_from_bcd(bp[5]);
	uint8_t yy = nn_from_bcd(bp[6]);
	PRINTF("Date:%02d/%02d/%02d, Time:%02d:%02d%:%02d\r\n",yy,mo,dd,hh,mm,ss);
//	Set_date_time(181230,214201);

	if(yy == 0)
	{
//		Set_date_time(181229,150201);
	}
}
//--------------------------------------------------------------------------
uint32_t RTC_start_secs;
uint32_t RTC_secs;
ULONG AC210_RTC_secs(void)
{
	if(RTC_start_secs == 0)
	{
		return 0;
	}
	return RTC_secs;
}
//--------------------------------------------------------------------------
// Note: Should Error if RTC not available
WORD AC210_set_rtc(uint32_t utc_secs_since_2000)
{
	uint32_t yymmdd = Get_yymmdd_from_secs(utc_secs_since_2000);
	uint32_t hhmmss = Get_hhmmss_from_secs(utc_secs_since_2000);
	Set_date_time(yymmdd,hhmmss);
	RTC_start_secs = utc_secs_since_2000;
	RTC_secs = RTC_start_secs;
	return 0;
}
//--------------------------------------------------------------------------
void AC210_RTC_init(void)
{
#ifdef MH_RTC

	RTC_start_secs = 0;

	if(getParameter(RTC_ENABLE) != 1)
	{
		return;
	}
	if(Board_test_for_DS3231() != 2)
	{
		return;
	}
    if(DS3231_i2c_init()== 0)
    {
//    	RTC_Display_date_time();
        RTC_start_secs = Get_secs_since_2000_from_bcd();
        RTC_secs = RTC_start_secs;
    }
#endif
}

