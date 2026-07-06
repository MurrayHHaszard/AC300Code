/*
 * ac200_pins.h
 *
 *  Created on: 3/11/2016
 *      Author: Murray
 */

#ifndef AC210_PINS_H_
#define AC210_PINS_H_


#define PINDEF(a,b)			(a<<5|b)
#define PINMASK(a)			(a&31)
#define PINPORT(a)			(a>>5)

#define PIN_DEF2(a)		PINPORT(a),PINMASK(a)

#define AC210_HW_VERSION_0_PIND		PINDEF(1,0)
#define AC210_HW_VERSION_1_PIND		PINDEF(1,1)
#define AC210_HW_VERSION_2_PIND		PINDEF(1,4)
#define AC210_HW_VERSION_3_PIND		PINDEF(1,8)

#define AC210_RC_PIND				PINDEF(1,9)			// MHH:03/11/2023. To identify Remote Control Board
#define AC210_BL_PIND				PINDEF(1,10)		// MHH:03/11/2023. To identify Brushless only board eg AC210_BL2

#define AC210_LED_0_PIND			PINDEF(2,12)		// Diagnostic led
#define AC210_LED_1_PIND			PINDEF(1,16)
#define AC210_LED_2_PIND			PINDEF(1,17)
#define AC210_LED_3_PIND			PINDEF(2,9)
#define AC210_LED_4_PIND			PINDEF(2,6)
#define AC210_LED_5_PIND			PINDEF(2,8)
#define AC210_LED_6_PIND			PINDEF(2,5)
#define AC210_LED_7_PIND			PINDEF(2,7)
#define AC210_LED_8_PIND			PINDEF(2,4)
#define AC210_LED_9_PIND			PINDEF(2,3)

#define AC210_SWITCH_1_PIND			PINDEF(0,18)
#define AC210_SWITCH_2_PIND			PINDEF(0,19)
#define AC210_SWITCH_3_PIND			PINDEF(0,17)
#define AC210_SWITCH_4_PIND			PINDEF(0,20)
#define AC210_SWITCH_5_PIND			PINDEF(0,15)
#define AC210_SWITCH_6_PIND			PINDEF(0,21)
#define AC210_SWITCH_7_PIND			PINDEF(0,16)

#define AC210_J2_1_STATE_U1_PIND	PINDEF(1,18)
#define AC210_J2_2_STATE_U2_PIND	PINDEF(1,19)
#define AC210_J2_3_STATE_U3_PIND	PINDEF(1,20)
#define AC210_J2_4_STATE_L1_PIND	PINDEF(1,21)
#define AC210_J2_5_STATE_L2_PIND	PINDEF(1,22)
#define AC210_J2_6_STATE_L3_PIND	PINDEF(1,23)

#define AC210_J2_7_MOTOR_STATE_SENSE_PIND	PINDEF(0,23)		// ADC0.0

#define AC210_J2_8_DRIVE_U1_PIND	PINDEF(1,24)
// J2_9 Not Connected
#define AC210_J2_10_DRIVE_U2_PIND	PINDEF(1,25)
#define AC210_J2_11_DRIVE_U3_PIND	PINDEF(1,26)

#define AC210_J2_12_DRIVE_L1_PIND	PINDEF(2,2)		// PWM1.3
#define AC210_J2_13_DRIVE_L2_PIND	PINDEF(2,1)		// PWM1.2
#define AC210_J2_14_DRIVE_L3_PIND	PINDEF(2,0)		// PWM1.1

#define AC210_J2_15_COARSE_SENSE_PIND	PINDEF(1,27)
#define AC210_J2_16_FINE_SENSE_PIND		PINDEF(1,28)
#define AC210_J2_ENABLE_PIND			PINDEF(1,29)			// For Octal buffer

#define AC210_RTC_SDA1_PIND			PINDEF(0,0)
#define AC210_RTC_SCL1_PIND			PINDEF(0,1)
#define AC210_RTC_RESET_PIND		PINDEF(2,13)

#define AC210_SERIAL_TX_PIND		PINDEF(0,2)
#define AC210_SERIAL_RX_PIND		PINDEF(0,3)

#define AC210_J1_MOTOR_CURRENT_SENSE_PIND	PINDEF(0,24)		// ADC0.1
#define AC210_J1_MOTOR_SUPPLY_SENSE_PIND	PINDEF(0,25)		// ADC0.2 (Battery Voltage)
#define AC210_J1_MAG_INT_PIND		PINDEF(0,22)	// Changed for board J

// AUX_1 = 12V

// Note: From 5C we can use AUX Dig3 and Dig4 as either an extra  serial port or a CAN port depending on parameters.
// To do this p0.10 (TXD2) and p0.11 (RXD2) are connected to pins p0.1 (CAN-TD1) and p0.0 (CAN-RD1) respectively.
// If we are not using any set of pins then they should be set to high-Z, though may be enough just to have in read mode.

#define AC210_AUX_2_DIG3_CAN_PIND		PINDEF(0,5)			// #80, P0.5 (TD2)
#define AC210_AUX_2_DIG3_TX2_PIND		PINDEF(0,10)		// #48, P0.10

#define AC210_AUX_3_DIG1_TX3_PIND		PINDEF(4,28)		// #82, P4.28
#define AC210_AUX_4_DIG2_RX3_PIND		PINDEF(4,29)		// #85, P4.29
// AUX_5 = GND
// AUX_6 = 3.3V
#define AC210_AUX_7_ADC_DIMMER_PIND		PINDEF(0,26)		// #6,  P0.26, ADC.3
#define AC210_AUX_8_ADC_PIND			PINDEF(1,30)		// #21, P1.30, ADC.4

#define AC210_ADC5_TEMPERATURE_PIND		PINDEF(1,31)		// #20, P1.31, ADC.5

#define AC210_AUX_9_DIG4_CAN_PIND		PINDEF(0,4)			// #81, P0.4 (RD2)
#define AC210_AUX_9_DIG4_RX2_PIND		PINDEF(0,11)		// #49, P0.11

// AUX_10 = GND

#define AC210_SSP1_SSEL_PIND			PINDEF(0,6)		// Slave select
#define AC210_SSP1_SCK_PIND				PINDEF(0,7)		// Clock
#define AC210_SSP1_MISO_PIND			PINDEF(0,8)		// MISO
#define AC210_SSP1_MOSI_PIND			PINDEF(0,9)		// MOSI

#define AC210_I2C0_DATA_PIND			PINDEF(0,27)			// Used to access EEPROM
#define AC210_I2C0_CLOCK_PIND			PINDEF(0,28)


#ifdef MH_OLD_SIG60_CTL
#define SIG60_HDC_PIN					AC210_AUX_2_DIG3_PIN
#define SIG60_HDC_PORT					0
#else
//#define SIG60_HDC_PIN					PINMASK(AC210_AUX_8_ADC_PIND)	// using AC200 throttle adc read pin.
//#define SIG60_HDC_PORT					PINPORT(AC210_AUX_8_ADC_PIND)
//#define SIG60_HDC_PIND					AC210_AUX_8_ADC_PIND
#endif




// Note: Still need to define and initialise auxiliary port

#endif /* AC210_PINS_H_ */
