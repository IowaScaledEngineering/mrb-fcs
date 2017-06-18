/*************************************************************************
Title:    MRBus Fast Clock Slave (Wired & Wireless)
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2016 Nathan D. Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <util/delay.h>
#include "avr-i2c-master.h"
#include "tlc59116.h"

#ifdef MRBEE
// If wireless, redefine the common variables and functions
#include "mrbee.h"
#define mrbusTxQueue mrbeeTxQueue
#define mrbusRxQueue mrbeeRxQueue
#else
#include "mrbus.h"
#endif

#define MRBUS_TX_BUFFER_DEPTH 4
#define MRBUS_RX_BUFFER_DEPTH 4

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];


#define MRBUS_CLOCK_SOURCE_ADDRESS  0x10
#define MRBUS_MAX_DEAD_RECKONING    0x11


#define I2C_ADDR_TLC59116_U3  0xC0
#define I2C_ADDR_TLC59116_U4  0xC2

uint8_t mrbus_dev_addr = 3;
uint8_t output_status=0;

uint16_t scaleFactor = 10;
uint8_t flags = 0;

typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t dayOfWeek;
	uint8_t day;
	uint8_t month;
	uint16_t year;
} TimeData;

TimeData realTime;
TimeData fastTime;

void initTimeData(TimeData* t)
{
	t->seconds = t->minutes = t->hours = 0;
	t->dayOfWeek = 0;
	t->year = 2012;
	t->month = t->day = 1;
}

void incrementTime(TimeData* t, uint8_t incSeconds)
{
	uint16_t i = t->seconds + incSeconds;

	while(i >= 60)
	{
		t->minutes++;
		i -= 60;
	}
	t->seconds = (uint8_t)i;
	
	while(t->minutes >= 60)
	{
		t->hours++;
		t->minutes -= 60;
	}
	
	if (t->hours >= 24)
		t->hours %= 24;
}

const uint8_t SEGMENTS[] = 
{
	0b00111111,   // 0
	0b00000110,   // 1
	0b01011011,   // 2
	0b01001111,   // 3
	0b01100110,   // 4
	0b01101101,   // 5
	0b01111101,   // 6
	0b00000111,   // 7
	0b01111111,   // 8
	0b01100111,   // 9
	0b01110111,   // 'A'
	0b01111100,   // 'b'
	0b00111001,   // 'C'
	0b01011110,   // 'd'
	0b01111001,   // 'E'
	0b01110001,   // 'F'
	0b01101111,   // 'g'
	0b01110100,   // 'h'
	0b00000110,   // 'i'
	0b00011110,   // 'J'
	0b01110110,   // 'K'
	0b00111000,   // 'L'
	0b01010101,   // 'M' // Bad, unusable, shown as bar-n
	0b01010100,   // 'n'
	0b01011100,   // 'o' 
	0b01110011,   // 'P'
	0b01100111,   // 'q'
	0b01010000,   // 'r'
	0b01101101,   // 'S'
	0b01110000,   // 't'
	0b00011100,   // 'u'
	0b00011101,   // 'v' // Unusable, shown as bar-u
	0b00011101,   // 'w' // Unusable, shown as bar-u
	0b01110110,   // 'x'
	0b01101110,   // 'y'
	0b01110110,   // 'z' // Unusable
	0b00000000,   // Blank
	0b01000000    // Dash
};

#define LCD_CHAR_0     0
#define LCD_CHAR_1     1
#define LCD_CHAR_2     2
#define LCD_CHAR_3     3
#define LCD_CHAR_4     4
#define LCD_CHAR_5     5
#define LCD_CHAR_6     6
#define LCD_CHAR_7     7
#define LCD_CHAR_8     8
#define LCD_CHAR_9     9
#define LCD_CHAR_H     17
#define LCD_CHAR_O     24
#define LCD_CHAR_L     21
#define LCD_CHAR_D     13
#define LED_CHAR_BLANK 36
#define LED_CHAR_DASH  37

#define DECIMAL_COLON        0x01
#define DECIMAL_PM_INDICATOR 0x08

uint8_t displayCharacters[4] = {LED_CHAR_DASH,LED_CHAR_DASH,LED_CHAR_DASH,LED_CHAR_DASH};
uint8_t displayDecimals = 0;

#define TIME_FLAGS_DISP_FAST       0x01
#define TIME_FLAGS_DISP_FAST_HOLD  0x02
#define TIME_FLAGS_DISP_REAL_AMPM  0x04
#define TIME_FLAGS_DISP_FAST_AMPM  0x08
#define TIME_FLAGS_UPDATE_DISPLAY  0x40
#define TIME_FLAGS_COLON_STATE     0x80

// ******** Start 100 Hz Timer 

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function


uint8_t ticks=0;
uint8_t decisecs=0;

volatile uint16_t fastDecisecs = 0;
volatile uint8_t scaleTenthsAccum = 0;
uint16_t pktPeriod = 0;
uint8_t maxDeadReckoningTime = 50;
uint8_t deadReckoningTime = 0;
uint8_t timeSourceAddress = 0xFF;

void initialize100HzTimer(void)
{
	// Set up timer 0 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 0x6C;
	ticks = 0;
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	TIMSK0 |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
	static uint8_t colon_ticks=0;
	if (++colon_ticks > 50)
	{
		colon_ticks -= 50;
		flags ^= TIME_FLAGS_COLON_STATE;
	}
	
	if (++ticks >= 10)
	{
		ticks -= 10;
		decisecs++;
		flags |= TIME_FLAGS_UPDATE_DISPLAY;
		
		if (deadReckoningTime)
			deadReckoningTime--;
		if (TIME_FLAGS_DISP_FAST == (flags & (TIME_FLAGS_DISP_FAST | TIME_FLAGS_DISP_FAST_HOLD)))
		{
			fastDecisecs += scaleFactor / 10;
			scaleTenthsAccum += scaleFactor % 10;
			if (scaleTenthsAccum > 10)
			{
				fastDecisecs++;
				scaleTenthsAccum -= 10;
			}		
		
		}
	}
}

volatile uint16_t busVoltage=0;

ISR(ADC_vect)
{
	static uint8_t busVoltageCount = 0;
	static uint16_t busVoltageAccum = 0;	
	
	busVoltageAccum += ADC;
	if (++busVoltageCount >= 64)
	{
		busVoltageAccum = busVoltageAccum / 64;
        //At this point, we're at (Vbus/3) / 5 * 1024
        //So multiply by 150, divide by 1024, or multiply by 75 and divide by 512
        busVoltage = ((uint32_t)busVoltageAccum * 75) / 512;
		busVoltageAccum = 0;
		busVoltageCount = 0;
	}
}

void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t i;
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];

	if (0 == mrbusPktQueuePop(&mrbusRxQueue, rxBuffer, sizeof(rxBuffer)))
		return;

	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (rxBuffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		goto	PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != rxBuffer[MRBUS_PKT_DEST] && mrbus_dev_addr != rxBuffer[MRBUS_PKT_DEST]) 
		goto	PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<rxBuffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, rxBuffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_L]))
		goto	PktIgnore;
		
	//*************** END PACKET FILTER ***************


	//*************** PACKET HANDLER - PROCESS HERE ***************

	// Just smash the transmit buffer if we happen to see a packet directed to us
	// that requires an immediate response
	//
	// If we're in here, then either we're transmitting, then we can't be 
	// receiving from someone else, or we failed to transmit whatever we were sending
	// and we're waiting to try again.  Either way, we're not going to corrupt an
	// in-progress transmission.
	//
	// All other non-immediate transmissions (such as scheduled status updates)
	// should be sent out of the main loop so that they don't step on things in
	// the transmit buffer
	
	if ('A' == rxBuffer[MRBUS_PKT_TYPE])
	{
		// PING packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 6;
		txBuffer[MRBUS_PKT_TYPE] = 'a';
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	} 
	else if ('W' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM WRITE Packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'w';
		eeprom_write_byte((uint8_t*)(uint16_t)rxBuffer[6], rxBuffer[7]);
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = rxBuffer[7];
		
		switch(rxBuffer[6])
		{
			case MRBUS_EE_DEVICE_ADDR:
				mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
				break;

			case MRBUS_CLOCK_SOURCE_ADDRESS:
				timeSourceAddress = eeprom_read_byte((uint8_t*)MRBUS_CLOCK_SOURCE_ADDRESS);
				break;
				
			case MRBUS_MAX_DEAD_RECKONING:
				deadReckoningTime = maxDeadReckoningTime = eeprom_read_byte((uint8_t*)MRBUS_MAX_DEAD_RECKONING);
				break;
		}
				txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;	
	}
	else if ('R' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM READ Packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'r';
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)rxBuffer[6]);			
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('V' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Version
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 16;
		txBuffer[MRBUS_PKT_TYPE] = 'v';
#ifdef MRBEE
		txBuffer[6]  = MRBUS_VERSION_WIRELESS;
#else
		txBuffer[6]  = MRBUS_VERSION_WIRED;
#endif
		txBuffer[7]  = 0xFF & ((uint32_t)(GIT_REV))>>16; // Software Revision
		txBuffer[8]  = 0xFF & ((uint32_t)(GIT_REV))>>8; // Software Revision
		txBuffer[9]  = 0xFF & (GIT_REV); // Software Revision
		txBuffer[10]  = 1; // Hardware Major Revision
		txBuffer[11]  = 0; // Hardware Minor Revision
		txBuffer[12] = 'F';
		txBuffer[13] = 'C';
		txBuffer[14] = 'S';
		txBuffer[15] = ' ';
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('X' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Reset
		cli();
		wdt_reset();
		MCUSR &= ~(_BV(WDRF));
		WDTCSR |= _BV(WDE) | _BV(WDCE);
		WDTCSR = _BV(WDE);
		while(1);  // Force a watchdog reset
		sei();
	}
	else if ('T' == rxBuffer[MRBUS_PKT_TYPE] &&
		((0xFF == timeSourceAddress) || (rxBuffer[MRBUS_PKT_SRC] == timeSourceAddress)) )
	{
		// It's a time packet from our time reference source
		realTime.hours = rxBuffer[6];
		realTime.minutes = rxBuffer[7];
		realTime.seconds = rxBuffer[8];
		flags = rxBuffer[9];
		// Time source packets aren't required to have a fast section
		// Doesn't really make sense outside model railroading applications, so...
		if (rxBuffer[MRBUS_PKT_LEN] >= 14)
		{
			fastTime.hours =  rxBuffer[10];
			fastTime.minutes =  rxBuffer[11];
			fastTime.seconds = rxBuffer[12];
			scaleFactor = (((uint16_t)rxBuffer[13])<<8) + (uint16_t)rxBuffer[14];
		}		
		// If we got a packet, there's no dead reckoning time anymore
		fastDecisecs = 0;
		scaleTenthsAccum = 0;
		deadReckoningTime = maxDeadReckoningTime;
	}


	//*************** END PACKET HANDLER  ***************

	
	//*************** RECEIVE CLEANUP ***************
PktIgnore:
	// Yes, I hate gotos as well, but sometimes they're a really handy and efficient
	// way to jump to a common block of cleanup code at the end of a function 
	return;	
}

void init(void)
{
	// Kill watchdog
    MCUSR = 0;
#ifdef ENABLE_WATCHDOG
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	WDTCSR |= _BV(WDE) | _BV(WDCE);
	WDTCSR = _BV(WDE) | _BV(WDP2) | _BV(WDP1); // Set the WDT to system reset and 1s timeout
	wdt_reset();
#else
	wdt_reset();
	wdt_disable();
#endif

	DDRD |= 0xF6;
	PORTD |= 0x0B;

	DDRC |= 0x0F;
	PORTC |= 0x30;

	// Initialize MRBus address from EEPROM
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	timeSourceAddress = eeprom_read_byte((uint8_t*)MRBUS_CLOCK_SOURCE_ADDRESS);
	maxDeadReckoningTime = eeprom_read_byte((uint8_t*)MRBUS_MAX_DEAD_RECKONING);	
	deadReckoningTime = 0; // Give us dashes until such time that we get a time packet
	pktPeriod = ((eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H) << 8) & 0xFF00) | (eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) & 0x00FF);
	
	fastDecisecs = 0;
	
	// Setup ADC
	ADMUX  = 0x46;  // AVCC reference; ADC6 input
	ADCSRA = _BV(ADATE) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 128 prescaler
	ADCSRB = 0x00;
	DIDR0  = 0x00;  // No digitals were harmed in the making of this ADC

	busVoltage = 0;
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);

}

void displayTime(TimeData* time, uint8_t ampm)
{
	uint8_t i=0;
	// Regular, non-fast time mode
	displayCharacters[3] = time->minutes % 10;
	displayCharacters[2] = (time->minutes / 10) % 10;

	if (ampm)
	{
		// If 12 hour mode
		if (time->hours == 0)
		{
			displayCharacters[0] = 1;
			displayCharacters[1] = 2;
		}
		else 
		{
			uint8_t hrs = time->hours;
			if (hrs > 12)
				hrs -= 12;
			
			i = (hrs / 10) % 10;
			displayCharacters[0] = (0==i)?LED_CHAR_BLANK:i;
			displayCharacters[1] = hrs % 10;	
		}

		if (time->hours >= 12)
			displayDecimals |= DECIMAL_PM_INDICATOR;
		else
			displayDecimals &= ~(DECIMAL_PM_INDICATOR);


	} else {
		// 24 hour mode
		i = (time->hours / 10) % 10;
		displayCharacters[1] = time->hours % 10;
		displayCharacters[0] = i;
		displayDecimals &= ~(DECIMAL_PM_INDICATOR);
	}
}

void tlc59116Reset()
{
	PORTC &= ~(_BV(PC3));
	_delay_us(100);
	PORTC |= _BV(PC3);
}


int main(void)
{
	uint8_t statusTransmit=0, i;
	TLC59116Context u3, u4;

	// Application initialization
	init();

	// Initialize MRBus core
#ifdef MRBEE
	mrbusPktQueueInitialize(&mrbeeTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbeeRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbeeInit();
#else
	mrbusPktQueueInitialize(&mrbusTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbusRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbusInit();
#endif

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();
	
	tlc59116Reset();
	
	i2c_master_init();	

	sei();	

	tlc59116Initialize(&u3, I2C_ADDR_TLC59116_U3);
	tlc59116Initialize(&u4, I2C_ADDR_TLC59116_U4);

	while (1)
	{
		wdt_reset();

		// Handle any packets that may have come in
		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbusRxQueue))
		{
			PktHandler();
		}
			
		/* Events that happen every second */
		if (0 != pktPeriod && decisecs >= pktPeriod)
		{
			decisecs -= pktPeriod;
			statusTransmit = 1;		
		}

		if ((TIME_FLAGS_DISP_FAST | TIME_FLAGS_DISP_FAST_HOLD) == (flags & (TIME_FLAGS_DISP_FAST | TIME_FLAGS_DISP_FAST_HOLD)) )  // Hold state
		{
			displayCharacters[0] = LCD_CHAR_H;
			displayCharacters[1] = LCD_CHAR_O;
			displayCharacters[2] = LCD_CHAR_1;
			displayCharacters[3] = LCD_CHAR_D;
			displayDecimals &= ~(DECIMAL_PM_INDICATOR);
		}
		else if (flags & TIME_FLAGS_DISP_FAST)
			displayTime(&fastTime, flags & TIME_FLAGS_DISP_FAST_AMPM);
		else
			displayTime(&realTime, flags & TIME_FLAGS_DISP_REAL_AMPM);

		if ((flags & TIME_FLAGS_DISP_FAST) && !(flags & TIME_FLAGS_DISP_FAST_HOLD) && fastDecisecs >= 10)
		{
			uint8_t fastTimeSecs;
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				fastTimeSecs = fastDecisecs / 10;
				fastDecisecs -= fastTimeSecs * 10;
			}
			incrementTime(&fastTime, fastTimeSecs);
		}
		
		if (flags & TIME_FLAGS_UPDATE_DISPLAY)
		{
			char a, b, c, d;

			a = SEGMENTS[(0 == deadReckoningTime)?LED_CHAR_DASH:displayCharacters[0]];
			b = SEGMENTS[(0 == deadReckoningTime)?LED_CHAR_DASH:displayCharacters[1]];
			c = SEGMENTS[(0 == deadReckoningTime)?LED_CHAR_DASH:displayCharacters[2]];
			d = SEGMENTS[(0 == deadReckoningTime)?LED_CHAR_DASH:displayCharacters[3]];						
		
			if (TIME_FLAGS_COLON_STATE & flags)
				a |= SEGMENT_DP;
			if (DECIMAL_PM_INDICATOR & displayDecimals)
				d |= SEGMENT_DP;
		
			// Convert display characters to TLC59116 output lines
			tlc59116FromCharacters(&u3, a, b);
			tlc59116FromCharacters(&u4, c, d);
			// Send I2C updates to parts
			tlc59116Update(&u3);
			tlc59116Update(&u4);			
			flags &= ~(TIME_FLAGS_UPDATE_DISPLAY);
		}
		

		if (statusTransmit && !(mrbusPktQueueFull(&mrbusTxQueue)))
		{
			uint8_t txBuffer[MRBUS_BUFFER_SIZE];
			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_DEST] = 0xFF;
			txBuffer[MRBUS_PKT_LEN] = 8;
			txBuffer[5] = 'S';
			txBuffer[6] = 0;  // Status byte - no idea what to use this for
			txBuffer[7] = busVoltage;
			statusTransmit = 0;
			mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		}

		// If we have a packet to be transmitted, try to send it here
		while(mrbusPktQueueDepth(&mrbusTxQueue))
		{
			wdt_reset();

			// A slight modification from normal - since slave clocks don't really need an address
			// this will skip over transmitting if we don't have a source address
			if (0xFF == mrbus_dev_addr)
			{
				mrbusPktQueueDrop(&mrbusTxQueue);
				break;
			}

#ifdef MRBUS
			// If we're here, we failed to start transmission due to somebody else transmitting
			// Given that our transmit buffer is full, priority one should be getting that data onto
			// the bus so we can start using our tx buffer again.  So we stay in the while loop, trying
			// to get bus time.

			// We want to wait 20ms before we try a retransmit to avoid hammering the bus
			// Because MRBus has a minimum packet size of 6 bytes @ 57.6kbps,
			// need to check roughly every millisecond to see if we have a new packet
			// so that we don't miss things we're receiving while waiting to transmit
			if (0 != mrbusTransmit()) // 0 is success, all others are failure
			{
				uint8_t bus_countdown = 20;
				while (bus_countdown-- > 0 && !mrbusIsBusIdle())
				{
					wdt_reset();
					_delay_ms(1);
					if (mrbusPktQueueDepth(&mrbusRxQueue))
						PktHandler();
				}
			}
#else
			// MRBee is simple - because the XBee has a buffer, we never fail
			mrbeeTransmit();
#endif
		}
	}
}



