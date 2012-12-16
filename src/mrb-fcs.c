/*************************************************************************
Title:    MRBus Fast Clock Slave (Wired & Wireless)
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2012 Nathan Holmes

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
#include <util/delay.h>


#ifdef MRBEE
// If wireless, redefine the common variables and functions
#include "mrbee.h"
#define mrbus_rx_buffer mrbee_rx_buffer
#define mrbus_tx_buffer mrbee_tx_buffer
#define mrbus_state mrbee_state
#define mrbux_rx_buffer mrbee_rx_buffer
#define mrbus_tx_buffer mrbee_tx_buffer
#define mrbus_state mrbee_state
#define mrbusInit mrbeeInit
#define mrbusPacketTransmit mrbeePacketTransmit
#endif

#include "mrbus.h"

extern uint8_t mrbus_rx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbus_tx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbus_state;
extern uint8_t mrbus_activity;

#define MRBUS_CLOCK_SOURCE_ADDRESS  0x10
#define MRBUS_MAX_DEAD_RECKONING    0x11

uint8_t mrbus_dev_addr = 0x11;
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

// I screwed up the layout...
//  Digit 0 is actually driven by bit 1, digit 1 by bit 0, digit 2 by bit 3, and digit 3 by bit 2

const uint8_t DIGIT_DRIVE [] =
{
	0xFD, // Digit 0
	0xFE, // Digit 1
	0xF7, // Digit 2
	0xFB  // Digit 3
};


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

#define DECIMAL_PM_INDICATOR 0x08

uint8_t displayCharacters[4] = {LED_CHAR_DASH,LED_CHAR_DASH,LED_CHAR_DASH,LED_CHAR_DASH};
uint8_t displayDecimals = 0;

#define TIME_FLAGS_DISP_FAST       0x01
#define TIME_FLAGS_DISP_FAST_HOLD  0x02
#define TIME_FLAGS_DISP_REAL_AMPM  0x04
#define TIME_FLAGS_DISP_FAST_AMPM  0x08

// ******** Start 100 Hz Timer 

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function


uint8_t ticks=0;
uint8_t decisecs=0;
uint8_t colon_ticks=0;
volatile uint16_t fastDecisecs = 0;
volatile uint8_t scaleTenthsAccum = 0;
uint16_t pktPeriod = 0;
uint8_t maxDeadReckoningTime = 50;
uint8_t deadReckoningTime = 0;
uint8_t timeSourceAddress = 0xFF;

void initialize400HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 0xC2;
	ticks = 0;
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02); // | _BV(CS00);
	TIMSK0 |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
	uint8_t digit = ticks % 4;
	uint8_t segments = SEGMENTS[(0 == deadReckoningTime)?LED_CHAR_DASH:displayCharacters[digit]];
	uint8_t anodes = DIGIT_DRIVE[digit];
	// Shut down segment drives
	PORTB &= ~(0x3F);
	PORTD &= ~(0xC0);

	if (displayDecimals & (1<<digit))
		segments |= 0x80;

	// Switch digit
	PORTC = (PORTC | 0x0F) & anodes;
	PORTB |= segments & 0x3F;
	PORTD |= segments & 0xC0;
	
	if (++colon_ticks > 200)
	{
		colon_ticks -= 200;
		PORTD ^= _BV(PD3);
	}
	
	if (++ticks >= 40)
	{
		ticks -= 40;
		decisecs++;
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

volatile uint16_t busVoltageAccum=0;
volatile uint16_t busVoltage=0;
volatile uint8_t busVoltageCount=0;

ISR(ADC_vect)
{
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

	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (mrbus_rx_buffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		goto	PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != mrbus_rx_buffer[MRBUS_PKT_DEST] && mrbus_dev_addr != mrbus_rx_buffer[MRBUS_PKT_DEST]) 
		goto	PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<mrbus_rx_buffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, mrbus_rx_buffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != mrbus_rx_buffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != mrbus_rx_buffer[MRBUS_PKT_CRC_L]))
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
	
	if ('A' == mrbus_rx_buffer[MRBUS_PKT_TYPE])
	{
		// PING packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 6;
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'a';
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	} 
	else if ('W' == mrbus_rx_buffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM WRITE Packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 8;			
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'w';
		eeprom_write_byte((uint8_t*)(uint16_t)mrbus_rx_buffer[6], mrbus_rx_buffer[7]);
		mrbus_tx_buffer[6] = mrbus_rx_buffer[6];
		mrbus_tx_buffer[7] = mrbus_rx_buffer[7];
		
		switch(mrbus_rx_buffer[6])
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
		
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	
	}
	else if ('R' == mrbus_rx_buffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM READ Packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 7;			
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'r';
		mrbus_tx_buffer[6] = eeprom_read_byte((uint8_t*)(uint16_t)mrbus_rx_buffer[6]);			
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	}
	else if ('V' == mrbus_rx_buffer[MRBUS_PKT_TYPE])
    {
        // Version
        mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
        mrbus_tx_buffer[MRBUS_PKT_LEN] = 15;
        mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'v';
#ifdef MRBEE
        mrbus_tx_buffer[6]  = MRBUS_VERSION_WIRELESS;
#else
        mrbus_tx_buffer[6]  = MRBUS_VERSION_WIRED;
#endif
        mrbus_tx_buffer[7]  = SWREV; // Software Revision
        mrbus_tx_buffer[8]  = SWREV; // Software Revision
        mrbus_tx_buffer[9]  = SWREV; // Software Revision
        mrbus_tx_buffer[10]  = HWREV_MAJOR; // Hardware Major Revision
        mrbus_tx_buffer[11]  = HWREV_MINOR; // Hardware Minor Revision
        mrbus_tx_buffer[12] = 'F';
        mrbus_tx_buffer[13] = 'C';
        mrbus_tx_buffer[13] = 'S';
        mrbus_state |= MRBUS_TX_PKT_READY;
        goto PktIgnore;
    }
	else if ('T' == mrbus_rx_buffer[MRBUS_PKT_TYPE] &&
		((0xFF == timeSourceAddress) || (mrbus_rx_buffer[MRBUS_PKT_SRC] == timeSourceAddress)) )
	{
		// It's a time packet from our time reference source
		realTime.hours = mrbus_rx_buffer[6];
		realTime.minutes = mrbus_rx_buffer[7];
		realTime.seconds = mrbus_rx_buffer[8];
		flags = mrbus_rx_buffer[9];
		// Time source packets aren't required to have a fast section
		// Doesn't really make sense outside model railroading applications, so...
		if (mrbus_rx_buffer[MRBUS_PKT_LEN] >= 14)
		{
			fastTime.hours =  mrbus_rx_buffer[10];
			fastTime.minutes =  mrbus_rx_buffer[11];
			fastTime.seconds = mrbus_rx_buffer[12];
			scaleFactor = (((uint16_t)mrbus_rx_buffer[13])<<8) + (uint16_t)mrbus_rx_buffer[14];
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

	// This section resets anything that needs to be reset in order to allow us to receive
	// another packet.  Typically, that's just clearing the MRBUS_RX_PKT_READY flag to 
	// indicate to the core library that the mrbus_rx_buffer is clear.
	mrbus_state &= (~MRBUS_RX_PKT_READY);
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
	busVoltageAccum = 0;
	busVoltageCount = 0;
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

int main(void)
{
	uint8_t statusTransmit=0, i;
	// Application initialization
	init();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize400HzTimer();

	PORTB &= ~(0x3F);
	PORTD &= ~(0xC0);
	PORTC |= 0x0F;

	DDRB |= 0x3F;
	DDRD |= 0xC0;
	DDRC |= 0x0F;
	
	// Initialize MRBus core
	mrbusInit();

	sei();	

	while (1)
	{
		wdt_reset();

#ifdef MRBEE
		mrbeePoll();
#endif

		// Handle any packets that may have come in
		if (mrbus_state & MRBUS_RX_PKT_READY)
			PktHandler();
			
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
			uint8_t fastTimeSecs = fastDecisecs / 10;
			incrementTime(&fastTime, fastTimeSecs);
			fastDecisecs -= fastTimeSecs * 10;
		}
		

		if (statusTransmit && !(mrbus_state & MRBUS_TX_PKT_READY))
		{
			mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			mrbus_tx_buffer[MRBUS_PKT_DEST] = 0xFF;
			mrbus_tx_buffer[MRBUS_PKT_LEN] = 8;
			mrbus_tx_buffer[5] = 'S';
			mrbus_tx_buffer[6] = 0;  // Status byte - no idea what to use this for
			mrbus_tx_buffer[7] = busVoltage;
			statusTransmit = 0;
			mrbus_state |= MRBUS_TX_PKT_READY;
		}

		// If we have a packet to be transmitted, try to send it here
		while(mrbus_state & MRBUS_TX_PKT_READY)
		{
			wdt_reset();

			// Even while we're sitting here trying to transmit, keep handling
			// any packets we're receiving so that we keep up with the current state of the
			// bus.  Obviously things that request a response cannot go, since the transmit
			// buffer is full.
#ifdef MRBEE
			mrbeePoll();
#endif			
	
			if (mrbus_state & MRBUS_RX_PKT_READY)
				PktHandler();

			// A slight modification from normal - since slave clocks don't really need an address
			// this will skip over transmitting if we don't have a source address
			if (0xFF == mrbus_dev_addr || 0 == mrbusPacketTransmit())
			{
				mrbus_state &= ~(MRBUS_TX_PKT_READY);
				break;
			}

#ifndef MRBEE
			// If we're here, we failed to start transmission due to somebody else transmitting
			// Given that our transmit buffer is full, priority one should be getting that data onto
			// the bus so we can start using our tx buffer again.  So we stay in the while loop, trying
			// to get bus time.

			// We want to wait 20ms before we try a retransmit
			// Because MRBus has a minimum packet size of 6 bytes @ 57.6kbps,
			// need to check roughly every millisecond to see if we have a new packet
			// so that we don't miss things we're receiving while waiting to transmit
			{
				uint8_t bus_countdown = 20;
				while (bus_countdown-- > 0 && MRBUS_ACTIVITY_RX_COMPLETE != mrbus_activity)
				{
					//clrwdt();
					_delay_ms(1);
					if (mrbus_state & MRBUS_RX_PKT_READY) 
						PktHandler();
				}
			}
#endif
		}
	}
}



