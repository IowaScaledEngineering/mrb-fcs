/*************************************************************************
Title:    MRBus 4-Channel Block Detector, Mark II
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
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "mrbus.h"

extern uint8_t mrbus_rx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbus_tx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbus_state;
extern uint8_t mrbus_activity;

#define MRBUS_CLOCK_SOURCE_ADDRESS  0x10

uint8_t mrbus_dev_addr = 0x11;
uint8_t output_status=0;

uint8_t scaleFactor = 1;
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

uint8_t displayCharacters[4] = {0,1,2,3};

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

// ******** Start 100 Hz Timer 

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

uint8_t ticks=0;
uint8_t decisecs=0;
uint8_t colon_ticks=0;
volatile uint16_t fastDecisecs=0;
uint8_t maxDeadReckoningTime = 50;
uint8_t deadReckoningTime = 50;
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
	uint8_t segments = SEGMENTS[(0 == deadReckoningTime)?37:displayCharacters[digit]];
	uint8_t anodes = DIGIT_DRIVE[digit];
	// Shut down segment drives
	PORTB &= ~(0x3F);
	PORTD &= ~(0xC0);

	// Switch digit
	PORTC = (PORTC | 0x0F) & anodes;
	PORTB |= segments & 0x3F;
	PORTD |= segments & 0xC0;
	
	if (++colon_ticks > 200)
	{
		colon_ticks = 0;
		PORTD ^= _BV(PD3);
	}
	
	if (++ticks >= 40)
	{
		ticks = 0;
		decisecs++;
		if (deadReckoningTime)
			deadReckoningTime--;
		if (0x01 == (flags & (0x01 | 0x08)))
			fastDecisecs += scaleFactor;
	}



}

void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t i, time_source_addr=0;

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
		if (MRBUS_EE_DEVICE_ADDR == mrbus_rx_buffer[6])
			mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
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

	time_source_addr = eeprom_read_byte((uint8_t*)MRBUS_CLOCK_SOURCE_ADDRESS);

	if ((mrbus_rx_buffer[MRBUS_PKT_SRC] == time_source_addr) && 'T' == mrbus_rx_buffer[MRBUS_PKT_TYPE])
	{
		// A-ha!  It's a time reference packet!
		realTime.hours = mrbus_rx_buffer[6];
		realTime.minutes = mrbus_rx_buffer[7];
		realTime.seconds = mrbus_rx_buffer[8];
		fastTime.hours =  mrbus_rx_buffer[10];
		fastTime.minutes =  mrbus_rx_buffer[11];
		fastTime.seconds = mrbus_rx_buffer[12];
		scaleFactor = mrbus_rx_buffer[13];
		flags = mrbus_rx_buffer[9];
		
		// If we got a packet, there's no dead reckoning time anymore
		fastDecisecs = 0;
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
	// Initialize MRBus address from EEPROM
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	fastDecisecs = 0;	
}


int main(void)
{
	uint8_t changed=0, i;
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
		// Handle any packets that may have come in
		if (mrbus_state & MRBUS_RX_PKT_READY)
			PktHandler();
			
		/* Events that happen every second */
		if (decisecs >= 20)
		{
			decisecs = 0;
			changed = 1;		
		}


		if (flags & 0x01) // Fast Mode
		{
			if (flags & 0x02) // Hold Mode
			{
				displayCharacters[0] = 17;
				displayCharacters[1] = 24;
				displayCharacters[2] = 1;
				displayCharacters[3] = 13;
			} else {
				displayCharacters[3] = fastTime.minutes % 10;
				displayCharacters[2] = (fastTime.minutes / 10) % 10;

				i = (fastTime.hours / 10) % 10;
				if (flags & 0x08)
				{
					// If 12 hour mode
					// FIXME
					displayCharacters[1] = fastTime.hours % 10;
					displayCharacters[0] = i;
				} else {
					// 24 hour mode
					displayCharacters[1] = fastTime.hours % 10;
					displayCharacters[0] = i;
				}
			}

		} else {
			// Regular, non-fast time mode
			displayCharacters[3] = realTime.minutes % 10;
			displayCharacters[2] = (realTime.minutes / 10) % 10;

			i = (realTime.hours / 10) % 10;
			if (flags & 0x04)
			{
				// If 12 hour mode
				// FIXME
				displayCharacters[1] = realTime.hours % 10;
				displayCharacters[0] = i;
			} else {
				// 24 hour mode
				displayCharacters[1] = realTime.hours % 10;
				displayCharacters[0] = i;
			}
		
		}

		if((flags & 0x01) && !(flags & 0x08) && fastDecisecs >= 10)
		{
			uint8_t fastTimeSecs = fastDecisecs / 10;
			incrementTime(&fastTime, fastTimeSecs);
			fastDecisecs -= fastTimeSecs * 10;
		}
		

		// If we have a packet to be transmitted, try to send it here
		while(mrbus_state & MRBUS_TX_PKT_READY)
		{
			uint8_t bus_countdown;

			// Even while we're sitting here trying to transmit, keep handling
			// any packets we're receiving so that we keep up with the current state of the
			// bus.  Obviously things that request a response cannot go, since the transmit
			// buffer is full.
			if (mrbus_state & MRBUS_RX_PKT_READY)
				PktHandler();


			if (0 == mrbusPacketTransmit())
			{
				mrbus_state &= ~(MRBUS_TX_PKT_READY);
				break;
			}

			// If we're here, we failed to start transmission due to somebody else transmitting
			// Given that our transmit buffer is full, priority one should be getting that data onto
			// the bus so we can start using our tx buffer again.  So we stay in the while loop, trying
			// to get bus time.

			// We want to wait 20ms before we try a retransmit
			// Because MRBus has a minimum packet size of 6 bytes @ 57.6kbps,
			// need to check roughly every millisecond to see if we have a new packet
			// so that we don't miss things we're receiving while waiting to transmit
			bus_countdown = 20;
			while (bus_countdown-- > 0 && MRBUS_ACTIVITY_RX_COMPLETE != mrbus_activity)
			{
				//clrwdt();
				_delay_ms(1);
				if (mrbus_state & MRBUS_RX_PKT_READY) 
					PktHandler();
			}
		}
	}
}



