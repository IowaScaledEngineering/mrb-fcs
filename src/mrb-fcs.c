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

uint8_t displayCharacters[4] = {0,1,2,3};

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
	0b01000000,   // Dash

};

// ******** Start 100 Hz Timer 

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

uint16_t ticks=0;
uint8_t secs=0;

void initialize400HzTimer(void)
{
	// Set up timer 1 for 400Hz interrupts
	TCNT0 = 0;
	OCR0A = 0xC2;
	ticks = 0;
	secs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02); // | _BV(CS00);
	TIMSK |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
	uint8_t digit = (ticks%4);
	// Shut down segment drives
	PORTB = 0xFF;
	// Switch digit
	PORTD = (PORTD | (_BV(PD3) | _BV(PD4) | _BV(PD5) | _BV(PD6))) & ~(8<<digit);
	PORTB = SEGMENTS[displayCharacters[3-digit]];

	if (++ticks >= 400)
	{
		ticks = 0;
		secs++;
	}
}
#define UINT16_HIGH_BYTE(a)  ((a)>>8)
#define UINT16_LOW_BYTE(a)  ((a) & 0xFF)

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

	if ((mrbus_rx_buffer[MRBUS_PKT_SRC] == time_source_addr) && 'T' == )
	{


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
}


int main(void)
{
	uint8_t changed=0, old_output_status = 0;
	// Application initialization
	init();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize400HzTimer();

	PORTB = 0xFF;
	PORTD |= _BV(PD3) | _BV(PD4) | _BV(PD5) | _BV(PD6);

	DDRB = 0x7F;
	DDRD = _BV(PD3) | _BV(PD4) | _BV(PD5) | _BV(PD6);

	
	// Initialize MRBus core
	mrbusInit();

	sei();	

	while (1)
	{
		// Handle any packets that may have come in
		if (mrbus_state & MRBUS_RX_PKT_READY)
			PktHandler();
			
		/* Events that happen every second */
		if (secs >= 2)
		{
			secs = 0;
			changed = 1;		
		}

		/* If we need to send a packet and we're not already busy... */
		if ((changed != 0) && !(mrbus_state & (MRBUS_TX_BUF_ACTIVE | MRBUS_TX_PKT_READY)))
		{
			mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			mrbus_tx_buffer[MRBUS_PKT_DEST] = 0xFF;
			mrbus_tx_buffer[MRBUS_PKT_LEN] = 7;			
			mrbus_tx_buffer[5] = 'S';
			mrbus_tx_buffer[6] = output_status;
			mrbus_state |= MRBUS_TX_PKT_READY;
			changed = 0;
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



