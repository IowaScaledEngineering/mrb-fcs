/*************************************************************************
Title:    AVR TLC59116 Control Library
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     tlc59116.h
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2016 Nathan Holmes

    Much of this was shamelessly borrowed from Atmel's appnote AVR315.
    My thanks to them for saving me a great amount of time.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#ifndef _TLC59116_H
#define _TLC59116_H

typedef struct
{
	uint8_t address;
	uint8_t ledStates[4];
} TLC59116Context;

typedef enum
{
	LED_OFF    = 0x00,
	LED_ON     = 0x01,
	LED_PWM    = 0x02,
	LED_GRPPWM = 0x03
} TLC59116LEDState;

void tlc59116Initialize(TLC59116Context* t, uint8_t address);
void tlc59116LedSet(TLC59116Context* t, uint8_t ledNumber, TLC59116LEDState ledState);
void tlc59116Update(TLC59116Context* t);
void tlc59116FromCharacters(TLC59116Context* t, uint8_t displayChar1, uint8_t displayChar2);

#define SEGMENT_A  0x01
#define SEGMENT_B  0x02
#define SEGMENT_C  0x04
#define SEGMENT_D  0x08
#define SEGMENT_E  0x10
#define SEGMENT_F  0x20
#define SEGMENT_G  0x40
#define SEGMENT_DP 0x80

#define LED_DIG1_A  11
#define LED_DIG1_B  12
#define LED_DIG1_C   6
#define LED_DIG1_D   7
#define LED_DIG1_E   8
#define LED_DIG1_F   9
#define LED_DIG1_G  10
#define LED_DIG1_DP  5

#define LED_DIG2_A  14
#define LED_DIG2_B  15
#define LED_DIG2_C   1
#define LED_DIG2_D   3
#define LED_DIG2_E   4
#define LED_DIG2_F  13
#define LED_DIG2_G   2
#define LED_DIG2_DP  0


#endif

