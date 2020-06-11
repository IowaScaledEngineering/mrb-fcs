#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "avr-i2c-master.h"
#include "tlc59116.h"

void tlc59116Initialize(TLC59116Context* t, uint8_t address)
{
	uint8_t i2cBuf[4];

	memset(t, 0, sizeof(TLC59116Context));
	t->address = address;

	i2cBuf[0] = t->address;
	i2cBuf[1] = 0;
	i2cBuf[2] = 0xE0; // Oscillator on, autoincrement on, all-call and subaddresses off

	// Got it - send to part
	i2c_transmit(i2cBuf, 3, 1);
	while(i2c_busy()) { wdt_reset(); };
	
	_delay_us(500); // Datasheet says we have to wait 500uS after the oscillator has been turned on
}

void tlc59116LedSet(TLC59116Context* t, uint8_t ledNumber, TLC59116LEDState ledState)
{
	uint8_t byteNum = ledNumber / 4;
	uint8_t shiftNum = 2*(ledNumber & 0x03);
	if (ledNumber > 15)
		return;
		
		
	ledState &= 0x03;
	
	t->ledStates[byteNum] &= ~(0x03 << shiftNum);
	t->ledStates[byteNum] |= (ledState << shiftNum);
}

void tlc59116Update(TLC59116Context* t)
{
	uint8_t i2cBuf[7];

	if (0 == t->address)
		return;

	memset(i2cBuf, 0, sizeof(i2cBuf));
	
	i2cBuf[0] = t->address;
	i2cBuf[1] = 0x14 | 0xE0;
	i2cBuf[2] = t->ledStates[0];
	i2cBuf[3] = t->ledStates[1];
	i2cBuf[4] = t->ledStates[2];
	i2cBuf[5] = t->ledStates[3];
	// Got it - send to part
	i2c_transmit(i2cBuf, 6, 1);
	while(i2c_busy()) { wdt_reset(); };
}





void tlc59116FromCharacters(TLC59116Context* t, uint8_t displayChar1, uint8_t displayChar2)
{
	tlc59116LedSet(t, LED_DIG1_A,  (displayChar1 & SEGMENT_A)?LED_ON:LED_OFF);
	tlc59116LedSet(t, LED_DIG1_B,  (displayChar1 & SEGMENT_B)?LED_ON:LED_OFF);
	tlc59116LedSet(t, LED_DIG1_C,  (displayChar1 & SEGMENT_C)?LED_ON:LED_OFF);
	tlc59116LedSet(t, LED_DIG1_D,  (displayChar1 & SEGMENT_D)?LED_ON:LED_OFF);
	tlc59116LedSet(t, LED_DIG1_E,  (displayChar1 & SEGMENT_E)?LED_ON:LED_OFF);
	tlc59116LedSet(t, LED_DIG1_F,  (displayChar1 & SEGMENT_F)?LED_ON:LED_OFF);
	tlc59116LedSet(t, LED_DIG1_G,  (displayChar1 & SEGMENT_G)?LED_ON:LED_OFF);
	tlc59116LedSet(t, LED_DIG1_DP, (displayChar1 & SEGMENT_DP)?LED_ON:LED_OFF);

	tlc59116LedSet(t, LED_DIG2_A,  (displayChar2 & SEGMENT_A)?LED_ON:LED_OFF);
	tlc59116LedSet(t, LED_DIG2_B,  (displayChar2 & SEGMENT_B)?LED_ON:LED_OFF);
	tlc59116LedSet(t, LED_DIG2_C,  (displayChar2 & SEGMENT_C)?LED_ON:LED_OFF);
	tlc59116LedSet(t, LED_DIG2_D,  (displayChar2 & SEGMENT_D)?LED_ON:LED_OFF);
	tlc59116LedSet(t, LED_DIG2_E,  (displayChar2 & SEGMENT_E)?LED_ON:LED_OFF);
	tlc59116LedSet(t, LED_DIG2_F,  (displayChar2 & SEGMENT_F)?LED_ON:LED_OFF);
	tlc59116LedSet(t, LED_DIG2_G,  (displayChar2 & SEGMENT_G)?LED_ON:LED_OFF);
	tlc59116LedSet(t, LED_DIG2_DP, (displayChar2 & SEGMENT_DP)?LED_ON:LED_OFF);


}




