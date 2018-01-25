#pragma once
#include <arduino.h>

/************************************************************

Simple rotary encoder library only tested on Arduino UNO r3
Connect the DT lead to data-pin and CLK lead to clock pin

For optimal performace use pin 2 or 3 for DT since they are
hardware interrupt pins. If another pin is used the library 
will use a software interrupt implementation instead.

Usage:

// Declare class instance
RotaryEncoder enc(data_pin, clk_pin, [optional] switch_pin);

// declare interrupt handlers
void rotary_handler() {
	int dir;

	// get rotation directon in dir if supplied (-1 CCW, 1 CW)
	int value = enc.on_rotary_interrupt( &dir );
}

void switch_handler() {
	// do whatever when switch is pressed
}

setup() {
	// start the encoder supplying interrupt handlers, switch handler optional
	enc.begin(rotary_handler, switch_handler);
}

loop() {
	
	// use value returned from enc.on_rotary_interrupt() or use
	// enc.get_value() to retrieve the last value
}

************************************************************/

#define RE_UNUSED 255

#define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_OUT_BASEREG(pin)             (portOutputRegister(digitalPinToPort(pin)))

#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define DIRECT_PIN_READ(base, mask)     (((*(base)) & (mask)) ? 1 : 0)
#define DIRECT_PIN_WRITE_HIGH(base, mask) (*(base) |= (mask))
#define DIRECT_PIN_WRITE_LOW(base, mask) (*(base) &= ~(mask))

typedef void(*interrupt_handler)();

static int pcint_mode[24] = {};
volatile static int pcint_last[3] = {};
volatile static interrupt_handler pcint_handler[24] = {};
volatile uint8_t *port_to_pcmask[] = 
{
	&PCMSK0,
	&PCMSK1,
	&PCMSK2
};

void re_handle_interrupt(uint8_t port)
{
	//Serial.print("port: "); Serial.println(port);
	
	uint8_t curr = *portInputRegister(port + 2);
	//Serial.print("curr: "); Serial.println(curr, 2);
	//Serial.print("last: "); Serial.println(pcint_last[port], 2);
	uint8_t mask = curr ^ pcint_last[port];
	//Serial.print("mask: "); Serial.println(mask, 2);
	pcint_last[port] = curr;
	//Serial.print("port_to_pcmask: "); Serial.println(*port_to_pcmask[port], 2);

	if((mask &= *port_to_pcmask[port]) == 0)
		return;
	//Serial.print("post mask: "); Serial.println(mask, 2);

	for (uint8_t i = 0; i < 8; ++i) {
		uint8_t bit = 1 << i;
		if (bit & mask) {
			uint8_t pin = port * 8 + i;
			if ((pcint_mode[pin] == CHANGE) ||
				((pcint_mode[pin] == FALLING) && !(curr & bit)) ||
				((pcint_mode[pin] == RISING) && (curr & bit)) &&
				(pcint_handler[pin] != nullptr)) {
				pcint_handler[pin]();
			}
		}
	}
}

ISR(PCINT0_vect)
{
	re_handle_interrupt(0);
}

ISR(PCINT1_vect)
{
	re_handle_interrupt(1);
}

ISR(PCINT2_vect)
{
	re_handle_interrupt(2);
}

class RotaryEncoder
{
public:
	RotaryEncoder(uint8_t dt, uint8_t clk, uint8_t sw = RE_UNUSED)
		: _dt_pin(dt), _clk_pin(clk), _sw_pin(sw), _value(0)
	{
	}

	void re_attachInterrupt(int pin, interrupt_handler handler, int mode)
	{
		if (digitalPinToInterrupt(pin) != NOT_AN_INTERRUPT) {
			attachInterrupt(digitalPinToInterrupt(pin), handler, mode);
		}
		else {
			uint8_t bit = digitalPinToBitMask(pin);
			uint8_t port = digitalPinToPort(pin);
			port -= 2;
			volatile uint8_t *pcmask = port_to_pcmask[port];
			int slot = (port == 1) ? (port * 8 + pin - 14) : (port * 8 + (pin % 8));

			//Serial.println("registering on slot:"); Serial.println(slot);
			pcint_mode[slot] = mode;
			pcint_handler[slot] = handler;

			*pcmask |= bit;
			PCICR |= 1 << port;
		}
	}

	void begin(interrupt_handler rotary_handler, interrupt_handler sw_handler = nullptr)
	{
		_rotary_handler = rotary_handler;
		_sw_handler = sw_handler;
		
		pinMode(_dt_pin, INPUT_PULLUP);
		pinMode(_clk_pin, INPUT_PULLUP);

		re_attachInterrupt(_dt_pin, _rotary_handler, CHANGE);

		_dt_reg = PIN_TO_BASEREG(_dt_pin);
		_dt_mask = PIN_TO_BITMASK(_dt_pin);
		_clk_reg = PIN_TO_BASEREG(_clk_pin);
		_clk_mask = PIN_TO_BITMASK(_clk_pin);

		if (_sw_pin != RE_UNUSED) {
			pinMode(_sw_pin, INPUT_PULLUP);
			re_attachInterrupt(_sw_pin, sw_handler, FALLING);
		}
	}

	int get_value()
	{
		return _value;
	}

	int on_rotary_interrupt(int *dir = nullptr)
	{
		if(dir)
			*dir = 0;

		int cur_dt = DIRECT_PIN_READ(_dt_reg, _dt_mask);
		int cur_clk = DIRECT_PIN_READ(_clk_reg, _clk_mask);

		if (cur_dt != _prev_dt) {
			if(cur_dt == LOW) 
				_prev_clk = cur_clk;
			else {
				if (_prev_clk != cur_clk) {
					if (_prev_clk) {
						_value++;
						if(dir) 
							*dir = 1;
					}
					else {
						_value--;
						if(dir) 
							*dir = -1;
					}
				}
			}
			_prev_dt = cur_dt;
		}
		return _value;
	}

private:
	uint8_t _dt_pin, _clk_pin, _sw_pin;

	volatile uint8_t * _dt_reg;
	volatile uint8_t * _clk_reg;
	uint8_t _dt_mask;
	uint8_t _clk_mask;

	uint8_t _prev_clk;
	uint8_t _prev_dt = HIGH;

	int _value;
	interrupt_handler _rotary_handler;
	interrupt_handler _sw_handler;
};



