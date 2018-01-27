#pragma once
#include <arduino.h>
#include <interrupt_handler.h>	// Depends on interrupt_handler library


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
void rotary_handler(int value int dir) {
	// gets the current trackes value in value and the last direction in dir
}

void switch_handler() {
	// do whatever when switch is pressed
}

setup() {
	// start the encoder supplying interrupt handlers, switch handler optional but required if a switch pin has been supplied
	enc.begin(rotary_handler, switch_handler);
}

loop() {
	
	// use value returned from rotary_handler or use
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

typedef void(*rotary_callback)(int value, int dir);
typedef void(*switch_callback)(void);

class RotaryEncoder
{
public:
	RotaryEncoder(uint8_t dt, uint8_t clk, uint8_t sw = RE_UNUSED)
		: _dt_pin(dt), _clk_pin(clk), _sw_pin(sw), _value(0)
	{
	}

	void begin(rotary_callback rotary_handler = nullptr, switch_callback sw_handler = nullptr)
	{
		_rotary_handler = rotary_handler;
		_switch_handler = sw_handler;
		
		pinMode(_dt_pin, INPUT_PULLUP);
		pinMode(_clk_pin, INPUT_PULLUP);

		interrupt::attach_interrupt(_dt_pin, int_rotary_handler, CHANGE, this);

		_dt_reg = PIN_TO_BASEREG(_dt_pin);
		_dt_mask = PIN_TO_BITMASK(_dt_pin);
		_clk_reg = PIN_TO_BASEREG(_clk_pin);
		_clk_mask = PIN_TO_BITMASK(_clk_pin);

		if (_sw_pin != RE_UNUSED) {
			pinMode(_sw_pin, INPUT_PULLUP);
			interrupt::attach_interrupt(_sw_pin, int_switch_handler, FALLING, this);
		}
	}

	int get_value()
	{
		return _value;
	}

	void on_switch_interrupt()
	{
		if (_switch_handler) {
			_switch_handler();
		}
	}

	void on_rotary_interrupt()
	{
		int dir = 0;
		int cur_dt = DIRECT_PIN_READ(_dt_reg, _dt_mask);
		int cur_clk = DIRECT_PIN_READ(_clk_reg, _clk_mask);

		if (cur_dt != _prev_dt) {
			if(cur_dt == LOW) 
				_prev_clk = cur_clk;
			else {
				if (_prev_clk != cur_clk) {
					if (_prev_clk) {
						_value++;
						dir = 1;
					}
					else {
						_value--;
						dir = -1;
					}
				}
			}
			_prev_dt = cur_dt;
		}
		if (_rotary_handler) {
			_rotary_handler(_value, dir);
		}
	}

	static void int_rotary_handler(void *user_data)
	{
		RotaryEncoder *re = (RotaryEncoder*)user_data;
		re->on_rotary_interrupt();
	}

	static void int_switch_handler(void *user_data)
	{
		RotaryEncoder *re = (RotaryEncoder*)user_data;
		re->on_switch_interrupt();
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
	rotary_callback _rotary_handler;
	switch_callback _switch_handler;
};



