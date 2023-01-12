#include <Arduino.h>
#include "Stepper.h"

// stepping scheme for the motor
const uint8_t phase_scheme[8][4] = 
{
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1},
  {1,0,0,0}
};

// constructor
Stepper::Stepper(uint8_t pin_1, uint8_t pin_2, uint8_t pin_3, uint8_t pin_4)
{
  // Initialize variables
  _step_act = 0;
  _step_target = 0;
  _is_modulo = false;
  _is_limited = false;
  _steps_turn = 4096;
  _steps_modulo = 0;
  _feed_const = _steps_turn / 360.0;
  _upper_limit = 0x7fffffff;
  _lower_limit = 0x80000001;
  _delay_step = 1250;
  _delay_powersave = 0;
  _time_last_step = micros() + _delay_step;
  
  // Arduino pins for the motor control connection:
  _pin_1 = pin_1;
  _pin_2 = pin_2;
  _pin_3 = pin_3;
  _pin_4 = pin_4;

  // setup the pins on the microcontroller:
  pinMode(_pin_1, OUTPUT);
  pinMode(_pin_2, OUTPUT);
  pinMode(_pin_3, OUTPUT);
  pinMode(_pin_4, OUTPUT);

  // and start in idle mode
  _power_off();
}

// variable steps (e.g. with gear)
Stepper::Stepper(uint8_t pin_1, uint8_t pin_2, uint8_t pin_3, uint8_t pin_4, uint16_t steps) : Stepper(pin_1, pin_2, pin_3, pin_4) 
{
  _steps_turn = steps;
}

// cyclic handle of motion (call in loop)
void Stepper::handle()
{
  // check if next step can be executed (rate limitation)
  unsigned long now = micros();
  if (now > _time_last_step + _delay_step)
  {
    // do one step in the right direction
    int32_t diff = _diff_modulo(_step_target - _step_act);
    if (diff > 0)
    {
      _step_act = _trim_modulo(_step_act + 1);
      _time_last_step = now;
      _step();
    }
    if (diff < 0)
    {
      _step_act = _trim_modulo(_step_act - 1);
      _time_last_step = now;
      _step();
    }
    if ((_delay_powersave > 0) && (now > _time_last_step + _delay_powersave))
    {
      _power_off();
    }
  }
}

// set new target position
void Stepper::set_inc(int32_t pos)
{
  if (_is_limited)
  {
    pos = min(max(pos, _lower_limit), _upper_limit);
  }
  _step_target = _trim_modulo(pos);
}

// set relative target position
void Stepper::set_inc_rel(int32_t steps)
{
  set_inc(_step_target + steps);
}

// set new target position
void Stepper::set_pos(float pos)
{
  set_inc((int32_t)(pos * _feed_const));
}

// set new target position relative
void Stepper::set_pos_rel(float pos)
{
  set_inc_rel((int32_t)(pos * _feed_const));
}

// automatic trim position in modulo range
int32_t Stepper::_trim_modulo(int32_t pos)
{
  if (_is_modulo) 
  {
    if (pos >= _steps_modulo)
    {
      pos -= _steps_modulo;
    }
    if (pos < 0)
    {
      pos += _steps_modulo;
    }
  }
  return pos;
}

// automatic trim position difference in modulo range
int32_t Stepper::_diff_modulo(int32_t diff)
{
  if (_is_modulo)
  {
    if (diff > (_steps_modulo >> 1))
    {
      diff -= _steps_modulo;
    }
    if (diff < -(_steps_modulo >> 1))
    {
      diff += _steps_modulo;
    }
  }
  return diff;
}

// return actual position
int32_t Stepper::get_inc()
{
  return (_step_act);
}

// get new cuurrent position
float Stepper::get_pos()
{
  return (float) _step_act / _feed_const;
}

// check if target position reached
bool Stepper::in_target()
{
  return (_step_target == _step_act);
}

// wait and handle steps until target position reached
void Stepper::move()
{
  while (!in_target())
  {
    handle();
  }
}

// set actual and target position to zero
void Stepper::reset()
{
  _step_act = 0;
  _step_target = 0;
}

// adjust position by some steps
void Stepper::adjust(int32_t steps)
{
  _step_act -= steps;
  _step();
}

// override stepper frequency
void Stepper::set_freq(uint16_t freq)
{
  _delay_step = 1000000UL / freq;
}

// make this a modulo axis
void Stepper::set_modulo(uint16_t steps)
{
  _is_modulo = true;
  _is_limited = false;
  _steps_modulo = steps;
}

// remove limits and modulo
void Stepper::set_unlimited()
{
  _is_limited = false;
  _is_modulo = false;
  _lower_limit = 0x80000001;
  _upper_limit = 0x7fffffff;
  _steps_modulo = 0;
}

// set software limits
void Stepper::set_limit(float lower, float upper)
{
  _is_limited = true;
  _is_modulo = false;
  _lower_limit = lower * _feed_const;
  _upper_limit = upper * _feed_const; 
}

// Feedrate per turn (default 360)
void Stepper::set_feed_const(float feed)
{
  _feed_const = _steps_turn / feed;
}

// invert direction
void Stepper::reverse_dir(bool neg)
{
  _neg_dir = neg;
}

void Stepper::set_powersave(uint16_t seconds)
{
  _delay_powersave = 1000000UL * seconds;
}

// execute one step
// TODO: decouple from act_step?
void Stepper::_step()
{
  int phase = (int) (_step_act & 0x07);
  if (_neg_dir)
  {
    // invert direction
    phase = 7 - phase;
  }
  digitalWrite(_pin_1, phase_scheme[phase][0]);
  digitalWrite(_pin_2, phase_scheme[phase][1]);
  digitalWrite(_pin_3, phase_scheme[phase][2]);
  digitalWrite(_pin_4, phase_scheme[phase][3]);
}

// switch power off
void Stepper::_power_off()
{
  digitalWrite(_pin_1, 0);
  digitalWrite(_pin_2, 0);
  digitalWrite(_pin_3, 0);
  digitalWrite(_pin_4, 0);
}
