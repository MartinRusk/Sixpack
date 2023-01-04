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
  step_act = 0;
  step_target = 0;
  is_modulo = false;
  is_limited = false;
  steps_turn = 4096;
  steps_modulo = 0;
  feed_const = steps_turn / 360.0;
  upper_limit = 0x7fffffff;
  lower_limit = 0x80000001;
  delay_step = 1250;
  delay_powersave = 0;
  time_last_step = micros() + delay_step;
  
  // Arduino pins for the motor control connection:
  motor_pin_1 = pin_1;
  motor_pin_2 = pin_2;
  motor_pin_3 = pin_3;
  motor_pin_4 = pin_4;

  // setup the pins on the microcontroller:
  pinMode(motor_pin_1, OUTPUT);
  pinMode(motor_pin_2, OUTPUT);
  pinMode(motor_pin_3, OUTPUT);
  pinMode(motor_pin_4, OUTPUT);

  // and start in idle mode
  power_off();
}

// variable steps (e.g. with gear)
Stepper::Stepper(uint8_t pin_1, uint8_t pin_2, uint8_t pin_3, uint8_t pin_4, uint16_t steps) : Stepper(pin_1, pin_2, pin_3, pin_4) 
{
  steps_turn = steps;
}

// cyclic handle of motion (call in loop)
void Stepper::handle()
{
  // check if next step can be executed (rate limitation)
  unsigned long now = micros();
  if (now > time_last_step + delay_step)
  {
    // do one step in the right direction
    int32_t diff = diff_modulo(step_target - step_act);
    if (diff > 0)
    {
      step_act = trim_modulo(step_act + 1);
      time_last_step = now;
      step();
    }
    if (diff < 0)
    {
      step_act = trim_modulo(step_act - 1);
      time_last_step = now;
      step();
    }
    if ((delay_powersave > 0) && (now > time_last_step + delay_powersave))
    {
      power_off();
    }
  }
}

// set new target position
void Stepper::set_inc(int32_t pos)
{
  if (is_limited)
  {
    pos = min(max(pos, lower_limit), upper_limit);
  }
  step_target = trim_modulo(pos);
}

// set relative target position
void Stepper::set_inc_rel(int32_t steps)
{
  set_inc(step_target + steps);
}

// set new target position
void Stepper::set_pos(float pos)
{
  set_inc((int32_t)(pos * feed_const));
}

// set new target position relative
void Stepper::set_pos_rel(float pos)
{
  set_inc_rel((int32_t)(pos * feed_const));
}

// automatic trim position in modulo range
int32_t Stepper::trim_modulo(int32_t pos)
{
  if (is_modulo) 
  {
    if (pos >= steps_modulo)
    {
      pos -= steps_modulo;
    }
    if (pos < 0)
    {
      pos += steps_modulo;
    }
  }
  return pos;
}

// automatic trim position difference in modulo range
int32_t Stepper::diff_modulo(int32_t diff)
{
  if (is_modulo)
  {
    if (diff > (steps_modulo >> 1))
    {
      diff -= steps_modulo;
    }
    if (diff < -(steps_modulo >> 1))
    {
      diff += steps_modulo;
    }
  }
  return diff;
}

// return actual position
int32_t Stepper::get_inc()
{
  return (step_act);
}

// get new cuurrent position
float Stepper::get_pos()
{
  return (float) step_act / feed_const;
}

// check if target position reached
bool Stepper::in_target()
{
  return (step_target == step_act);
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
  step_act = 0;
  step_target = 0;
}

// adjust position by some steps
void Stepper::adjust(int32_t steps)
{
  step_act -= steps;
  step();
}

// override stepper frequency
void Stepper::set_freq(uint16_t freq)
{
  delay_step = 1000000UL / freq;
}

// make this a modulo axis
void Stepper::set_modulo(uint16_t steps)
{
  is_modulo = true;
  is_limited = false;
  steps_modulo = steps;
}

// remove limits and modulo
void Stepper::set_unlimited()
{
  is_limited = false;
  is_modulo = false;
  lower_limit = 0x80000001;
  upper_limit = 0x7fffffff;
  steps_modulo = 0;
}

// set software limits
void Stepper::set_limit(float lower, float upper)
{
  is_limited = true;
  is_modulo = false;
  lower_limit = lower * feed_const;
  upper_limit = upper * feed_const; 
}

// Feedrate per turn (default 360)
void Stepper::set_feed_const(float feed)
{
  feed_const = steps_turn / feed;
}

// invert direction
void Stepper::reverse_dir(bool neg)
{
  neg_dir = neg;
}

void Stepper::set_powersave(uint16_t seconds)
{
  delay_powersave = 1000000UL * seconds;
}

// execute one step
// TODO: decouple from act_step?
void Stepper::step()
{
  int phase = (int) (step_act & 0x07);
  if (neg_dir)
  {
    // invert direction
    phase = 7 - phase;
  }
  digitalWrite(motor_pin_1, phase_scheme[phase][0]);
  digitalWrite(motor_pin_2, phase_scheme[phase][1]);
  digitalWrite(motor_pin_3, phase_scheme[phase][2]);
  digitalWrite(motor_pin_4, phase_scheme[phase][3]);
}

// switch power off
void Stepper::power_off()
{
  digitalWrite(motor_pin_1, 0);
  digitalWrite(motor_pin_2, 0);
  digitalWrite(motor_pin_3, 0);
  digitalWrite(motor_pin_4, 0);
}
