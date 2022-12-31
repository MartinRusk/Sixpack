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
Stepper::Stepper(uint8_t pin_1, uint8_t pin_2, uint8_t pin_3, uint8_t pin_4, uint16_t steps)
{
  // Initialize variables
  step_act = 0;
  step_target = 0;
  step_delay = 1500;
  step_next = micros() + step_delay;
  is_modulo = false;
  modulo_steps = 0;
  upper_limit = 0x3fffffff;
  lower_limit = 0x40000001;
  steps_turn = steps;
  feed = steps_turn / 360.0;
  
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
}

Stepper::Stepper(uint8_t pin_1, uint8_t pin_2, uint8_t pin_3, uint8_t pin_4)
{
  Stepper(pin_1, pin_2, pin_3, pin_4, 4096);
}

// cyclic handle of motion (call in loop)
void Stepper::handle()
{
  // check if next step can be executed (rate limitation)
  unsigned long now = micros();
  if (now > step_next)
  {
    // next time step
    step_next = now + step_delay;
    // do one step in the right direction
    int32_t diff = diff_modulo(step_target - step_act);
    if (diff > 0)
    {
      step_act = trim_modulo(step_act + 1);
    }
    if (diff < 0)
    {
      step_act = trim_modulo(step_act - 1);
    }
    // execute step
    step(step_act);
  }
}

// set new target position
void Stepper::move_abs(int32_t pos)
{
  pos = min(max(pos, lower_limit), upper_limit);
  step_target = trim_modulo(pos);
}

// set relative target position
void Stepper::move_rel(int32_t steps)
{
  step_target = trim_modulo(step_target + steps);
}

// set new target position
void Stepper::move(float pos)
{
  move_abs((int32_t)(pos * feed));
}

// automatic trim position in modulo range
int32_t Stepper::trim_modulo(int32_t pos)
{
  if (is_modulo) 
  {
    if (pos >= modulo_steps)
    {
      pos -= modulo_steps;
    }
    if (pos < 0)
    {
      pos += modulo_steps;
    }
  }
  return pos;
}

// automatic trim position difference in modulo range
int32_t Stepper::diff_modulo(int32_t diff)
{
  if (is_modulo)
  {
    if (diff > (modulo_steps >> 1))
    {
      diff -= modulo_steps;
    }
    if (diff < -(modulo_steps >> 1))
    {
      diff += modulo_steps;
    }
  }
  return diff;
}

// return actual position
int32_t Stepper::pos()
{
  return (step_act);
}

// check if target position reached
bool Stepper::in_target()
{
  return (step_target == step_act);
}

// wait and handle steps until target position reached
void Stepper::wait()
{
  while (!in_target())
  {
    handle();
  }
}

// set actual position to zero
void Stepper::reset()
{
  step_act = 0;
  step_target = 0;
}

// make a calibration until block and return to center position
void Stepper::calibrate(int32_t range)
{
  move_rel(2 * range);
  wait();
  move_rel(-range);
  wait();
  reset();
}

// override stepper frequency
void Stepper::set_freq(uint16_t freq)
{
  step_delay = 1000000UL / freq;
}

// Make this a modulo axis
void Stepper::set_modulo(uint16_t steps)
{
  is_modulo = true;
  modulo_steps = steps;
}

void Stepper::set_limit(int32_t lower, int32_t upper)
{
  lower_limit = lower;
  upper_limit = upper; 
}

void Stepper::set_limit_feed(float lower, float upper)
{
  lower_limit = lower * feed;
  upper_limit = upper * feed; 
}

// Feedrate per turn (default 360)
void Stepper::set_feedrate(float feed)
{
  feed = steps_turn / feed;
}

// Ivert direction
void Stepper::set_dir(bool neg)
{
  dir = neg;
}

// execute one step
void Stepper::step(int32_t step)
{
  int phase = (int) (step & 0x07);
  if (dir)
  {
    // invert direction
    phase = 7 - phase;
  }
  digitalWrite(motor_pin_1, phase_scheme[phase][0]);
  digitalWrite(motor_pin_2, phase_scheme[phase][1]);
  digitalWrite(motor_pin_3, phase_scheme[phase][2]);
  digitalWrite(motor_pin_4, phase_scheme[phase][3]);
}
