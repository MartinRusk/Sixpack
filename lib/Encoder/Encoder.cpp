#include <Arduino.h>
#include "Encoder.h"

// Encoders
Encoder::Encoder(uint8_t pin1, uint8_t pin2, uint8_t pulses)
{
  _pin1 = pin1;
  _pin2 = pin2;
  _pulses = pulses;
  _count = 0;
  _state = 0;
  pinMode(_pin1, INPUT_PULLUP);
  pinMode(_pin2, INPUT_PULLUP);
}

void Encoder::handle()
{
  // collect new state
  _state = ((_state & 0x03) << 2) | (!digitalRead(_pin2) << 1) | (!digitalRead(_pin1));
  // evaluate state change
  switch (_state)
  {
  case 0:
  case 5:
  case 10:
  case 15:
    break;
  case 1:
  case 7:
  case 8:
  case 14:
    _count++;
    break;
  case 2:
  case 4:
  case 11:
  case 13:
    _count--;
    break;
  case 3:
  case 12:
    _count += 2;
    break;
  default:
    _count -= 2;
    break;
  }
}

int16_t Encoder::pos()
{
  return (_count);
}

bool Encoder::up()
{
  if (_count >= _pulses)
  {
    _count -= _pulses;
    return true;
  }
  return false;
}

bool Encoder::down()
{
  if (_count <= -_pulses)
  {
    _count += _pulses;
    return true;
  }
  return false;
}
