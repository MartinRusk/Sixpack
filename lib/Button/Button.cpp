#include <Arduino.h>
#include "Button.h"

#define DEBOUNCE_DELAY 200

enum {eNone, ePressed, eReleased};

// Buttons
Button::Button(uint8_t pin)
{
  _pin = pin;
  _state = 0;
  _transition = 0;
  pinMode(_pin, INPUT_PULLUP);
}

void Button::handle()
{
  if (!digitalRead(_pin))
  {
    if (_state == 0)
    {
      _state = DEBOUNCE_DELAY;
      _transition = ePressed;
    }
  }
  else if (_state > 0)
  {
    if (--_state == 0)
    {
      _transition = eReleased;
    }
  }
}

bool Button::pressed()
{
  if (_transition == ePressed)
  {
    _transition = eNone;
    return true;
  }
  return false;
}

bool Button::released()
{
  if (_transition == eReleased)
  {
    _transition = eNone;
    return true;
  }
  return false;
}