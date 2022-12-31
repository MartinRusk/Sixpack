#include <Arduino.h>
#include "Button.h"

#define DEBOUNCE_DELAY 50;

// Buttons
Button::Button(uint8_t pin)
{
  _pin = pin;
  _state = 0;
  pinMode(_pin, INPUT_PULLUP);
}

bool Button::is_pressed()
{
  if (!digitalRead(_pin))
  {
    if (_state == 0)
    {
      _state = DEBOUNCE_DELAY;
      return true;
    }
  }
  else if (_state > 0)
  {
    _state--;
  }
  return false;
}
