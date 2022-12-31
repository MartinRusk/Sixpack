#ifndef Button_h
#define Button_h

class Button
{
public:
  Button(uint8_t pin);
  bool is_pressed();

private:
  uint8_t _pin;
  uint8_t _state;
};

#endif
