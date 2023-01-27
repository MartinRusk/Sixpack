#ifndef Button_h
#define Button_h

class Button
{
public:
  Button(uint8_t pin);
  void handle();
  bool pressed();
  bool released();

private:
  uint8_t _pin;
  uint8_t _state;
  uint8_t _transition;
};

#endif
