#ifndef Switch_h
#define Switch_h

class Switch
{
public:
  Switch(uint8_t pin);
  bool is_on();

private:
  uint8_t _pin;
  uint8_t _state;
};

#endif
