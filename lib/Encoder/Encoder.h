#ifndef Encoder_h
#define Encoder_h

class Encoder
{
public:
  Encoder(uint8_t pin1, uint8_t pin2, uint8_t pulses);
  void handle();
  int16_t pos();
  bool up();
  bool down();

private:
  uint8_t _pin1, _pin2;
  uint8_t _pulses;
  uint8_t _state;
  int8_t _count;
};

#endif
