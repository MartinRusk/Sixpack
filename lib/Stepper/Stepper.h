#ifndef Stepper_h
#define Stepper_h

class Stepper
{
public:
  Stepper(uint8_t pin_1, uint8_t pin_2, uint8_t pin_3, uint8_t pin_4);
  Stepper(uint8_t pin_1, uint8_t pin_2, uint8_t pin_3, uint8_t pin_4, uint16_t steps);
  void handle();
  void move_abs(int32_t pos);
  void move_rel(int32_t steps);
  int32_t pos();
  bool in_target();
  void wait();
  void reset();
  void calibrate(int32_t range);
  void set_freq(uint16_t freq);
  void set_dir(bool neg);
  void set_modulo(uint16_t steps);
  void set_limit(int32_t lower, int32_t upper);
  // float based motion
  void set_feedrate(float feed);
  void set_limit_feed(float lower, float upper);
  void move(float pos);

private:
  void step(int32_t step);
  int32_t trim_modulo(int32_t pos);
  int32_t diff_modulo(int32_t diff);
  int32_t step_act;
  int32_t step_target;
  bool is_modulo;
  int32_t modulo_steps;
  int32_t upper_limit;
  int32_t lower_limit;
  float feed;
  bool dir;
// motor pin numbers:
  uint8_t motor_pin_1;
  uint8_t motor_pin_2;
  uint8_t motor_pin_3;
  uint8_t motor_pin_4;
  uint16_t steps_turn;
  
  unsigned long step_delay;
  unsigned long step_next;
};

#endif