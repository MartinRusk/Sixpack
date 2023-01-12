#ifndef Stepper_h
#define Stepper_h

class Stepper
{
public:
  Stepper(uint8_t pin_1, uint8_t pin_2, uint8_t pin_3, uint8_t pin_4);
  Stepper(uint8_t pin_1, uint8_t pin_2, uint8_t pin_3, uint8_t pin_4, uint16_t steps);
  void handle();
  void set_inc(int32_t pos);
  void set_inc_rel(int32_t steps);
  int32_t get_inc();
  void set_pos(float pos);
  void set_pos_rel(float pos);
  float get_pos();
  void move();
  bool in_target();
  void reset();
  void adjust(int32_t steps);
  void set_feed_const(float feed);
  void set_modulo(uint16_t steps);
  void set_unlimited();
  void set_limit(float lower, float upper);
  void set_freq(uint16_t freq);
  void reverse_dir(bool neg);
  void set_powersave(uint16_t seconds);

private:
  void _step();
  int32_t _trim_modulo(int32_t pos);
  int32_t _diff_modulo(int32_t diff);
  void _power_off();
  uint16_t _steps_turn;
  int32_t _step_act;
  int32_t _step_target;
  bool _is_modulo;
  bool _is_limited;
  int32_t _steps_modulo;
  int32_t _upper_limit;
  int32_t _lower_limit;
  float _feed_const;
  bool _neg_dir;

  // motor pin numbers
  uint8_t _pin_1;
  uint8_t _pin_2;
  uint8_t _pin_3;
  uint8_t _pin_4;
  
  //timing
  unsigned long _delay_powersave;
  unsigned long _delay_step;
  unsigned long _time_last_step;
};

#endif