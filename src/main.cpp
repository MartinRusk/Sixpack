#include <Arduino.h>
#include <Stepper.h>
#include <Encoder.h>
#include <Button.h>
#include <XPLDirect.h>

// activate test mode (alternative loop function)
#define TEST 0

// XPLDirect connection
XPLDirect xp(&Serial);

// Stepper motors
Stepper stpSpeed(22, 23, 24, 25);
Stepper stpRoll(26, 27, 28, 29);
Stepper stpPitch(30, 31, 32, 33);
Stepper stpAltitude(34, 35, 36, 37);
Stepper stpVario(38, 39, 40, 41);
Stepper stpGyro(42, 43, 44, 45);
Stepper stpHeading(46, 47, 48, 49);

// Input devices (TODO)
Button btnUp(15);
Button btnDn(16);
Encoder encBaro(8, 9, 4);
Encoder encHeading(50, 51, 4);

// Altimeter calculation
#define INHG2HPA 33.863886666667

// synchronized variables
float sim_time_sec;
float roll_electric_deg_pilot;
float pitch_electric_deg_pilot;
float airspeed_kts_pilot;
float vvi_fpm_pilot;
float altitude_ft_pilot;
float barometer_setting_in_hg_pilot;
float heading_electric_deg_mag_pilot;
float heading_dial_deg_mag_pilot;

// commands
int barometer_down;
int barometer_up;
int barometer_std;
int heading_down;
int heading_up;

// running check
bool xp_running;
long next_check;
long last_running;
float last_time_sec;
#define XP_TIMEOUT 30

// handle all steppers and encoders
void handle_all()
{
  stpSpeed.handle();
  stpRoll.handle();
  stpPitch.handle();
  stpAltitude.handle();
  stpVario.handle();
  stpGyro.handle();
  stpHeading.handle();
  encBaro.handle();
  encHeading.handle();
}

// check all steppers for target position reached
bool all_in_target()
{
  return stpSpeed.in_target() &&
         stpRoll.in_target() &&
         stpPitch.in_target() &&
         stpAltitude.in_target() &&
         stpVario.in_target() &&
         stpGyro.in_target() &&
         stpHeading.in_target();
}

// move alle steppers to target position (blocking)
void move_all()
{
  while (!all_in_target())
  {
    xp.xloop();
    handle_all();
  }
}

// check if XPlane ist running, otherwise timeout
bool check_xp_running()
{
  long time = millis();
  if (time > next_check)
  {
    // check all 2 seconds
    next_check = time + 2000;
    // XP alive?
    if (sim_time_sec != last_time_sec)
    {
      last_time_sec = sim_time_sec;
      last_running = time;
    }
    // timeout reached?
    xp_running = (time < last_running + XP_TIMEOUT * 1000l);
  }
  return xp_running;
}

// initialization
void setup()
{
  // initialize the serial port and register device
  Serial.begin(XPLDIRECT_BAUDRATE);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"
  xp.begin("Sixpack");
#pragma GCC diagnostic pop

  // register DataRefs
  xp.registerDataRef("sim/time/zulu_time_sec", XPL_READ, 1000, 1.0, &sim_time_sec);
  // attitude
  xp.registerDataRef("sim/cockpit2/gauges/indicators/roll_electric_deg_pilot", XPL_READ, 50, 0.2, &roll_electric_deg_pilot);
  xp.registerDataRef("sim/cockpit2/gauges/indicators/pitch_electric_deg_pilot", XPL_READ, 50, 0.2, &pitch_electric_deg_pilot);
  // airspeed
  xp.registerDataRef("sim/cockpit2/gauges/indicators/airspeed_kts_pilot", XPL_READ, 50, 0.2, &airspeed_kts_pilot);
  // variometer
  xp.registerDataRef("sim/cockpit2/gauges/indicators/vvi_fpm_pilot", XPL_READ, 50, 10.0, &vvi_fpm_pilot);
  // altimeter
  xp.registerDataRef("sim/cockpit2/gauges/indicators/altitude_ft_pilot", XPL_READ, 50, 1.0, &altitude_ft_pilot);
  xp.registerDataRef("sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot", XPL_READ, 50, 0.01, &barometer_setting_in_hg_pilot);
  // gyro
  xp.registerDataRef("sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot", XPL_READ, 50, 0.2, &heading_electric_deg_mag_pilot);
  xp.registerDataRef("sim/cockpit2/autopilot/heading_dial_deg_mag_pilot", XPL_READ, 50, 0.2, &heading_dial_deg_mag_pilot);

  // register Commands
  barometer_down = xp.registerCommand("sim/instruments/barometer_down");
  barometer_up = xp.registerCommand("sim/instruments/barometer_up");
  barometer_std = xp.registerCommand("sim/instruments/barometer_std");
  heading_down = xp.registerCommand("sim/autopilot/heading_down");
  heading_up = xp.registerCommand("sim/autopilot/heading_up");

  // speed indicator
  stpSpeed.set_feedrate(185.8);
  stpSpeed.set_limit_feed(0, 165);

  // attitude indicator
  // stpRoll.reverse_dir(true);
  stpRoll.set_feedrate(360.0);
  stpRoll.set_limit_feed(-45, 45);
  // stpPitch.reverse_dir(true);
  stpPitch.set_feedrate(1000.0);
  stpPitch.set_limit(-17, 17);
  
  // altimeter
  stpAltitude.reverse_dir(true);
  stpAltitude.set_feedrate(1000.0);

  // variometer
  stpVario.set_feedrate(4235.3);
  stpVario.set_limit_feed(-2000, 2000);

  // gyro
  stpGyro.set_modulo(4096);
  stpHeading.set_freq(800);
  stpHeading.set_modulo(4096);
  stpHeading.reverse_dir(true);

  // init sequence -> move all indicators and calibrate horizon
  stpSpeed.set_pos(60.0);
  stpRoll.set_pos_rel(-1200); // move to block
  stpPitch.set_pos_rel(0);
  stpAltitude.set_pos(200.0);
  stpVario.set_pos(500.0);
  stpGyro.set_pos(90.0);
  stpHeading.set_pos(-90.0);
  move_all();
  stpSpeed.set_pos(0.0);
  stpRoll.set_pos_rel(590); // center
  stpPitch.set_pos_rel(0);
  stpAltitude.set_pos(0.0);
  stpVario.set_pos(-500.0);
  stpGyro.set_pos(0.0);
  move_all();
  stpPitch.set_pos_rel(-200); // move to block
  stpVario.set_pos(0);
  stpHeading.set_pos(0.0);
  move_all();
  stpPitch.set_pos_rel(90); // center
  move_all();
  
  // reset all steppers to zero
  stpSpeed.reset();
  stpRoll.reset();
  stpPitch.reset();
  stpAltitude.reset();
  stpVario.reset();
  stpGyro.reset();
  stpHeading.reset();

  // initialize XP run check
  xp_running = false;
  next_check = millis() + 2000;
}

// Main loop
#if TEST == 1

void loop()
{
  stpRoll.set_pos(45);
  stpPitch.set_pos(15);
  stpSpeed.set_pos(205-40);
  stpVario.set_pos(2000);
  stpAltitude.set_pos(1000);
  stpGyro.set_pos(180);
  stpHeading.set_pos(-180);
  move_all();
  delay(1000);
  stpRoll.set_pos(0);
  stpPitch.set_pos(0);
  stpSpeed.set_pos(0);
  stpVario.set_pos(0);
  stpAltitude.set_pos(0);
  stpGyro.set_pos(0);
  stpHeading.set_pos(0);
  move_all();
  delay(5000);
}

#else

void loop()
{
  // handle interface
  xp.xloop();

  // if XP not running: zero all instruments
  if (!check_xp_running())
  {
    roll_electric_deg_pilot = 0.0;
    pitch_electric_deg_pilot = 0.0;
    airspeed_kts_pilot = 0.0;
    vvi_fpm_pilot = 0.0;
    altitude_ft_pilot = 0.0;
    barometer_setting_in_hg_pilot = 0.0;
    barometer_setting_in_hg_pilot = 0.0;
    heading_electric_deg_mag_pilot = 0.0;
    heading_dial_deg_mag_pilot = 0.0;
  }

  // barometer up/down
  if (encBaro.up())
  {
    xp.commandTrigger(barometer_up);
  }
  if (encBaro.down())
  {
    xp.commandTrigger(barometer_down);
  }

  // heading bug up/down
  if (encHeading.up())
  {
    xp.commandTrigger(heading_up);
  }
  if (encHeading.down())
  {
    xp.commandTrigger(heading_down);
  }

  // set instrument values
  stpSpeed.set_pos(airspeed_kts_pilot - 40.0);
  stpRoll.set_pos(roll_electric_deg_pilot);
  stpPitch.set_pos(pitch_electric_deg_pilot);
  stpAltitude.set_pos(altitude_ft_pilot);
  stpVario.set_pos(vvi_fpm_pilot);
  stpGyro.set_pos(heading_electric_deg_mag_pilot);
  stpHeading.set_pos(heading_dial_deg_mag_pilot - heading_electric_deg_mag_pilot);

  // handle all steppers and encoders
  handle_all();
}
#endif