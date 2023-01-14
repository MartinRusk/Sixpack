#include <Arduino.h>
#include <Stepper.h>
#include <Encoder.h>
#include <Button.h>
#include <Switch.h>
#include <XPLDirect.h>

// XPLDirect connection
XPLDirect xp(&Serial);

// Stepper motors
Stepper stpSpeed(22, 23, 24, 25);
Stepper stpRoll(26, 27, 28, 29);
Stepper stpPitch(30, 31, 32, 33);
Stepper stpAltitude(34, 35, 36, 37);
Stepper stpBaro(A8, A9, A10, A11);
Stepper stpVario(38, 39, 40, 41);
Stepper stpGyro(42, 43, 44, 45);
Stepper stpHeading(46, 47, 48, 49);
Stepper stpTurn(2, 3, 4, 5);
Stepper stpBall(18, 19, 20, 21);

// Input devices
Button btnBaro(A13);
Encoder encBaro(A14, A15, 4);
Button btnHeading(52);
Encoder encHeading(50, 51, 4);
Switch swEnable(A12);

// Altimeter calculation in_hg -> hPa
#define INHG2HPA 33.863886666667
#define hPa2baro(p) (p - 1013) / INHG2HPA

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
float turn_rate_roll_deg_pilot;
float slip_deg;

// commands
int barometer_down;
int barometer_up;
int barometer_std;
int heading_down;
int heading_up;
int heading_sync;

// for rate limiting of commands
long time_last_command;
#define COMMAND_TIMEOUT 50

// running check
bool xp_running;
long time_next_check;
long time_last_running;
float last_sim_time_sec;
#define XP_TIMEOUT 30

// adjustment
int adjust;

// handle all steppers and encoders
void handle_all()
{
  // Steppers
  stpSpeed.handle();
  stpRoll.handle();
  stpPitch.handle();
  stpAltitude.handle();
  stpBaro.handle();
  stpVario.handle();
  stpGyro.handle();
  stpHeading.handle();
  stpTurn.handle();
  stpBall.handle();
  // Encoders
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
         stpBaro.in_target() &&
         stpVario.in_target() &&
         stpGyro.in_target() &&
         stpHeading.in_target() &&
         stpTurn.in_target() &&
         stpBall.in_target();
}

// move alle steppers to target position (blocking)
void move_all()
{
  while (!all_in_target())
  {
    // serve XPLDirect
    xp.xloop();
    // and move all steppers
    handle_all();
  }
}

// check if XPlane is running, otherwise timeout
bool check_xp_running()
{
  long time = millis();
  if (time > time_next_check)
  {
    // check every two seconds
    time_next_check = time + 2000;
    // XP alive? sim_time_sec is updated every second
    if (sim_time_sec != last_sim_time_sec)
    {
      last_sim_time_sec = sim_time_sec;
      time_last_running = time;
      xp_running = true;
    }
    // timeout reached?
    if (time > time_last_running + XP_TIMEOUT * 1000l)
    {
      xp_running = false;
    }
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
  xp.registerDataRef("sim/cockpit2/gauges/indicators/pitch_electric_deg_pilot", XPL_READ, 50, 0.1, &pitch_electric_deg_pilot);
  // airspeed
  xp.registerDataRef("sim/cockpit2/gauges/indicators/airspeed_kts_pilot", XPL_READ, 50, 0.2, &airspeed_kts_pilot);
  // variometer
  xp.registerDataRef("sim/cockpit2/gauges/indicators/vvi_fpm_pilot", XPL_READ, 50, 1.0, &vvi_fpm_pilot);
  // altimeter
  xp.registerDataRef("sim/cockpit2/gauges/indicators/altitude_ft_pilot", XPL_READ, 50, 1.0, &altitude_ft_pilot);
  xp.registerDataRef("sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot", XPL_READ, 50, 0.01, &barometer_setting_in_hg_pilot);
  // gyro
  xp.registerDataRef("sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot", XPL_READ, 50, 0.2, &heading_electric_deg_mag_pilot);
  xp.registerDataRef("sim/cockpit2/autopilot/heading_dial_deg_mag_pilot", XPL_READ, 50, 0.2, &heading_dial_deg_mag_pilot);
  // turn coordinator
  xp.registerDataRef("sim/cockpit2/gauges/indicators/turn_rate_roll_deg_pilot", XPL_READ, 50, 0.2, &turn_rate_roll_deg_pilot);
  xp.registerDataRef("sim/cockpit2/gauges/indicators/slip_deg", XPL_READ, 50, 0.05, &slip_deg);

  // register Commands
  barometer_down = xp.registerCommand("sim/instruments/barometer_down");
  barometer_up = xp.registerCommand("sim/instruments/barometer_up");
  barometer_std = xp.registerCommand("sim/instruments/barometer_std");
  heading_down = xp.registerCommand("sim/autopilot/heading_down");
  heading_up = xp.registerCommand("sim/autopilot/heading_up");
  heading_sync = xp.registerCommand("sim/autopilot/heading_sync");

  // speed indicator (310° = 160kt)
  stpSpeed.set_feed_const(185.8);
  stpSpeed.set_backlash(36);
  // attitude indicator
  stpRoll.set_feed_const(360.0);
  stpRoll.set_backlash(24);
  stpPitch.set_feed_const(1000.0);
  // altimeter (1000ft/turn)
  stpAltitude.set_feed_const(1000.0);
  stpAltitude.set_backlash(20);
  // baro 130 hPa/turn (3.8 inHg) // 50/16 gear
  stpBaro.set_feed_const(130.0 / INHG2HPA * 16.0 / 50.0);
  // baro 3.8 inHg/turn // 50/16 gear
  // stpBaro.set_feed_const(3.8 * 16.0 / 50.0);
  stpBall.set_backlash(12);
  // variometer
  stpVario.set_feed_const(4235.3);
  stpVario.set_backlash(12);
  // gyro (°)
  stpGyro.set_modulo(4096);
  stpGyro.reverse_dir(true);
  stpGyro.set_backlash(15);
  stpHeading.set_modulo(4096);
  stpHeading.reverse_dir(true);
  stpHeading.set_backlash(20);
  // turn coordinator
  stpTurn.set_feed_const(360.0);
  stpTurn.set_backlash(12);
  stpBall.set_feed_const(360.0);

  // init sequence -> move all indicators and calibrate horizon
  stpSpeed.set_pos(60.0);
  stpRoll.set_inc_rel(-1200); // move to block
  stpPitch.set_inc_rel(0);
  stpAltitude.set_pos(200.0);
  stpBaro.set_pos(hPa2baro(1000)); // 1000hPa
  stpVario.set_pos(500.0);
  stpGyro.set_pos(90.0);
  stpHeading.set_pos(-90.0);
  stpTurn.set_pos(30.0);
  stpBall.set_pos(15.0);
  move_all();
  stpSpeed.set_pos(0.0);
  stpRoll.set_inc_rel(590); // center
  stpPitch.set_inc_rel(0);
  stpAltitude.set_pos(0.0);
  stpBaro.set_pos(hPa2baro(1020)); // 1020hPa
  stpVario.set_pos(-500.0);
  stpGyro.set_pos(0.0);
  stpTurn.set_pos(-30.0);
  stpBall.set_pos(-15.0);
  move_all();
  stpPitch.set_inc_rel(-200); // move to block
  stpBaro.set_pos(hPa2baro(1013));
  stpVario.set_pos(0.0);
  stpHeading.set_pos(0.0);
  stpTurn.set_pos(0.0);
  stpBall.set_pos(0.0);
  move_all();
  stpPitch.set_inc_rel(90); // center
  move_all();

  // reset all steppers to zero
  stpSpeed.reset();
  stpRoll.reset();
  stpPitch.reset();
  stpAltitude.reset();
  stpBaro.reset();
  stpVario.reset();
  stpGyro.reset();
  stpHeading.reset();
  stpTurn.reset();
  stpBall.reset();

  // set software limits
  // stpSpeed.set_limit(0, 205 - 40);
  stpSpeed.set_limit(0, (4 * 185.8) + 40); // allow 4 turns
  stpRoll.set_limit(-45, 45);
  stpPitch.set_limit(-17, 17);
  stpBaro.set_limit(hPa2baro(970), hPa2baro(1050)); // 970 - 1050 hPa
  // stpBaro.set_limit(28.6 - 29.92, 31.1 - 29.92); // 970 - 1050 hPa
  stpVario.set_limit(-2000, 2000);
  stpTurn.set_limit(-30.0, 30.0);
  stpBall.set_limit(-16.0, 16.0);

  // initialize XP run check
  xp_running = false;
  time_next_check = millis();
  time_last_command = millis();
  time_last_running = 0;

  // adjustment
  adjust = 0;

  // LED
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  // handle interface
  xp.xloop();

  // if XP running: set instruments
  if (check_xp_running() && swEnable.is_on())
  {
    // set instrument values
    stpSpeed.set_pos(airspeed_kts_pilot - 40.0);
    stpRoll.set_pos(roll_electric_deg_pilot);
    stpPitch.set_pos(pitch_electric_deg_pilot);
    stpAltitude.set_pos(altitude_ft_pilot);
    stpBaro.set_pos(barometer_setting_in_hg_pilot - 29.92);
    stpVario.set_pos(vvi_fpm_pilot);
    stpGyro.set_pos(heading_electric_deg_mag_pilot);
    stpHeading.set_pos(heading_dial_deg_mag_pilot - heading_electric_deg_mag_pilot);
    stpTurn.set_pos(turn_rate_roll_deg_pilot);
    stpBall.set_pos(slip_deg * 4.0);

    // Handle commands max all 50ms
    long now = millis();
    if (now > time_last_command + COMMAND_TIMEOUT)
    {
      // barometer up/down
      if (encBaro.up())
      {
        xp.commandTrigger(barometer_up);
        time_last_command = now;
      }
      if (encBaro.down())
      {
        xp.commandTrigger(barometer_down);
        time_last_command = now;
      }
      if (btnBaro.is_pressed())
      {
        xp.commandTrigger(barometer_std);
        time_last_command = now;
      } // heading bug up/down
      if (encHeading.up())
      {
        xp.commandTrigger(heading_up);
        time_last_command = now;
      }
      if (encHeading.down())
      {
        xp.commandTrigger(heading_down);
        time_last_command = now;
      }
      if (btnHeading.is_pressed())
      {
        xp.commandTrigger(heading_sync);
        time_last_command = now;
      }
    }
  }
  else // XP NOT running: set all to zero
  {
    stpSpeed.set_pos(0);
    stpRoll.set_pos(0);
    stpPitch.set_pos(0);
    stpAltitude.set_pos(0);
    stpBaro.set_pos(0);
    stpVario.set_pos(0);
    stpGyro.set_pos(0);
    stpHeading.set_pos(0);
    stpTurn.set_pos(0);
    stpBall.set_pos(0);

    // use gyro encoder for zero adjustment, cycle through instruments
    if (btnHeading.is_pressed())
    {
      adjust = (adjust + 1) % 12;
    }
    int32_t steps = 0;
    if (encHeading.up())
    {
      steps = 8;
    }
    if (encHeading.down())
    {
      steps = -8;
    }
    switch (adjust)
    {
    case 1:
      stpSpeed.adjust(steps);
      break;
    case 2:
      stpRoll.adjust(steps);
      break;
    case 3:
      stpPitch.adjust(steps);
      break;
    case 4:
      stpAltitude.adjust(steps * 51); // 100ft
      break;
    case 5:
      stpAltitude.adjust(steps);
      break;
    case 6:
      stpBaro.adjust(steps * 3.125);
      break;
    case 7:
      stpVario.adjust(steps);
      break;
    case 8:
      stpGyro.adjust(steps);
      break;
    case 9:
      stpHeading.adjust(steps);
      break;
    case 10:
      stpTurn.adjust(steps);
      break;
    case 11:
      stpBall.adjust(steps);
      break;
    default:;
    }
  }

  // handle all steppers and encoders
  handle_all();

  digitalWrite(LED_BUILTIN, swEnable.is_on());
}
