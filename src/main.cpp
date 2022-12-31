#include <Arduino.h>
#include <Stepper.h>
// #include <Servo.h>
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

// // Servo drives
// Servo srvSpeed;
// Servo srvVario;

// Input devices (TODO)
Button btnUp(15);
Button btnDn(16);
Encoder encBaro(8, 9, 4);
Encoder encHeading(11, 10, 4);

// Altimeter (steps / foot)
#define INHG2HPA 33.863886666667

void handle()
{
  stpSpeed.handle();
  stpRoll.handle();
  stpPitch.handle();
  stpAltitude.handle();
  stpVario.handle();
  stpGyro.handle();
  stpHeading.handle();
}

bool in_target()
{
  return stpSpeed.in_target() &&
         stpRoll.in_target() &&
         stpPitch.in_target() &&
         stpAltitude.in_target() &&
         stpVario.in_target() &&
         stpGyro.in_target() &&
         stpHeading.in_target();
}

void wait()
{
  while (!in_target())
  {
    handle();
  }
}

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
  xp.registerDataRef("sim/cockpit2/gauges/indicators/pitch_electric_deg_pilot", XPL_READ, 50, 0.5, &pitch_electric_deg_pilot);
  // airspeed
  xp.registerDataRef("sim/cockpit2/gauges/indicators/airspeed_kts_pilot", XPL_READ, 50, 0.5, &airspeed_kts_pilot);
  // variometer
  xp.registerDataRef("sim/cockpit2/gauges/indicators/vvi_fpm_pilot", XPL_READ, 50, 1.0, &vvi_fpm_pilot);
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

  // for reset
  delay(5000);

  // speed indicator
  stpSpeed.set_freq(800);
  stpSpeed.set_feedrate(185.8);
  stpSpeed.set_limit_feed(0, 165);
  stpSpeed.reset();
  // stpSpeed.move(30.0);
  // stpSpeed.wait();
  // stpSpeed.move(0.0);
  // stpSpeed.wait();

  // attitude indicator
  stpRoll.set_freq(800);
  stpRoll.set_dir(true);
  stpRoll.set_feedrate(360.0);
  stpRoll.set_limit(-500, 500);
  stpRoll.calibrate(600);
  stpPitch.set_freq(800);
  stpPitch.set_dir(true);
  stpPitch.set_feedrate(1000.0);
  stpPitch.set_limit(-70, 70);
  stpPitch.calibrate(96);

  // // altimeter
  stpAltitude.set_freq(800);
  stpAltitude.set_dir(true);
  stpAltitude.set_feedrate(1000.0);
  stpAltitude.reset();
  // stpAltitude.move(200.0);
  // stpAltitude.wait();
  // stpAltitude.move(0.0);
  // stpAltitude.wait();

  // variometer
  stpVario.set_freq(800);
  stpVario.reset();
  stpVario.set_feedrate(4235.3);
  stpVario.set_limit_feed(-2000, 2000);
  stpVario.reset();
  // stpVario.move(500.0);
  // stpVario.wait();
  // stpVario.move(-500.0);
  // stpVario.wait();
  // stpVario.move(0.0);
  // stpVario.wait();

  // gyro
  stpGyro.set_freq(800);
  stpGyro.set_modulo(4096);
  stpGyro.reset();
  // stpGyro.move(180.0);
  // stpGyro.wait();
  // stpGyro.move(0.0);
  // stpGyro.wait();

  stpHeading.set_freq(800);
  stpHeading.set_modulo(4096);
  stpHeading.set_dir(true);
  // stpHeading.move(-3.0);
  // stpHeading.wait();
  stpHeading.reset();
  stpHeading.move(90.0);
  stpHeading.wait();
  stpHeading.move(0.0);
  stpHeading.wait();

  xp_running = false;
  next_check = millis() + 2000;
}

// Main loop
#if TEST == 1

int i = 0;
int dir = +1;
void loop()
{
  // i += dir;
  // if ((dir*(i+=dir)) >= 20) dir = -dir;
  // stpRoll.move((float)i);
  // stpPitch.move((float)i * 0.5);
  // stpSpeed.move(60 + 2 * (float)i);
  // stpVario.move((float)i * 30.0);
  // stpAltitude.move((float)i*10);
  // stpGyro.move((float)i);
  // stpHeading.move((float)i);
  // while (!in_target()) handle();
  // if (i == 0) delay(5000);
}

#else

void loop()
{
  // handle interface
  xp.xloop();

  // if XP not running: zero instruments and enable adjustment for altimeter
  if (!check_xp_running())
  {
    // zero indicators
    roll_electric_deg_pilot = 0.0;
    pitch_electric_deg_pilot = 0.0;
    airspeed_kts_pilot = 0.0;
    vvi_fpm_pilot = 0.0;
    altitude_ft_pilot = 0.0;
    barometer_setting_in_hg_pilot = 0.0;
    barometer_setting_in_hg_pilot = 0.0;
    heading_electric_deg_mag_pilot = 0.0;
    heading_dial_deg_mag_pilot = 0.0;

    // TODO: final concept for adjusting
    if (encBaro.up() || btnUp.is_pressed())
    {
      altitude_ft_pilot = 20;
    }
    if (encBaro.down() || btnDn.is_pressed())
    {
      altitude_ft_pilot = -20;
    }
    // wait until altitude corrected
    stpAltitude.move(altitude_ft_pilot);
    stpAltitude.wait();
    stpAltitude.reset();
  }
  else
  {
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
  }

  // set instrument values
  stpSpeed.move(airspeed_kts_pilot - 40.0);
  stpRoll.move(roll_electric_deg_pilot);
  stpPitch.move(pitch_electric_deg_pilot);
  stpAltitude.move(altitude_ft_pilot);
  stpVario.move(vvi_fpm_pilot);
  stpGyro.move(heading_electric_deg_mag_pilot);
  stpHeading.move(heading_dial_deg_mag_pilot - heading_electric_deg_mag_pilot);

  // handle all steppers
  handle();
  encBaro.handle();
  encHeading.handle();
}
#endif
