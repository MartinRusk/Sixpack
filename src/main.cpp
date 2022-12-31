#include <Arduino.h>
#include <Stepper.h>
#include <Servo.h>
#include <Encoder.h>
#include <Button.h>
#include <XPLDirect.h>

// activate test mode (alternative loop function)
#define TEST 1

// XPLDirect connection
XPLDirect xp(&Serial);

// Stepper motors
Stepper stpSpeed(22, 23, 24, 25);
Stepper stpRoll(26, 27, 28, 29);
Stepper stpPitch(4, 5, 6, 7);
Stepper stpAltitude(A0, A1, A2, A3);
Stepper stpVario(14, 14, 14, 14);
Stepper stpGyro(14, 14, 14, 14);
Stepper stpHeading(14, 14, 14, 14);

// Servo drives
Servo srvSpeed;
Servo srvVario;

// Input devices (TODO)
Button btnUp(15);
Button btnDn(16);
Encoder encBaro(8, 9, 4);
Encoder encHeading(14, 14, 4);

// Speed Indicator
#define SPEED_PIN 10
#define SPEED_MIN 40.0
#define SPEED_MAX 175.0

// Variometer
#define VARIO_PIN 14
#define VARIO_MIN -1800.0
#define VARIO_MAX 1800.0

// Attitude Indicator
#define CAL_ROLL 600   // half mechanical range
#define MAX_ROLL 500   // full usable range
#define SCALE_ROLL -12 // steps/°
#define CAL_PITCH 96
#define MAX_PITCH 70
#define SCALE_PITCH -4

// Altimeter (steps / foot)
#define SCALE_ALT 4.096
#define INHG2HPA 33.863886666667

void handle()
{
  stpSpeed.handle();
  stpRoll.handle();
  stpPitch.handle();
  stpAltitude.handle();
  stpGyro.handle();
  stpHeading.handle();
  stpVario.handle();
}

bool in_target()
{
  return
    stpSpeed.in_target() && 
    stpRoll.in_target() && 
    stpPitch.in_target()&& 
    stpAltitude.in_target() && 
    stpGyro.in_target() &&
    stpHeading.in_target() &&
    stpVario.in_target();
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

// // set roll angle (°)
// void set_roll(float angle)
// {
//   int16_t pos = (int16_t)(angle * SCALE_ROLL);
//   pos = min(max(pos, -MAX_ROLL), MAX_ROLL);
//   stpRoll.move_abs(pos);
// }

// // set pitch angle (°)
// void set_pitch(float angle)
// {
//   int32_t pos = (int32_t)(angle * SCALE_PITCH);
//   pos = min(max(pos, -MAX_PITCH), MAX_PITCH);
//   stpPitch.move_abs(pos);
// }

// handle steppers and wait until pitch and roll settled (fot testing)
// void wait()
// {
//   while (!stpRoll.in_target() || !stpPitch.in_target())
//   {
//     stpRoll.handle();
//     stpPitch.handle();
//   }
// }

// set altitude (feet)
// void set_altitude(float altitude)
// {
//   int32_t pos = (int32_t)(altitude * SCALE_ALT);
//   stpAltitude.move_abs(pos);
// }

int servo_time(float val, float min, float max)
{
  int time;
  if (val < min)
  {
    time = MIN_PULSE_WIDTH;
  }
  else if (val > max)
  {
    time = MAX_PULSE_WIDTH;
  }
  else
  {
    time = MIN_PULSE_WIDTH + (int)((val - min) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / (max - min));
  }
  return time;
}

// set airspeed (kts)
void set_speed(float speed)
{
  srvSpeed.writeMicroseconds(servo_time(speed, SPEED_MIN, SPEED_MAX));
}

// set vertical speed (fpm)
void set_vario(float vario)
{
  srvVario.writeMicroseconds(servo_time(vario, VARIO_MIN, VARIO_MAX));
}

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

  // servos
  srvSpeed.attach(SPEED_PIN);
  srvSpeed.writeMicroseconds(MIN_PULSE_WIDTH);
  srvVario.attach(VARIO_PIN);
  srvVario.writeMicroseconds((MAX_PULSE_WIDTH + MIN_PULSE_WIDTH) / 2);

  // calibrate attitude indicator
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

  // initialize altimeter
  stpAltitude.set_freq(800);
  stpAltitude.set_feedrate(100.0);
  stpAltitude.reset();

  // Sweep instruments once to check function
  set_speed(100.0);
  set_vario(500.0);
  delay(200);
  set_vario(-500.0);
  delay(200);
  set_speed(0.0);
  set_vario(0.0);

  // set_altitude(200.0);
  stpAltitude.move(200.0);
  stpAltitude.wait();
  // set_altitude(0.0);
  stpAltitude.move(0.0);
  stpAltitude.wait();
  
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
  if ((dir*(i+=dir)) >= 60) dir = -dir;
  stpRoll.move((float)i);
  stpPitch.move((float)i * 0.5);
  stpSpeed.move(60 + 2 * (float)i);
  stpVario.move((float)i * 30.0);
  stpAltitude.move((float)i*10);
  stpGyro.move((float)i);
  stpHeading.move((float)i);
  while (!in_target()) handle();
  if (i == 0) delay(2000);
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
    // set_altitude(altitude_ft_pilot);
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
  // set_roll(roll_electric_deg_pilot);
  // set_pitch(pitch_electric_deg_pilot);
  stpRoll.move(roll_electric_deg_pilot);
  stpPitch.move(pitch_electric_deg_pilot);

  set_speed(airspeed_kts_pilot);
  set_vario(vvi_fpm_pilot);

  // set_altitude(altitude_ft_pilot);
  stpAltitude.move(altitude_ft_pilot);

  // handle all steppers 
  handle();
}
#endif
