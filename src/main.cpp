#include <Arduino.h>
#include <Encoder.h>
#include <Button.h>
#include <Switch.h>
#include <XPLDirect.h>
#include <Stepper.h>

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
Encoder encBaro(A14, A15, NOT_USED, enc4Pulse);
Button btnHeading(52);
Encoder encHeading(50, 51, NOT_USED, enc4Pulse);
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
void handleAll()
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
  // Buttons
  btnBaro.handle();
  btnHeading.handle();
}

// check all steppers for target position reached
bool allInTarget()
{
  return stpSpeed.inTarget() &&
         stpRoll.inTarget() &&
         stpPitch.inTarget() &&
         stpAltitude.inTarget() &&
         stpBaro.inTarget() &&
         stpVario.inTarget() &&
         stpGyro.inTarget() &&
         stpHeading.inTarget() &&
         stpTurn.inTarget() &&
         stpBall.inTarget();
}

// move alle steppers to target position (blocking)
void allMoveTarget()
{
  while (!allInTarget())
  {
    // serve XPLDirect
    XP.xloop();
    // and move all steppers
    handleAll();
  }
}

// check if XPlane is running, otherwise timeout
bool checkRunningXP()
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
  XP.begin("Sixpack");

  // register DataRefs
  XP.registerDataRef(F("sim/time/zulu_time_sec"), XPL_READ, 1000, 1.0, &sim_time_sec);
  // attitude
  XP.registerDataRef(F("sim/cockpit2/gauges/indicators/roll_electric_deg_pilot"), XPL_READ, 50, 0.2, &roll_electric_deg_pilot);
  XP.registerDataRef(F("sim/cockpit2/gauges/indicators/pitch_electric_deg_pilot"), XPL_READ, 50, 0.1, &pitch_electric_deg_pilot);
  // airspeed
  XP.registerDataRef(F("sim/cockpit2/gauges/indicators/airspeed_kts_pilot"), XPL_READ, 50, 0.1, &airspeed_kts_pilot);
  // variometer
  XP.registerDataRef(F("sim/cockpit2/gauges/indicators/vvi_fpm_pilot"), XPL_READ, 50, 1.0, &vvi_fpm_pilot);
  // altimeter
  XP.registerDataRef(F("sim/cockpit2/gauges/indicators/altitude_ft_pilot"), XPL_READ, 50, 1.0, &altitude_ft_pilot);
  XP.registerDataRef(F("sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot"), XPL_READ, 50, 0.01, &barometer_setting_in_hg_pilot);
  // gyro
  XP.registerDataRef(F("sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot"), XPL_READ, 50, 0.2, &heading_electric_deg_mag_pilot);
  XP.registerDataRef(F("sim/cockpit2/autopilot/heading_dial_deg_mag_pilot"), XPL_READ, 50, 0.2, &heading_dial_deg_mag_pilot);
  // turn coordinator
  XP.registerDataRef(F("sim/cockpit2/gauges/indicators/turn_rate_roll_deg_pilot"), XPL_READ, 50, 0.2, &turn_rate_roll_deg_pilot);
  XP.registerDataRef(F("sim/cockpit2/gauges/indicators/slip_deg"), XPL_READ, 50, 0.01, &slip_deg);

  // register Commands
  barometer_down = XP.registerCommand(F("sim/instruments/barometer_down"));
  barometer_up = XP.registerCommand(F("sim/instruments/barometer_up"));
  barometer_std = XP.registerCommand(F("sim/instruments/barometer_std"));
  heading_down = XP.registerCommand(F("sim/autopilot/heading_down"));
  heading_up = XP.registerCommand(F("sim/autopilot/heading_up"));
  heading_sync = XP.registerCommand(F("sim/autopilot/heading_sync"));

  // speed indicator (310° = 160kt)
  stpSpeed.setFeedConst(185.8);
  stpSpeed.setBacklash(30);
  // attitude indicator
  stpRoll.setFeedConst(360.0);
  stpRoll.setBacklash(24);
  stpPitch.setFeedConst(1000.0);
  // altimeter (1000ft/turn)
  stpAltitude.setFeedConst(1000.0);
  stpAltitude.setBacklash(20);
  // baro 130 hPa/turn (3.8 inHg) // 50/16 gear
  stpBaro.setFeedConst(130.0 / INHG2HPA * 16.0 / 50.0);
  // baro 3.8 inHg/turn // 50/16 gear
  // stpBaro.setFeedConst(3.8 * 16.0 / 50.0);
  stpBall.setBacklash(12);
  // variometer
  stpVario.setFeedConst(4235.3);
  stpVario.setBacklash(12);
  // gyro (°)
  stpGyro.setModulo(4096);
  stpGyro.dirReverse(true);
  stpGyro.setBacklash(15);
  stpHeading.setModulo(4096);
  stpHeading.dirReverse(true);
  stpHeading.setBacklash(20);
  // turn coordinator
  stpTurn.setFeedConst(360.0);
  stpTurn.setBacklash(12);
  stpBall.setFeedConst(360.0);

  // init sequence -> move all indicators and calibrate horizon
  stpSpeed.setPosition(60.0);
  stpRoll.setIncrementsRelative(-1200); // move to block
  stpPitch.setIncrementsRelative(0);
  stpAltitude.setPosition(200.0);
  stpBaro.setPosition(hPa2baro(1000)); // 1000hPa
  stpVario.setPosition(500.0);
  stpGyro.setPosition(90.0);
  stpHeading.setPosition(-90.0);
  stpTurn.setPosition(30.0);
  stpBall.setPosition(15.0);
  allMoveTarget();
  stpSpeed.setPosition(0.0);
  stpRoll.setIncrementsRelative(590); // center
  stpPitch.setIncrementsRelative(0);
  stpAltitude.setPosition(0.0);
  stpBaro.setPosition(hPa2baro(1020)); // 1020hPa
  stpVario.setPosition(-500.0);
  stpGyro.setPosition(0.0);
  stpTurn.setPosition(-30.0);
  stpBall.setPosition(-15.0);
  allMoveTarget();
  stpPitch.setIncrementsRelative(-200); // move to block
  stpBaro.setPosition(hPa2baro(1013));
  stpVario.setPosition(0.0);
  stpHeading.setPosition(0.0);
  stpTurn.setPosition(0.0);
  stpBall.setPosition(0.0);
  allMoveTarget();
  stpPitch.setIncrementsRelative(90); // center
  allMoveTarget();

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
  // stpSpeed.setPositionLimit(0, 205 - 40);
  stpSpeed.setPositionLimit(0, (4 * 185.8) + 40); // allow 4 turns
  stpRoll.setPositionLimit(-45, 45);
  stpPitch.setPositionLimit(-17, 17);
  stpBaro.setPositionLimit(hPa2baro(970), hPa2baro(1050)); // 970 - 1050 hPa
  // stpBaro.setPositionLimit(28.6 - 29.92, 31.1 - 29.92); // 970 - 1050 hPa
  stpVario.setPositionLimit(-2000, 2000);
  stpTurn.setPositionLimit(-30.0, 30.0);
  stpBall.setPositionLimit(-16.0, 16.0);

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
  XP.xloop();

  // if XP running: set instruments
  if (checkRunningXP() && swEnable.on())
  {
    // set instrument values
    stpSpeed.setPosition(airspeed_kts_pilot - 40.0);
    stpRoll.setPosition(roll_electric_deg_pilot);
    stpPitch.setPosition(pitch_electric_deg_pilot);
    stpAltitude.setPosition(altitude_ft_pilot);
    stpBaro.setPosition(barometer_setting_in_hg_pilot - 29.92);
    stpVario.setPosition(vvi_fpm_pilot);
    stpGyro.setPosition(heading_electric_deg_mag_pilot);
    stpHeading.setPosition(heading_dial_deg_mag_pilot - heading_electric_deg_mag_pilot);
    stpTurn.setPosition(turn_rate_roll_deg_pilot);
    stpBall.setPosition(slip_deg * 4.0);

    // Handle commands max all 50ms
    long now = millis();
    if (now > time_last_command + COMMAND_TIMEOUT)
    {
      // barometer up/down
      if (encBaro.up())
      {
        XP.commandTrigger(barometer_up);
        time_last_command = now;
      }
      if (encBaro.down())
      {
        XP.commandTrigger(barometer_down);
        time_last_command = now;
      }
      if (btnBaro.pressed())
      {
        XP.commandTrigger(barometer_std);
        time_last_command = now;
      } // heading bug up/down
      if (encHeading.up())
      {
        XP.commandTrigger(heading_up);
        time_last_command = now;
      }
      if (encHeading.down())
      {
        XP.commandTrigger(heading_down);
        time_last_command = now;
      }
      if (btnHeading.pressed())
      {
        XP.commandTrigger(heading_sync);
        time_last_command = now;
      }
    }
  }
  else // XP NOT running: set all to zero
  {
    stpSpeed.setPosition(0);
    stpRoll.setPosition(0);
    stpPitch.setPosition(0);
    stpAltitude.setPosition(0);
    stpBaro.setPosition(0);
    stpVario.setPosition(0);
    stpGyro.setPosition(0);
    stpHeading.setPosition(0);
    stpTurn.setPosition(0);
    stpBall.setPosition(0);

    // use gyro encoder for zero adjustment, cycle through instruments
    if (btnHeading.pressed())
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
  handleAll();

  digitalWrite(LED_BUILTIN, swEnable.on());
}
