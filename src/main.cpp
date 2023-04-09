#include <Arduino.h>
#include <XPLPro.h>
#include <StepperMC.h>

// Stepper motors
StepperMC stpSpeed(22, 23, 24, 25);
StepperMC stpRoll(26, 27, 28, 29);
StepperMC stpPitch(30, 31, 32, 33);
StepperMC stpAltitude(34, 35, 36, 37);
StepperMC stpBaro(A8, A9, A10, A11);
StepperMC stpVario(38, 39, 40, 41);
StepperMC stpGyro(42, 43, 44, 45);
StepperMC stpHeading(46, 47, 48, 49);
StepperMC stpTurn(2, 3, 4, 5);
StepperMC stpBall(18, 19, 20, 21);

// Input devices
Encoder encBaro(A14, A15, A13, enc4Pulse);
Encoder encHeading(50, 51, 52, enc4Pulse);
Switch swEnable(A12);

Timer tmrIndicate(50);
Timer tmrMain(1000);

// Altimeter calculation in_hg -> hPa
#define INHG2HPA 33.863886666667
#define hPa2baro(p) (p - 1013) / INHG2HPA

// Stepper speed
#define STEP_SPEED 1200
#define STEP_ACC 4000

// datarefs
int dref_sim_time_sec;
int dref_roll_electric_deg_pilot;
int dref_pitch_electric_deg_pilot;
int dref_airspeed_kts_pilot;
int dref_vvi_fpm_pilot;
int dref_altitude_ft_pilot;
int dref_barometer_setting_in_hg_pilot;
int dref_heading_electric_deg_mag_pilot;
int dref_heading_dial_deg_mag_pilot;
int dref_turn_rate_roll_deg_pilot;
int dref_slip_deg;

float heading_electric_deg_mag_pilot;

// running check
bool indicationActive;
bool xp_running;
unsigned long time_last_running;
#define XP_TIMEOUT 30

// adjustment
int selectedStepper;

// handle all steppers
void handleAllSteppers()
{
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
}

// check all steppers for target position reached
bool allSteppersInTarget()
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
void moveAllSteppersToTarget()
{
  while (!allSteppersInTarget())
  {
    // serve XPLDirect
    XP.xloop();
    // and move all steppers
    handleAllSteppers();
  }
}

// true if XPlane is running, false if timeout
bool checkRunningXP()
{
  // timeout reached?
  return (millis() < time_last_running + XP_TIMEOUT * 1000l);
}

// register all datarefs and commands
void xpInit()
{
  // attitude
  dref_roll_electric_deg_pilot = XP.registerDataRef(F("sim/cockpit2/gauges/indicators/roll_electric_deg_pilot"));
  XP.requestUpdates(dref_roll_electric_deg_pilot, 50, 0.2);
  dref_pitch_electric_deg_pilot = XP.registerDataRef(F("sim/cockpit2/gauges/indicators/pitch_electric_deg_pilot"));
  XP.requestUpdates(dref_pitch_electric_deg_pilot, 50, 0.1);
  // airspeed
  dref_airspeed_kts_pilot = XP.registerDataRef(F("sim/cockpit2/gauges/indicators/airspeed_kts_pilot"));
  XP.requestUpdates(dref_airspeed_kts_pilot, 50, 0.1);
  // variometer
  dref_vvi_fpm_pilot = XP.registerDataRef(F("sim/cockpit2/gauges/indicators/vvi_fpm_pilot"));
  XP.requestUpdates(dref_vvi_fpm_pilot, 50, 1.0);
  // altimeter
  dref_altitude_ft_pilot = XP.registerDataRef(F("sim/cockpit2/gauges/indicators/altitude_ft_pilot"));
  XP.requestUpdates(dref_altitude_ft_pilot, 50, 1.0);
  dref_barometer_setting_in_hg_pilot = XP.registerDataRef(F("sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot"));
  XP.requestUpdates(dref_barometer_setting_in_hg_pilot, 50, 0.01);
  // gyro
  dref_heading_electric_deg_mag_pilot = XP.registerDataRef(F("sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot"));
  XP.requestUpdates(dref_heading_electric_deg_mag_pilot, 50, 0.2);
  dref_heading_dial_deg_mag_pilot = XP.registerDataRef(F("sim/cockpit2/autopilot/heading_dial_deg_mag_pilot"));
  XP.requestUpdates(dref_heading_dial_deg_mag_pilot, 50, 0.2);
  // turn coordinator
  dref_turn_rate_roll_deg_pilot = XP.registerDataRef(F("sim/cockpit2/gauges/indicators/turn_rate_roll_deg_pilot"));
  XP.requestUpdates(dref_turn_rate_roll_deg_pilot, 50, 0.2);
  dref_slip_deg = XP.registerDataRef(F("sim/cockpit2/gauges/indicators/slip_deg"));
  XP.requestUpdates(dref_slip_deg, 50, 0.01);
  // register Commands with encoders
  encBaro.setCommand(F("sim/instruments/barometer_up"), F("sim/instruments/barometer_down"), F("sim/instruments/barometer_std"));
  encHeading.setCommand(F("sim/autopilot/heading_up"), F("sim/autopilot/heading_down"), F("sim/autopilot/heading_sync"));
}

// stop triggered, reset steppers
void xpStop()
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
  moveAllSteppersToTarget();
  indicationActive = false;
}

// handle incoming datarefs
void xpUpdate(int handle)
{
  time_last_running = millis();
  // process datarefs only when indication active
  if (!indicationActive)
  {
    return;
  }
  if (handle == dref_airspeed_kts_pilot)
  {
    stpSpeed.setPosition(XP.datarefReadFloat() - 40.0);
  }
  if (handle == dref_roll_electric_deg_pilot)
  {
    stpRoll.setPosition(XP.datarefReadFloat());
  }
  if (handle == dref_pitch_electric_deg_pilot)
  {
    stpPitch.setPosition(XP.datarefReadFloat());
  }
  if (handle == dref_altitude_ft_pilot)
  {
    stpAltitude.setPosition(XP.datarefReadFloat());
  }
  if (handle == dref_barometer_setting_in_hg_pilot)
  {
    stpBaro.setPosition(XP.datarefReadFloat() - 29.92);
  }
  if (handle == dref_vvi_fpm_pilot)
  {
    stpVario.setPosition(XP.datarefReadFloat());
  }
  if (handle == dref_heading_electric_deg_mag_pilot)
  {
    // heading is also needed for bug position
    heading_electric_deg_mag_pilot = XP.datarefReadFloat();
    stpGyro.setPosition(heading_electric_deg_mag_pilot);
  }
  if (handle == dref_heading_dial_deg_mag_pilot)
  {
    stpHeading.setPosition(XP.datarefReadFloat() - heading_electric_deg_mag_pilot);
  }
  if (handle == dref_turn_rate_roll_deg_pilot)
  {
    stpTurn.setPosition(XP.datarefReadFloat());
  }
  if (handle == dref_slip_deg)
  {
    stpBall.setPosition(XP.datarefReadFloat() * 4.0);
  }
}

// initialization
void setup()
{
  // initialize the interface
  Serial.begin(XPL_BAUDRATE);
  XP.begin("Sixpack", &xpInit, &xpStop, &xpUpdate);

  // speed indicator (310° = 160kt)
  stpSpeed.setFeedConst(185.8);
  stpSpeed.setBacklash(0);
  stpSpeed.setSpeed(STEP_SPEED, STEP_ACC);
  // attitude indicator
  stpRoll.setFeedConst(360.0);
  stpRoll.setBacklash(24);
  stpRoll.setSpeed(STEP_SPEED, STEP_ACC);
  stpPitch.setFeedConst(1000.0);
  stpPitch.setSpeed(STEP_SPEED, STEP_ACC);
  // altimeter (1000ft/turn)
  stpAltitude.setFeedConst(1000.0);
  stpAltitude.setBacklash(0);
  stpAltitude.setSpeed(STEP_SPEED, STEP_ACC);
  // baro 130 hPa/turn (3.8 inHg) // 50/16 gear
  stpBaro.setGearRatio(16, 50);
  stpBaro.setFeedConst(130.0 / INHG2HPA);
  stpBaro.setSpeed(STEP_SPEED, STEP_ACC);
  // baro 3.8 inHg/turn // 50/16 gear
  // stpBaro.setFeedConst(3.8 * 16.0 / 50.0);
  // variometer
  stpVario.setFeedConst(4235.3);
  stpVario.setBacklash(12);
  stpVario.setSpeed(STEP_SPEED, STEP_ACC);
  // gyro (°)
  stpGyro.setModulo();
  stpGyro.reverseDir(true);
  stpGyro.setBacklash(15);
  stpGyro.setSpeed(STEP_SPEED, STEP_ACC);
  stpHeading.setModulo();
  stpHeading.reverseDir(true);
  stpHeading.setBacklash(20);
  stpHeading.setSpeed(STEP_SPEED, STEP_ACC);
  // turn coordinator
  stpTurn.setBacklash(12);
  stpTurn.setSpeed(STEP_SPEED, STEP_ACC);
  stpBall.setBacklash(12);
  stpBall.setSpeed(STEP_SPEED, STEP_ACC);

#if 0
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
#endif

  // reset all steppers to zero
  stpSpeed.setZero();
  stpRoll.setZero();
  stpPitch.setZero();
  stpAltitude.setZero();
  stpBaro.setZero();
  stpVario.setZero();
  stpGyro.setZero();
  stpHeading.setZero();
  stpTurn.setZero();
  stpBall.setZero();

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
  time_last_running = 0;

  // adjustment
  selectedStepper = 0;

  // LED
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  // handle interface
  XP.xloop();

  // handle all steppers and encoders
  handleAllSteppers();
  // Encoders
  encBaro.handle();
  encHeading.handle();
  // Switch
  swEnable.handle();

  // trigger 
  if ((swEnable.off() && indicationActive) || !checkRunningXP())
  {
    xpStop();
  }
  // if XP running: enable indication via callback
  if (XP.connectionStatus() && checkRunningXP() && swEnable.on())
  {
    indicationActive = true;
  }

  // show status on LED
  digitalWrite(LED_BUILTIN, indicationActive);

  // handle encoders
  if (indicationActive)
  {
    // handle commands with XP
    encBaro.processCommand();
    encHeading.processCommand();
  }
  else 
  {
    // use encoders for zero adjustment, cycle through instruments on push
    if (encHeading.pressed() || encBaro.pressed())
    {
      selectedStepper = (selectedStepper + 1) % 12;
    }
    int32_t steps = 0;
    if (encHeading.up() || encBaro.up())
    {
      steps = 8;
    }
    if (encHeading.down() || encBaro.down())
    {
      steps = -8;
    }
    switch (selectedStepper)
    {
    case 1:
      stpSpeed.adjustZero(steps);
      break;
    case 2:
      stpRoll.adjustZero(steps);
      break;
    case 3:
      stpPitch.adjustZero(steps / 2);
      break;
    case 4:
      stpAltitude.adjustZero(steps * 51); // 100ft
      break;
    case 5:
      stpAltitude.adjustZero(steps);
      break;
    case 6:
      stpBaro.adjustZero(steps * 3.125);
      break;
    case 7:
      stpVario.adjustZero(steps);
      break;
    case 8:
      stpGyro.adjustZero(steps);
      break;
    case 9:
      stpHeading.adjustZero(steps);
      break;
    case 10:
      stpTurn.adjustZero(steps);
      break;
    case 11:
      stpBall.adjustZero(steps);
      break;
    default:;
    }
  }

  // measure runtime and send to XP
  if (tmrMain.elapsed())
  {
    char tmp[16];
    sprintf(tmp, " %ld Cycles/s", tmrMain.count());
    XP.sendDebugMessage(tmp);
  }
}
