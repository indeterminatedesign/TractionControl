#include <Arduino.h>
#include <IBusBM.h>
#include <ESP32Servo.h>

IBusBM IBus;
const uint16_t steeringMin = 1000;
const uint16_t steeringMax = 2000;
uint16_t steeringCenter = 1500;
bool steeringCalibrated = false;

const uint16_t throttleMin = 1000;
const uint16_t throttleMax = 2000;
const uint16_t throttleNeutral = 1500;

const uint16_t standardChannelMin = 1000;
const uint16_t standardChannelMax = 2000;
const uint16_t standardChannelNeutral = 1500;

const float_t kP = 5000;

#define THROTTLE_OUT_PIN 14
#define STEERING_OUT_PIN 27

Servo servoThrottle;
Servo servoSteering;

// create variables to hold a local copies of the channel inputs
uint16_t throttleIn = throttleNeutral;
uint16_t steeringIn = steeringCenter;

uint16_t throttleOut = throttleMin;
uint16_t SteeringOut = steeringCenter;

#define IBUS_RX2_PIN 16
#define IBUS_TX2_PIN 32 // Not used because I'm not sending back sensor data to the RX.  But must be defined...

const int16_t frontPin = 2;
const int16_t rearPin = 15;
boolean frontPulseFlag = false;
int32_t frontCurrentMicros = 0;
int32_t frontPreviousMicros = 0;
boolean rearPulseFlag = false;
int32_t rearCurrentMicros = 0;
int32_t rearPreviousMicros = 0;

float_t frontRPM = 0;
float_t frontPreviousRPM = 0;
float_t rearRPM = 0;
float_t rearPreviousRPM = 0;

float_t targetSlipAngle = .1;

float_t steeringRate = 1000; // Steering rate based on speed

int32_t previousRPMCalcMicros = 0;
const int32_t rpmCalcInterval = 5000; // Calculate the rpm every .005 seconds

int32_t previousTractionControlMicros = 0;
const int32_t tractionControlInterval = 10000; // Run the traction controller every .01 seconds

float EMA_function(float alpha, float latest, float stored);
float calcRPM(int32_t currentMicros, int32_t previousMicros, float_t &currentRPM, float_t &previousRPM);
void processReceiverInput();
void processRPM();
void processTractionControl ();

// Add a pulse every time the hall sensor changes state
void IRAM_ATTR frontPulse()
{
  frontPreviousMicros = frontCurrentMicros;
  frontCurrentMicros = micros();
  frontPulseFlag = true;
  // Serial.println("Front Pulse");
}
void IRAM_ATTR rearPulse()
{
  rearPreviousMicros = rearCurrentMicros;
  rearCurrentMicros = micros();
  rearPulseFlag = true;
  // Serial.println("Rear Pulse");
}

void setup()
{
  pinMode(frontPin, INPUT_PULLUP);
  pinMode(rearPin, INPUT_PULLUP);
  Serial.begin(115200);

  // Ibus process for reciever data
  IBus.begin(Serial2, IBUSBM_NOTIMER, IBUS_RX2_PIN, IBUS_TX2_PIN);

  attachInterrupt(frontPin, frontPulse, FALLING);
  attachInterrupt(rearPin, rearPulse, FALLING);

  // Attach Servos
  servoThrottle.attach(THROTTLE_OUT_PIN);
  servoSteering.attach(STEERING_OUT_PIN);
}

void loop()
{
  processReceiverInput();
  
  processRPM();
  processTractionControl ();
}

/*************************************************************************************
  Process Reciever
***********************************************************************************/
void processReceiverInput()
{
  // Loop thru and process Ibus
  IBus.loop();
  throttleIn = IBus.readChannel(1); // get latest value for servo channel 3
  steeringIn = IBus.readChannel(0); // get latest value for the servo channel 1

  if (!steeringCalibrated && steeringIn > 1400)
  {
    // Set the initial centered steering position
    steeringCenter = steeringIn;
    steeringCalibrated = true;
  }

  // Setpoint used for traction control and braking
  // Channel 6
  targetSlipAngle = map(IBus.readChannel(5), standardChannelMin, standardChannelMax, 0, 20) / 100.00;
  Serial.print("Slip Angle Target ");
  Serial.println(targetSlipAngle);

  /*
  for (int i = 0; i < 5; i++)
  {
    Serial.print("Channel ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(IBus.readChannel(i));
  }
*/
  /*
  Serial.print("From RX SteeringCutRate: ");
  Serial.println(SteeringCorrection);
  Serial.print("From RX ThrottleCutRate: ");
  Serial.println(ThrottleCutRate);
  Serial.print("From RX ThrottleIn: ");
  Serial.println(ThrottleIn);
  Serial.print("From RX SteeringIn: ");
  Serial.println(SteeringIn);
  */
}

/*************************************************************************************
  Process RPM
***********************************************************************************/
void processRPM()
{
  if (micros() - previousRPMCalcMicros > rpmCalcInterval)
  {
    if (frontPulseFlag)
    {
      calcRPM(frontCurrentMicros, frontPreviousMicros, frontRPM, frontPreviousRPM);
      frontPulseFlag = false;
    }
    if (rearPulseFlag)
    {
      calcRPM(rearCurrentMicros, rearPreviousMicros, rearRPM, rearPreviousRPM);
      rearPulseFlag = false;
    }
    previousRPMCalcMicros = micros();
  }
}

/*************************************************************************************
  Process Traction Control
***********************************************************************************/
void processTractionControl ()
{
  if (micros() - previousTractionControlMicros > tractionControlInterval)
  {
// Serial.println(frontRPM);
  float_t currentPercentSlip = 0;
  if (frontRPM > 0)
  {
    currentPercentSlip = (rearRPM - frontRPM) / frontRPM;     // will be positive if rear is spinning faster
    currentPercentSlip = constrain(currentPercentSlip, 0, 1); // Current percent slip only positive for this, and cannot exceed 100% or 1
                                                              // Serial.print("Current Slip Angle ");
    // Serial.println(currentPercentSlip);
  }

  if (targetSlipAngle < currentPercentSlip)
  {
    float_t slipError = currentPercentSlip - targetSlipAngle;
    int32_t correction = kP * slipError;
    correction = constrain(correction, 0, 500); // Correction cannot be greater than 500 or less than 0
    throttleOut = throttleIn - correction;
    throttleOut = constrain(throttleOut, throttleNeutral, throttleMax); // Don't allow the traction control to turn on braking or exceed the boundaries
    Serial.print("Throttle In ");
    Serial.println(throttleIn);
    Serial.print("Throttle Out ");
    Serial.println(throttleOut);
  }
  else
  {
    throttleOut = throttleIn;
  }

  servoThrottle.writeMicroseconds(throttleOut);
  }
}

float calcRPM(int32_t currentMicros, int32_t previousMicros, float_t &currentRPM, float_t &previousRPM)
{
  int32_t dt = currentMicros - previousMicros;
  int32_t dtSinceLastPulse = micros() - currentMicros; // Check to see how long it's been since a pulse came in
  if (dtSinceLastPulse < 250000 && dt > 0)
  {
    previousRPM = currentRPM;
    currentRPM = (60 / (dt / 1000000.00)) / 4; // 4 magnets

    currentRPM = EMA_function(0.4, currentRPM, previousRPM);
  }
  else
  {
    currentRPM = 0;
  }
  return currentRPM;
}

float EMA_function(float alpha, float latest, float stored)
{
  return (alpha * latest) + ((1 - alpha) * stored);
}