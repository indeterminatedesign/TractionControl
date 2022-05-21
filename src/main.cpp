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

int32_t kP = 5000;

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

#define FRONT_PIN 26
#define REAR_PIN 33

boolean frontPulseFlag = false;
uint32_t frontDT = 0;
uint32_t frontStartMicros = 0;
boolean rearPulseFlag = false;
uint32_t rearDT = 0;
uint32_t rearStartMicros = 0;
uint16_t frontPulseCount = 0;
uint16_t rearPulseCount = 0;

float_t frontRPM = 0;
float_t rearRPM = 0;

float_t targetSlipAngle = .1;

float_t steeringRate = 1000; // Steering rate based on speed

int32_t previousTractionControlMicros = 0;
const int32_t tractionControlInterval = 10000; // Run the traction controller every .01 seconds

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux1 = portMUX_INITIALIZER_UNLOCKED;

float EMA_function(float alpha, float latest, float stored);
float_t calcRPM(uint32_t &startMicros, float_t currentRPM, boolean &flag, uint16_t &pulseCount, uint16_t pin);
void processReceiverInput();
void processRPM();
void processTractionControl();


void setup()
{
  pinMode(FRONT_PIN, INPUT_PULLUP);
  pinMode(REAR_PIN, INPUT_PULLUP);

  Serial.begin(115200);

  // Ibus process for reciever data
  IBus.begin(Serial2, IBUSBM_NOTIMER, IBUS_RX2_PIN, IBUS_TX2_PIN);

  // Attach Servos
  servoThrottle.attach(THROTTLE_OUT_PIN);
  servoSteering.attach(STEERING_OUT_PIN);
}

void loop()
{
  processReceiverInput();
  processRPM();
  processTractionControl();
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

  // kp channel 5
  kP = map(IBus.readChannel(4), standardChannelMin, standardChannelMax, 1000, 20000);
  // Channel 6
  targetSlipAngle = map(IBus.readChannel(5), standardChannelMin, standardChannelMax, 0, 20) / 100.00;
  // Serial.print("Slip Angle Target ");
  // Serial.println(targetSlipAngle);

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
uint32_t previousRPMMicros = 0;
const int32_t rpmInterval = 1000;

// set number of hall trips for RPM reading (higher improves accuracy)
uint16_t hall_thresh = 2;
void processRPM()

{
  uint32_t dt = micros() - previousRPMMicros;
  if (dt > rpmInterval)
  {
    frontRPM = calcRPM(frontStartMicros, frontRPM, frontPulseFlag, frontPulseCount, FRONT_PIN);
    rearRPM = calcRPM(rearStartMicros, rearRPM, rearPulseFlag, rearPulseCount, REAR_PIN);
  }
  /*
  Serial.print(frontRPM);
  Serial.print(",");
  Serial.println(rearRPM);
  */
}

/*************************************************************************************
  Process Traction Control
***********************************************************************************/
void processTractionControl()
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
      /*
      Serial.print("Throttle In ");
      Serial.println(throttleIn);
      Serial.print("Throttle Out ");
      Serial.println(throttleOut);
      */
    }
    else
    {
      throttleOut = throttleIn;
    }

    servoThrottle.writeMicroseconds(throttleOut);
  }
}

float_t calcRPM(uint32_t &startMicros, float_t currentRPM, boolean &flag, uint16_t &pulseCount, uint16_t pin)
{
  
  int32_t dtSinceLastPulse = micros() - startMicros; // Check to see how long it's been since a pulse came in

  dtSinceLastPulse = dtSinceLastPulse/ (pulseCount +1);


  if (dtSinceLastPulse >= 1000000)
  {
    pulseCount = 0;
    flag = false;
    startMicros = micros();
    Serial.println("0 RPM)");
    return 0;
  }

  if (digitalRead(pin) == 0)
  {
    if (flag == false)
    {
      flag = true;
      pulseCount++;
     //Serial.println("Pulse ++)");
    }
  }
  else
  {
    flag = false;
  }

  if (pulseCount >= hall_thresh)
  {
    // print information about Time and RPM
    uint32_t end_time = micros();
    float_t time_passed = ((end_time - startMicros) / 1000000.0);
    currentRPM = ((pulseCount / time_passed) * 60.0) / 2.0;
    //Serial.println(currentRPM);
    startMicros = micros();
    pulseCount = 0;
  }

  return currentRPM;
}

float EMA_function(float alpha, float latest, float stored)
{
  return (alpha * latest) + ((1 - alpha) * stored);
}