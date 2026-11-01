#include "FastAccelStepper.h"

// Pin Definitions matching your CNC Shield V3 top-left header
const int EN_PIN = 14;                     // Row 1 Blue (Active LOW Enable)
const int STEP_PINS[4] = {18, 21, 23, 26}; // Rows 2-5 Blue (X, Y, Z, A Step)
const int DIR_PINS[4]  = {19, 22, 25, 27}; // Rows 2-5 White (X, Y, Z, A Dir)

// Kinematic & Speed Constants
// 36T GT2 Pulley Pitch Circumference = 36 teeth * 2mm pitch = 72.0 mm/rev
// Microstepping = 200 full steps * 16 = 3,200 microsteps/rev (44.44 steps/mm)
const uint32_t MAX_SPEED_HZ     = 15500; // 15,500 steps/s = 4.84 rev/s (290.6 RPM) = 0.35 m/s cable velocity
const int32_t  ACCELERATION     = 35000; // 35,000 steps/s^2 -> Ramps 0 to 15.5 kHz in 0.44 seconds
const int32_t  TEST_TRAVEL_STEPS = 9600; // 9,600 microsteps = 3 full revolutions (~216 mm of cable)

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper[4];

void setup() {
  Serial.begin(115200);
  engine.init();

  for (int i = 0; i < 4; i++) {
    stepper[i] = engine.stepperConnectToPin(STEP_PINS[i]);
    if (stepper[i]) {
      stepper[i]->setDirectionPin(DIR_PINS[i]);
      stepper[i]->setEnablePin(EN_PIN);
      stepper[i]->setAutoEnable(false); // Keep holding torque active continuously
      stepper[i]->enableOutputs();

      stepper[i]->setSpeedInHz(MAX_SPEED_HZ);
      stepper[i]->setAcceleration(ACCELERATION);
    }
  }
}

void loop() {
  // 1. Accelerate all 4 axes to +3 revolutions (CW), cruise at 15.5 kHz, and decelerate to stop
  for (int i = 0; i < 4; i++) {
    if (stepper[i]) stepper[i]->moveTo(TEST_TRAVEL_STEPS);
  }
  while (stepper[0]->isRunning() || stepper[1]->isRunning() || 
         stepper[2]->isRunning() || stepper[3]->isRunning()) {
    delay(5); // Hardware peripheral drives pulses in the background
  }
  delay(400); // Hold position for 0.4s

  // 2. Accelerate all 4 axes back to 0 (CCW), cruise at 15.5 kHz, and decelerate to stop
  for (int i = 0; i < 4; i++) {
    if (stepper[i]) stepper[i]->moveTo(0);
  }
  while (stepper[0]->isRunning() || stepper[1]->isRunning() || 
         stepper[2]->isRunning() || stepper[3]->isRunning()) {
    delay(5);
  }
  delay(400);
}