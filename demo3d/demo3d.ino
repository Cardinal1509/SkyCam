#include <Arduino.h>
#include <math.h>
#include "FastAccelStepper.h"

// ==========================================
// PIN DEFINITIONS (Physical Jumper Wiring)
// ==========================================
const int EN_PIN       = 14; 
const int STEP_PINS[4] = {18, 21, 23, 26}; // Indices: 0=X, 1=Y, 2=Z, 3=A
const int DIR_PINS[4]  = {19, 22, 25, 27}; // Indices: 0=X, 1=Y, 2=Z, 3=A

// ==========================================
// GEOMETRY & CARRIAGE OFFSETS (in mm)
// ==========================================
const float FRAME_SIZE    = 760.0;
const float Z_TOP         = 685.8; // 27 inches from ground
const float CART_HALF_WID = 50.8;  // Half of 4-inch (101.6 mm) square cart

// Exact Corner Coordinates matching your labeled photo:
// Bottom-Left  = Y (0, 0)
// Bottom-Right = X (760, 0)
// Top-Left     = A (0, 760)
// Top-Right    = Z (760, 760)
const float ANCHORS[4][3] = {
  { FRAME_SIZE,        0.0, Z_TOP }, // Index 0: Motor X (Bottom-Right)
  {        0.0,        0.0, Z_TOP }, // Index 1: Motor Y (Bottom-Left)
  { FRAME_SIZE, FRAME_SIZE, Z_TOP }, // Index 2: Motor Z (Top-Right)
  {        0.0, FRAME_SIZE, Z_TOP }  // Index 3: Motor A (Top-Left)
};

// Matching corner offsets on the 4-inch (101.6 mm) carriage plate
const float CART_OFFSETS[4][2] = {
  {  CART_HALF_WID, -CART_HALF_WID }, // Index 0: X corner (Bottom-Right: +50.8, -50.8)
  { -CART_HALF_WID, -CART_HALF_WID }, // Index 1: Y corner (Bottom-Left:  -50.8, -50.8)
  {  CART_HALF_WID,  CART_HALF_WID }, // Index 2: Z corner (Top-Right:    +50.8, +50.8)
  { -CART_HALF_WID,  CART_HALF_WID }  // Index 3: A corner (Top-Left:     -50.8, +50.8)
};

// 36T GT2 Pulley (72.0 mm/rev), 3,200 microsteps/rev
const float STEPS_PER_MM = 3200.0 / 72.0; // ~44.4444 steps/mm

// Motion Profile Limits
const uint32_t MAX_CRUISE_SPEED_HZ = 4000;  
const int32_t  MAX_ACCELERATION    = 12000; 

// Carriage Home / Center Coordinates
float currentX = 380.0;
float currentY = 380.0;
float currentZ = 342.9; // Halfway down (13.5 inches off ground)

int32_t currentSteps[4] = {0, 0, 0, 0};

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper[4];

// Calculate 3D distance from frame corner bearing to the corresponding cart corner
float calculateCableLength(int motorIdx, float cartX, float cartY, float cartZ) {
  float attachX = cartX + CART_OFFSETS[motorIdx][0];
  float attachY = cartY + CART_OFFSETS[motorIdx][1];
  float attachZ = cartZ;

  float dx = ANCHORS[motorIdx][0] - attachX;
  float dy = ANCHORS[motorIdx][1] - attachY;
  float dz = ANCHORS[motorIdx][2] - attachZ;
  return sqrt(dx * dx + dy * dy + dz * dz);
}

// Synchronized 4-axis move with proportional speed/acceleration scaling
void moveToPosition(float targetX, float targetY, float targetZ) {
  int32_t targetSteps[4];
  int32_t absStepsDelta[4];
  int32_t maxDelta = 0;

  for (int i = 0; i < 4; i++) {
    float lengthInitial = calculateCableLength(i, currentX, currentY, currentZ);
    float lengthTarget  = calculateCableLength(i, targetX, targetY, targetZ);
    float deltaMm = lengthTarget - lengthInitial;

    int32_t stepsDelta = lround(deltaMm * STEPS_PER_MM);
    absStepsDelta[i] = abs(stepsDelta);
    if (absStepsDelta[i] > maxDelta) {
      maxDelta = absStepsDelta[i];
    }

    // Winding Orientation Logic:
    // deltaMm > 0 means cable needs to LOOSEN (pay out line).
    // Motors 0, 1, 2 (X, Y, Z) loosen CCW (- steps).
    // Motor 3 (A) loosens CW (+ steps).
    if (i == 3) {
      targetSteps[i] = currentSteps[i] + stepsDelta; // A: CW (+) loosens
    } else {
      targetSteps[i] = currentSteps[i] - stepsDelta; // X, Y, Z: CCW (-) loosens
    }
  }

  if (maxDelta == 0) return;

  // Scale speed and acceleration per axis so all 4 motors finish simultaneously
  for (int i = 0; i < 4; i++) {
    if (stepper[i] && absStepsDelta[i] > 0) {
      float ratio = (float)absStepsDelta[i] / (float)maxDelta;
      uint32_t scaledSpeed = (uint32_t)(MAX_CRUISE_SPEED_HZ * ratio);
      int32_t  scaledAccel = (int32_t)(MAX_ACCELERATION * ratio);

      if (scaledSpeed < 50)  scaledSpeed = 50;
      if (scaledAccel < 100) scaledAccel = 100;

      stepper[i]->setSpeedInHz(scaledSpeed);
      stepper[i]->setAcceleration(scaledAccel);
      stepper[i]->moveTo(targetSteps[i]);
    }
  }

  // Wait synchronously until all 4 motors complete the move
  while (stepper[0]->isRunning() || stepper[1]->isRunning() || 
         stepper[2]->isRunning() || stepper[3]->isRunning()) {
    delay(5);
  }

  // Update position state
  for (int i = 0; i < 4; i++) {
    currentSteps[i] = targetSteps[i];
  }
  currentX = targetX;
  currentY = targetY;
  currentZ = targetZ;
}

void setup() {
  Serial.begin(115200);
  engine.init();

  for (int i = 0; i < 4; i++) {
    stepper[i] = engine.stepperConnectToPin(STEP_PINS[i]);
    if (stepper[i]) {
      stepper[i]->setDirectionPin(DIR_PINS[i]);
      stepper[i]->setEnablePin(EN_PIN);
      stepper[i]->setAutoEnable(false); // Constant holding torque
      stepper[i]->enableOutputs();

      stepper[i]->setSpeedInHz(MAX_CRUISE_SPEED_HZ);
      stepper[i]->setAcceleration(MAX_ACCELERATION);
      stepper[i]->setCurrentPosition(0);
    }
  }

  Serial.println("[CDPR] Initialized with exact photo coordinates.");
  Serial.println("[CDPR] Starting 5-second countdown...");
  delay(5000);
}

void loop() {
  // 1. Move Right (+80mm X, toward X and Z)
  Serial.println("Moving Right (+X)");
  moveToPosition(460.0, 380.0, 342.9);
  delay(1000);

  // 2. Move Up/Forward (+80mm Y, toward A and Z)
  Serial.println("Moving Up (+Y)");
  moveToPosition(460.0, 460.0, 342.9);
  delay(1000);

  // 3. Move Left (-160mm X, toward A and Y)
  Serial.println("Moving Left (-X)");
  moveToPosition(300.0, 460.0, 342.9);
  delay(1000);

  // 4. Move Down/Back (-160mm Y, toward Y and X)
  Serial.println("Moving Down (-Y)");
  moveToPosition(300.0, 300.0, 342.9);
  delay(1000);

  // 5. Return to Center
  Serial.println("Returning to Center");
  moveToPosition(380.0, 380.0, 342.9);
  delay(1500);

  // 6. Test pure Z-Axis Lift (+100mm straight UP, then back to center)
  Serial.println("Testing Z Lift (+100mm UP)");
  moveToPosition(380.0, 380.0, 442.9);
  delay(1500);

  Serial.println("Returning Z to Center");
  moveToPosition(380.0, 380.0, 342.9);
  delay(3000);
}