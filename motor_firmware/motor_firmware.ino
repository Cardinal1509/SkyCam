#include <Arduino.h>
#include <math.h>
#include "FastAccelStepper.h"

// ==========================================
// PIN DEFINITIONS (ESP32 CNC Shield + Servo)
// ==========================================
const int EN_PIN       = 14; 
const int STEP_PINS[4] = {18, 21, 23, 26}; // 0=X, 1=Y, 2=Z, 3=A
const int DIR_PINS[4]  = {19, 22, 25, 27}; // 0=X, 1=Y, 2=Z, 3=A
const int SERVO_PIN    = 13;               // SG90 Signal Pin

// ==========================================
// 14-BIT NATIVE ESP32 PWM (50Hz RC Servo)
// ==========================================
const int SERVO_CHANNEL = 4;
const int SERVO_FREQ    = 50;   // 50Hz (20,000 us period)
const int SERVO_RES     = 14;   // 14-bit resolution (0 - 16383 ticks)

void writeServoAngle(int angle) {
  angle = constrain(angle, 10, 170);
  // Map 0-180 deg to 500us - 2400us
  long pulseUs = map(angle, 0, 180, 500, 2400);
  // Period is 20,000 us. 14-bit max is 16384.
  uint32_t duty = (uint32_t)((pulseUs * 16384L) / 20000L);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(SERVO_PIN, duty);
#else
  ledcWrite(SERVO_CHANNEL, duty);
#endif
}

void initServo() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES);
#else
  ledcSetup(SERVO_CHANNEL, SERVO_FREQ, SERVO_RES);
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);
#endif

  // Boot verification wiggle: proves hardware and PWM are active
  writeServoAngle(60);
  delay(350);
  writeServoAngle(120);
  delay(350);
  writeServoAngle(90);
  delay(200);
}

// ==========================================
// GEOMETRY & LIMITS (in mm)
// ==========================================
const float FRAME_SIZE    = 760.0;
const float Z_TOP         = 685.8; 
const float CART_HALF_WID = 50.8;  

const float ANCHORS[4][3] = {
  { FRAME_SIZE,        0.0, Z_TOP }, // 0: Motor X
  {        0.0,        0.0, Z_TOP }, // 1: Motor Y
  { FRAME_SIZE, FRAME_SIZE, Z_TOP }, // 2: Motor Z
  {        0.0, FRAME_SIZE, Z_TOP }  // 3: Motor A
};

const float CART_OFFSETS[4][2] = {
  {  CART_HALF_WID, -CART_HALF_WID }, 
  { -CART_HALF_WID, -CART_HALF_WID }, 
  {  CART_HALF_WID,  CART_HALF_WID }, 
  { -CART_HALF_WID,  CART_HALF_WID }  
};

const float STEPS_PER_MM = 3200.0 / 72.0; 

const uint32_t MAX_CRUISE_SPEED_HZ = 3500;  
const int32_t  MAX_ACCELERATION    = 12000; 

const float HOME_X = 380.0;
const float HOME_Y = 380.0;
const float HOME_Z = 342.9;

float currentX = HOME_X;
float currentY = HOME_Y;
float currentZ = HOME_Z;
float homeCableLengths[4];

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper[4];

float calculateCableLength(int motorIdx, float cartX, float cartY, float cartZ) {
  float attachX = cartX + CART_OFFSETS[motorIdx][0];
  float attachY = cartY + CART_OFFSETS[motorIdx][1];
  float attachZ = cartZ;

  float dx = ANCHORS[motorIdx][0] - attachX;
  float dy = ANCHORS[motorIdx][1] - attachY;
  float dz = ANCHORS[motorIdx][2] - attachZ;
  return sqrt(dx * dx + dy * dy + dz * dz);
}

void moveToTarget(float targetX, float targetY, float targetZ) {
  targetX = constrain(targetX, 140.0, FRAME_SIZE - 140.0);
  targetY = constrain(targetY, 140.0, FRAME_SIZE - 140.0);
  targetZ = constrain(targetZ, 190.0, Z_TOP - 150.0);

  for (int i = 0; i < 4; i++) {
    if (abs(stepper[i]->targetPos() - stepper[i]->getCurrentPosition()) > 1000) {
      return;
    }
  }

  int32_t targetSteps[4];
  int32_t absStepsDelta[4];
  int32_t maxDelta = 0;

  for (int i = 0; i < 4; i++) {
    float lengthTarget = calculateCableLength(i, targetX, targetY, targetZ);
    float totalDeltaMm = lengthTarget - homeCableLengths[i];
    int32_t totalSteps = lround(totalDeltaMm * STEPS_PER_MM);

    if (i == 3) {
      targetSteps[i] = totalSteps;
    } else {
      targetSteps[i] = -totalSteps;
    }

    int32_t stepDiff = abs(targetSteps[i] - stepper[i]->getCurrentPosition());
    absStepsDelta[i] = stepDiff;
    if (stepDiff > maxDelta) maxDelta = stepDiff;
  }

  if (maxDelta == 0) return;

  for (int i = 0; i < 4; i++) {
    if (stepper[i] && absStepsDelta[i] > 0) {
      float ratio = (float)absStepsDelta[i] / (float)maxDelta;
      uint32_t scaledSpeed = (uint32_t)(MAX_CRUISE_SPEED_HZ * ratio);
      int32_t  scaledAccel = (int32_t)(MAX_ACCELERATION * ratio);
      if (scaledSpeed < 80)  scaledSpeed = 80;
      if (scaledAccel < 250) scaledAccel = 250;

      stepper[i]->setSpeedInHz(scaledSpeed);
      stepper[i]->setAcceleration(scaledAccel);
      stepper[i]->moveTo(targetSteps[i]);
    }
  }

  currentX = targetX;
  currentY = targetY;
  currentZ = targetZ;
}

void setup() {
  Serial.begin(115200);
  engine.init();

  initServo();

  for (int i = 0; i < 4; i++) {
    homeCableLengths[i] = calculateCableLength(i, HOME_X, HOME_Y, HOME_Z);
    stepper[i] = engine.stepperConnectToPin(STEP_PINS[i]);
    if (stepper[i]) {
      stepper[i]->setDirectionPin(DIR_PINS[i]);
      stepper[i]->setEnablePin(EN_PIN);
      stepper[i]->setAutoEnable(false);
      stepper[i]->enableOutputs();
      stepper[i]->setSpeedInHz(MAX_CRUISE_SPEED_HZ);
      stepper[i]->setAcceleration(MAX_ACCELERATION);
      stepper[i]->setCurrentPosition(0);
    }
  }

  Serial.println("[ESP32] Ready.");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    if (input == "HOME") {
      writeServoAngle(90);
      moveToTarget(HOME_X, HOME_Y, HOME_Z);
      Serial.println("[ACK] HOME");
      return;
    }

    int aIdx = input.indexOf(",A");
    if (aIdx > 0) {
      int servoDeg = input.substring(aIdx + 2).toInt();
      writeServoAngle(servoDeg);
      Serial.printf("[ACK A=%d]\n", servoDeg);
      input = input.substring(0, aIdx);
    }

    int xIdx = input.indexOf("X");
    int yIdx = input.indexOf(",Y");
    int zIdx = input.indexOf(",Z");

    if (xIdx == 0 && yIdx > 0 && zIdx > 0) {
      float dxMm = input.substring(1, yIdx).toInt() * 0.1f;
      float dyMm = input.substring(yIdx + 2, zIdx).toInt() * 0.1f;
      float dzMm = input.substring(zIdx + 2).toInt() * 0.1f;

      if (abs(dxMm) > 0.01f || abs(dyMm) > 0.01f || abs(dzMm) > 0.01f) {
        dxMm = constrain(dxMm, -12.0f, 12.0f);
        dyMm = constrain(dyMm, -12.0f, 12.0f);
        dzMm = constrain(dzMm, -6.0f, 6.0f);
        moveToTarget(currentX + dxMm, currentY + dyMm, currentZ + dzMm);
      }
    }
  }
}