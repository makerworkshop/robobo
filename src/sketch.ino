// Copyright (c) 2014-2015 José Carlos Nieto, https://menteslibres.net/xiam
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <Servo.h>

#include <MotorDriver.h>
#include <Ping.h>

// #define DEBUG

// Debug PIN
#define ROBOBO_DEBUG_PIN              13

// Minimum safe distance to obstacle (mm).
#define ROBOBO_MINIMUM_DISTANCE_TO_OBSTACLE   800

#define ROBOBO_PING_SENSOR_PIN        4
#define ROBOBO_SERVO_PIN              2

#define ROBOBO_PING_FOV               40
#define ROBOBO_SERVO_ANGLE_NEUTRAL    90

#define ROBOBO_SERVO_FULL_TURN_SPEED  2500

#define ROBOBO_INERTIA_MS             500
#define ROBOBO_FULL_TURN_TIME         2800
#define ROBOBO_SLUG_TURN_SPEED        int(ROBOBO_FULL_TURN_TIME/(int(360/ROBOBO_PING_FOV)))

#define ROBOBO_DRV8833_AIN1_PIN       6
#define ROBOBO_DRV8833_AIN2_PIN       5

#define ROBOBO_DRV8833_BIN1_PIN       10
#define ROBOBO_DRV8833_BIN2_PIN       9

#define SERIAL_BAUD_RATE              115200

#define DIRECTION_CLOCKWISE           0
#define DIRECTION_FORWARD             1
#define DIRECTION_COUNTER_CLOCKWISE   2
#define DIRECTION_UNKNOWN             3

#define ROBOBO_FULL_SPEED             1.0

#define ROBOBO_RANDOM_CHECK_PROBABILITY 10

MotorDriver *motorLeft;
MotorDriver *motorRight;

Ping *pingSensor;

Servo pingServo;

int currentServoAngle = -1;

void setup() {
  // Enable serial communication.
#ifdef DEBUG
  Serial.begin(SERIAL_BAUD_RATE);
#endif

  randomSeed(analogRead(0));

  // Initializing debug pin.
  pinMode(ROBOBO_DEBUG_PIN, OUTPUT);

  // Initializing motors.
  motorLeft   = new MotorDriver(ROBOBO_DRV8833_AIN1_PIN, ROBOBO_DRV8833_AIN2_PIN);
  motorRight  = new MotorDriver(ROBOBO_DRV8833_BIN1_PIN, ROBOBO_DRV8833_BIN2_PIN);

  // Initializing PING)))
  pingSensor = new Ping(ROBOBO_PING_SENSOR_PIN);

  // Initializing servo.
  pingServo.attach(ROBOBO_SERVO_PIN);

  // Setting servo at neutral position.
  setServoAngle(ROBOBO_SERVO_ANGLE_NEUTRAL);

  // Waiting for the user...
  for (int i = 0; i < 10; i++) {
    digitalWrite(ROBOBO_DEBUG_PIN, i%2 ? HIGH : LOW);
    delay(500);
  }
}

void setServoAngle(int angle) {
  if (currentServoAngle != angle) {
    pingServo.write(angle);
    if (currentServoAngle >= 0) {
      int diff = abs(currentServoAngle - angle);
      delay(ROBOBO_SERVO_FULL_TURN_SPEED/int(360/diff));
    }
    currentServoAngle = angle;
  }
}

int randomCheck() {
  return (random(0, 100) < ROBOBO_RANDOM_CHECK_PROBABILITY);
}

long distanceToObstacle() {
  long pulse = pingSensor->Duration();

#ifdef DEBUG
  Serial.print("D(mm): ");
#endif

  if (pulse == PING_UNKNOWN_DISTANCE) {
#ifdef DEBUG
    Serial.println("?");
#endif
    return 0.0;
  }

  float mm = PING_DISTANCE_MM(pulse);

#ifdef DEBUG
  Serial.println(mm);
#endif

  return mm;
}

bool canKeepForward() {
  return (distanceToObstacle() > ROBOBO_MINIMUM_DISTANCE_TO_OBSTACLE);
}

int chooseDirection() {
#ifdef DEBUG
  Serial.println("Best direction?");
#endif

  byte choices;
  byte tested;
  byte dice;
  byte direction;

  choices = 0;
  tested = 0;

  for (int i = DIRECTION_CLOCKWISE; i < DIRECTION_UNKNOWN; i++) {

    while (1) {
      dice = random(DIRECTION_CLOCKWISE, DIRECTION_UNKNOWN);
      direction = (1 << dice);

      if ((tested | direction) != tested) {
        tested = tested | direction;
        break;
      }
    }

    switch (dice) {
      case DIRECTION_CLOCKWISE:
#ifdef DEBUG
        Serial.println("Testing clockwise");
#endif
        setServoAngle(ROBOBO_SERVO_ANGLE_NEUTRAL - ROBOBO_PING_FOV);
      break;
      case DIRECTION_FORWARD:
#ifdef DEBUG
        Serial.println("Testing forward");
#endif
        setServoAngle(ROBOBO_SERVO_ANGLE_NEUTRAL);
      break;
      case DIRECTION_COUNTER_CLOCKWISE:
#ifdef DEBUG
        Serial.println("Testing counter-clockwise");
#endif
        setServoAngle(ROBOBO_SERVO_ANGLE_NEUTRAL + ROBOBO_PING_FOV);
      break;
    }

    if (canKeepForward()) {
      choices = choices | direction;

#ifdef DEBUG
      Serial.print("Adding option: ");
      Serial.println(direction);
#endif
    }
  }

  // Getting servo back to neutral.
  setServoAngle(ROBOBO_SERVO_ANGLE_NEUTRAL);

  if (choices == 0) {
#ifdef DEBUG
    Serial.println("No choices, sorry.");
#endif
    return DIRECTION_UNKNOWN;
  }

  while (1) {
    dice = random(DIRECTION_CLOCKWISE, DIRECTION_UNKNOWN);
    if ((choices & (1 << dice)) > 0) {
      return dice;
    }
  }
}

void loop() {

  int direction;
  bool halt;

  direction = DIRECTION_FORWARD;
  halt = false;

  digitalWrite(ROBOBO_DEBUG_PIN, HIGH);

  if (!canKeepForward()) {
    halt = true;
  }

  if (halt || randomCheck()) {
#ifdef DEBUG
    Serial.println("Think again!");
#endif

    if (halt) {
      motorLeft->Halt();
      motorRight->Halt();
    }

    direction = chooseDirection();
  }

  switch (direction) {
    case DIRECTION_FORWARD:
#ifdef DEBUG
      Serial.println("Keep forward.");
#endif
      motorLeft->Forward(ROBOBO_FULL_SPEED);
      motorRight->Forward(ROBOBO_FULL_SPEED);
    break;
    case DIRECTION_CLOCKWISE:
#ifdef DEBUG
      Serial.println("Turn clockwise.");
#endif
      motorLeft->Forward(ROBOBO_FULL_SPEED);
      motorRight->Backward(ROBOBO_FULL_SPEED);
      delay(ROBOBO_SLUG_TURN_SPEED);
    break;
    case DIRECTION_COUNTER_CLOCKWISE:
#ifdef DEBUG
      Serial.println("Turn counter-clockwise.");
#endif
      motorLeft->Backward(ROBOBO_FULL_SPEED);
      motorRight->Forward(ROBOBO_FULL_SPEED);
      delay(ROBOBO_SLUG_TURN_SPEED);
    break;
    case DIRECTION_UNKNOWN:
#ifdef DEBUG
      Serial.println("Keep doing random stuff until finding a safe path.");
#endif

      int randomDelay = -1;

      while (!canKeepForward()) {

        if (randomDelay < 0) {
          switch (random(0, 3)) {
            case 0:
#ifdef DEBUG
              Serial.print("Turn clockwise.");
#endif
              motorLeft->Forward(ROBOBO_FULL_SPEED);
              motorRight->Backward(ROBOBO_FULL_SPEED);
            break;
            case 1:
#ifdef DEBUG
              Serial.print("Turn counter-clockwise.");
#endif
              motorLeft->Backward(ROBOBO_FULL_SPEED);
              motorRight->Forward(ROBOBO_FULL_SPEED);
            break;
            case 2:
#ifdef DEBUG
              Serial.print("Go backward.");
#endif
              motorLeft->Backward(ROBOBO_FULL_SPEED);
              motorRight->Backward(ROBOBO_FULL_SPEED);
            break;
          }

          randomDelay = random(ROBOBO_SLUG_TURN_SPEED, ROBOBO_FULL_TURN_TIME);
#ifdef DEBUG
          Serial.println(randomDelay);
#endif
        }

        if (randomDelay > 0) {
          int randomSample = random(int(ROBOBO_SLUG_TURN_SPEED/2), ROBOBO_SLUG_TURN_SPEED);
          delay(randomSample);
          randomDelay -= randomSample;
        }
      }

      motorLeft->Halt();
      motorRight->Halt();
    break;
  }

  digitalWrite(ROBOBO_DEBUG_PIN, LOW);
  delay(random(int(ROBOBO_INERTIA_MS/2), ROBOBO_INERTIA_MS));
}
