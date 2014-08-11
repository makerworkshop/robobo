// Copyright (c) 2014 José Carlos Nieto, https://menteslibres.net/xiam
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

#include <motor_driver.h>
#include <ping.h>

#define GENERAL_LED_PIN       13

// Minimum safe distance to obstacle.
#define MIN_SANE_DISTANCE     100

#define PING_PIN              4
#define PARALLAX_SERVO_SIGNAL 2

#define ANGLE_NEUTRAL         90
#define ANGLE_FOV             30

#define SERVO_TURN_SPEED      300
#define SAFE_INERTIA_TIME     500

#define DRV8833_AIN1_PIN      6
#define DRV8833_AIN2_PIN      5

#define DRV8833_BIN1_PIN      10
#define DRV8833_BIN2_PIN      9

#define DIRECTION_FORWARD     0
#define DIRECTION_LEFT        1
#define DIRECTION_RIGHT       2
#define DIRECTION_BACKWARD    3

// DRV8833.
MotorDrv *motorA;
MotorDrv *motorB;

// PING))) Sensor.
Ping *ping;

// Parallax servo.
Servo mainServo;

void setup() {
  // Enable serial communication.
  Serial.begin(9600);

  // Initializing led pin.
  pinMode(GENERAL_LED_PIN, OUTPUT);

  // Initializing motors.
  motorA = new MotorDrv(DRV8833_AIN1_PIN, DRV8833_AIN2_PIN);
  motorB = new MotorDrv(DRV8833_BIN1_PIN, DRV8833_BIN2_PIN);

  // Initializing PING)))
  ping = new Ping(PING_PIN);

  // Initializing servo.
  mainServo.attach(PARALLAX_SERVO_SIGNAL);

  // Setting servo at neutral position.
  mainServo.write(ANGLE_NEUTRAL);

  // Waiting for the user...
  for (int i = 0; i < 5; i++) {
    digitalWrite(GENERAL_LED_PIN, HIGH);
    delay(500);
    digitalWrite(GENERAL_LED_PIN, LOW);
    delay(500);
  }
}

// Has a 5 in 100 possibility of returning 1.
int randomCheck() {
  if (random(0, 100) >= 95) {
    return 1;
  }
  return 0;
}

// Returns 1 if the way is clear.
int canKeepForward() {
  long distance;

  distance = ping->distance();

  if (distance > MIN_SANE_DISTANCE) {
    return 1;
  }

  return 0;
}

// Checks environment to determine the position with less obstancles.
int determineBestDirection() {

  int bestDirection;
  int testDirection;

  long distance;
  long maxDistance;
  bool okForward;

  maxDistance   = -1;
  bestDirection = -1;
  okForward = false;

  for (int i = 0; i < 3; i++) {
    switch (i) {
      case 0:
        mainServo.write(ANGLE_NEUTRAL + ANGLE_FOV);
        testDirection = DIRECTION_LEFT;
      break;
      case 1:
        mainServo.write(ANGLE_NEUTRAL);
        testDirection = DIRECTION_FORWARD;
      break;
      case 2:
        mainServo.write(ANGLE_NEUTRAL - ANGLE_FOV);
        testDirection = DIRECTION_RIGHT;
      break;
    }

    delay(SERVO_TURN_SPEED);

    distance = ping->distance();

    Serial.print("Distance: ");
    Serial.print("-> ");
    Serial.print(i);
    Serial.print("-> ");
    Serial.print(distance);
    Serial.println("cm.");

    if (testDirection == DIRECTION_FORWARD && distance >= 200) {
      okForward = true;
    }

    if (bestDirection < 0 || distance > maxDistance) {
      maxDistance = distance;
      bestDirection = testDirection;
    }
  }

  mainServo.write(ANGLE_NEUTRAL);
  delay(SERVO_TURN_SPEED);

  if (okForward) {
    // No matter what is the best position, if we have a field clear ahead
    // choose it.
    return DIRECTION_FORWARD;
  }

  if (maxDistance < MIN_SANE_DISTANCE) {
    // If we don't have any option, turn backwards.
    return DIRECTION_BACKWARD;
  }

  return bestDirection;
}

void loop() {

  int direction;
  bool stopped;

  direction = DIRECTION_FORWARD;

  stopped = false;

  if (!canKeepForward()) {
    stopped = true;
  }

  if (stopped || randomCheck()) {
    digitalWrite(GENERAL_LED_PIN, HIGH);

    if (stopped) {
      motorA->setSpeed(0);
      motorB->setSpeed(0);
    }

    direction = determineBestDirection();
    Serial.print("Best direction: ");
    Serial.print(direction);
    Serial.println(".");

    digitalWrite(GENERAL_LED_PIN, LOW);
  }

  switch (direction) {
    case DIRECTION_FORWARD:
      Serial.println("Keep forward.");
      motorA->setSpeed(255);
      motorB->setSpeed(255);
    break;
    case DIRECTION_LEFT:
      Serial.println("Go left.");
      motorA->setSpeed(-255);
      motorB->setSpeed(255);
      delay(SAFE_INERTIA_TIME*2);
    break;
    case DIRECTION_RIGHT:
      Serial.println("Go right.");
      motorA->setSpeed(255);
      motorB->setSpeed(-255);
      delay(SAFE_INERTIA_TIME*2);
    break;
    case DIRECTION_BACKWARD:
      motorA->setSpeed(-255);
      motorB->setSpeed(-255);

      delay(SAFE_INERTIA_TIME*3);

      Serial.println("Turn until safe.");
      int safe = 0;
      int k;

      for (k = 0; safe == 0; k++) {
        Serial.print("k: ");
        Serial.println(k);
        int j;
        for (j = 0; j < 100; j++) {
          digitalWrite(GENERAL_LED_PIN, HIGH);
          if (canKeepForward()) {
            safe = 1;
            break;
          };
          if (k%2) {
            motorA->setSpeed(255);
            motorB->setSpeed(-255);
          } else {
            motorA->setSpeed(-255);
            motorB->setSpeed(255);
          };
          digitalWrite(GENERAL_LED_PIN, LOW);
          delay(100);
        };
      };

      motorA->setSpeed(0);
      motorB->setSpeed(0);

      delay(SAFE_INERTIA_TIME);

    break;
  }

  digitalWrite(GENERAL_LED_PIN, LOW);

  delay(SAFE_INERTIA_TIME);
}
