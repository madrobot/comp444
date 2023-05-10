// PIN VARIABLES
// the motor will be controlled by the motor A pins on the motor driver
const int AIN1 = 13;      // control pin 1 on the motor driver for the right motor
const int AIN2 = 12;      // control pin 2 on the motor driver for the right motor
const int PWMA = 11;      // speed control pin on the motor driver for the right motor

// the left motor will be controlled by the motor B pins on the motor driver
const int PWMB = 10;      // speed control pin on the motor driver for the left motor
const int BIN2 = 9;       // control pin 2 on the motor driver for the left motor
const int BIN1 = 8;       // control pin 1 on the motor driver for the left motor

const int speedPin = A0;  // potentiometer is connected to analog pin A0
const int roamPin = A1;   // switch to turn roam mode on
const int followPin = A2; // switch to turn follow mode on
const int buzzerPin = 7;  // buzzer is connected to pin 4
const int trigPin = 6;    // ultrasonic distance sensor trigger pin
const int echoPin = 5;    // ultrasonic distance sensor echo pin

const int redPin = 4;     // red pin of RGB LED
const int greenPin = 3;   // green pin of RGB LED
const int bluePin = 2;    // blue pin of RGB LED

// VARIABLES
int speed = 0;       // starting speed for the motor
float distance = 0;       // the distance to travel in each direction
int stopTime = 200;       // amount of time that the robot will stop when it senses an object
int backupTime = 300;     // amount of time that the robot will back up when it senses an object
int turnTime = 200;       // amount that the robot will turn once it has backed up

int obstacleMinDistance = 15;
int obstacleMaxDistance = 25;
bool foundObstacle = false;
bool clearedObstacle = false;
int obstacleDirection = 0;

long barkTimer = 0;
long barkTicker = 0;

void setup() {
  // set this as a pullup to sense whether the switch is flipped
  pinMode(roamPin, INPUT_PULLUP);
  pinMode(followPin, INPUT_PULLUP);

  // this pin will send ultrasonic pulses out from the distance sensor
  pinMode(trigPin, OUTPUT);

  // this pin will sense when the pulses reflect back to the distance sensor
  pinMode(echoPin, INPUT);

  // set the motor control pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // set the RGB LED pins
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // setup serial communications
  // Serial.begin(9600);
  // Serial.println("Starting robot!");
}

void loop() {
  // modulate the potentiometer value between 0 and 255
  speed = getSpeed();

  // detect the distance using the ultrasonic sensor
  distance = getDistance();

  // if the switch is on...
  if (digitalRead(roamPin) == LOW) {
    roam();
    checkBark();    
  } else if (digitalRead(followPin) == LOW) {
    follow();
    checkBark();
  } else {
    stop();
  }

  // wait 50 ms between readings
  delay(50);
}

void roam() {
  if (!foundObstacle && distance < obstacleMinDistance) {
    foundObstacle = true;
    clearedObstacle = false;
    obstacleDirection = 255; // random(0, 255);

    stop();
    bark();
    delay(stopTime);
  } else if (foundObstacle && distance >= obstacleMaxDistance) {
    clearedObstacle = true;
  }

  if (foundObstacle && !clearedObstacle) {
      driveBackward(speed/1.5);
      light(LOW, LOW, HIGH);
  } else if (foundObstacle && clearedObstacle) {
      if (obstacleDirection > 128) {
        driveRight(speed);
        light(LOW, HIGH, HIGH);
      } else {
        driveLeft(speed);
        light(LOW, HIGH, HIGH);
      }
      delay(turnTime);
      if (distance >= obstacleMaxDistance) {
        foundObstacle = false;
      }
  } else {
    driveForward(speed);
    light(LOW, HIGH, LOW);
  }
}

void follow() {
  light(HIGH, LOW, HIGH);
  if (distance < 30) {
    driveBackward(speed/2);
  } else if (distance > 35) {
    driveForward(speed/2);
  } else {
    stop();
  }
}

void stop() {
  rightMotor(0);
  leftMotor(0);
  light(HIGH, LOW, LOW);
}

void driveForward(int motorSpeed) {
  rightMotor(motorSpeed*-1);
  leftMotor(motorSpeed);
}

void driveBackward(int motorSpeed) {
  rightMotor(motorSpeed);
  leftMotor(motorSpeed*-1);
}

void driveRight(int motorSpeed) {
  rightMotor(motorSpeed);
  leftMotor(motorSpeed*(255/200));
}

void driveLeft(int motorSpeed) {
  rightMotor(motorSpeed*(255/200)*-1);
  leftMotor(motorSpeed*-1);
}

void rightMotor(int motorSpeed) {
  // if the motor should drive forward (positive speed)
  if (motorSpeed > 0) {
    // set pin 1 to high
    digitalWrite(AIN1, HIGH);
    // set pin 2 to low
    digitalWrite(AIN2, LOW);
  } else if (motorSpeed < 0) {
    // if the motor should drive backward (negative speed)
    // set pin 1 to low
    digitalWrite(AIN1, LOW);
    // set pin 2 to high
    digitalWrite(AIN2, HIGH);
  } else {
    // if the motor should stop
    // set pin 1 to low
    digitalWrite(AIN1, LOW);
    // set pin 2 to low
    digitalWrite(AIN2, LOW);
  }

  // now that the motor direction is set, drive it at the entered speed
  analogWrite(PWMA, abs(motorSpeed));
}

void leftMotor(int motorSpeed) {
  // if the motor should drive forward (positive speed)
  if (motorSpeed > 0) {
    // set pin 1 to high
    digitalWrite(BIN1, HIGH);
    // set pin 2 to low
    digitalWrite(BIN2, LOW);
  } else if (motorSpeed < 0) {
    // if the motor should drive backward (negative speed)
    // set pin 1 to low
    digitalWrite(BIN1, LOW);
    // set pin 2 to high
    digitalWrite(BIN2, HIGH);
  } else {
    // if the motor should stop
    // set pin 1 to low
    digitalWrite(BIN1, LOW);
    // set pin 2 to low
    digitalWrite(BIN2, LOW);
  }

  // now that the motor direction is set, drive it at the entered speed
  analogWrite(PWMB, abs(motorSpeed));
}

int getSpeed() {
  return analogRead(speedPin)/(1023/255);
}

float getDistance() {
  // variable to store the time it takes for a ping to bounce off an object
  float echoTime;

  // variable to store the distance calculated from the echo time
  float calculatedDistance;

  // send out an ultrasonic pulse that's 10ms long
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // use the pulsein command to see how long it takes for the pulse to bounce back to the sensor
  echoTime = pulseIn(echoPin, HIGH);

  // calculate the distance of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)
  calculatedDistance = echoTime / 148.0;

  // send back the distance that was calculated
  return calculatedDistance;
}

void light(int red, int green, int blue) {
  digitalWrite(redPin, red);
  digitalWrite(greenPin, green);
  digitalWrite(bluePin, blue);  
}

void bark() {
  int tempo = 113;
  tone(buzzerPin, 440, tempo);
  delay(113);
  tone(buzzerPin, 392, tempo*4);
  delay(113);
  tone(buzzerPin, 349, tempo*2);
  delay(113);
  tone(buzzerPin, 440, tempo);
  delay(113);
}

void checkBark() {
  // set up the random barking
  barkTicker++;
  if (barkTimer == 0) {
    barkTimer = random(1, 512);
  }
  if (barkTicker >= barkTimer) {
    bark();
    barkTimer = 0;
    barkTicker = 0;
  }
}
