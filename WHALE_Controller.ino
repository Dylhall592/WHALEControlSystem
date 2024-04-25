#include <Wire.h>
#include <VL53L1X.h>

//----------------------------------------------------------------------------------------------------------------------------
//Connections
//----------------------------------------------------------------------------------------------------------------------------
//Pins and other components
//Motor Controllers
#define motor1pin1 6
#define motor1pin2 7
#define motor2pin1 9
#define motor2pin2 8
#define motor3pin1 12
#define motor3pin2 13
#define motor4pin1 11
#define motor4pin2 10
#define motor1analog 46
#define motor2analog 45
#define motor3analog 44
#define motor4analog 44
//--LIDAR Sensor
VL53L1X sensor;
//--Ultrasonic Sensor 1
#define trigPin 5
#define echoPin1 2
//--Ultrasonic Sensor 2
#define echoPin2 3
//--Ultrasonic Sensor 3
#define echoPin3 4

//----------------------------------------------------------------------------------------------------------------------------
//Other Variables
//----------------------------------------------------------------------------------------------------------------------------
char direction = "s";
long distance, duration, UltraSensor1, UltraSensor2;

int FrontRange = 350;
int FrontStopRange = 100;
int SideRange = 40;
int BackRange = 20;
int TurnTime = 50;

int rightDistance = 0;
int leftDistance = 0;

//----------------------------------------------------------------------------------------------------------------------------
//Speeds  ***CURRENTLY UNUSED***
//----------------------------------------------------------------------------------------------------------------------------
//Between 0-255, inclusive
int forwardSpeed = 200;
int backwardSpeed = 100;
int turnSpeed = 200;
int superSpeed = 255;

//----------------------------------------------------------------------------------------------------------------------------
//Individual Component Controls
//----------------------------------------------------------------------------------------------------------------------------
//Controls individual motor spin and sensor reads.
//(Takes pin 1 and 2 for a motor)
void motorStop(int motorPin1, int motorPin2){
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
}

void motorForward(int motorPin1, int motorPin2){
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
}

void motorBackward(int motorPin1, int motorPin2){
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
}

void motorSetSpeed(int Speed){
  analogWrite(motor1analog, Speed);
  analogWrite(motor2analog, Speed);
  analogWrite(motor3analog, Speed);
  analogWrite(motor4analog, Speed);
}

// Generates and reads the ultrasonic wave. Returns distance
long sonarSensor(int trigPinSensor,int echoPinSensor)//it takes the trigPIN and the echoPIN 
{
  digitalWrite(trigPinSensor, LOW);// put trigpin LOW 
  delayMicroseconds(2);// wait 2 microseconds
  digitalWrite(trigPinSensor, HIGH);// switch trigpin HIGH
  delayMicroseconds(10); // wait 10 microseconds
  digitalWrite(trigPinSensor, LOW);// turn it LOW again

  duration = pulseIn(echoPinSensor, HIGH);
  distance = (duration/2) / 29.1; // first we have to divide the duration by two
  return distance;
}

//----------------------------------------------------------------------------------------------------------------------------
//Motor Set Controls
//----------------------------------------------------------------------------------------------------------------------------
//Basic movements, controls a set of motors
void moveStop() {
  
  // Serial.println("---BRAKEING---");
  motorStop(motor1pin1, motor1pin2);
  motorStop(motor2pin1, motor2pin2);
  motorStop(motor3pin1, motor3pin2);
  motorStop(motor4pin1, motor4pin2);
  direction = "s";
}
  
void moveForward() {
  //Prevents pointlessly setting motors on repeat.
  if (direction != "f") {
    // Serial.println("---MOVING FORWARD---");
    motorSetSpeed(forwardSpeed);
    direction = "f";
    motorForward(motor1pin1, motor1pin2);
    motorForward(motor3pin1, motor3pin2);
    motorForward(motor2pin1, motor2pin2);
    motorForward(motor4pin1, motor4pin2);
  }
}

void moveBackward() {
  if(direction != "b") {
    // Serial.println("---MOVING BACK---");
    motorSetSpeed(backwardSpeed);
    direction = "b";
    motorBackward(motor1pin1, motor1pin2);
    motorBackward(motor3pin1, motor3pin2);
    motorBackward(motor2pin1, motor2pin2);
    motorBackward(motor4pin1, motor4pin2);
  }
}

void moveRight() {
  if(direction != "r")
    // Serial.println("---MOVING RIGHT---");
    motorSetSpeed(turnSpeed);
    direction = "r";
    motorForward(motor1pin1, motor1pin2);
    motorBackward(motor3pin1, motor2pin2);
    motorForward(motor3pin1, motor3pin2);
    motorBackward(motor4pin1, motor4pin2);
}

void moveLeft() {
  if(direction != "l");
    // Serial.println("---MOVING LEFT---");
    motorSetSpeed(turnSpeed);
    direction = "l";
    motorBackward(motor1pin1, motor1pin2);
    motorForward(motor2pin1, motor2pin2);
    motorBackward(motor3pin1, motor3pin2);
    motorForward(motor4pin1, motor4pin2);
}

//Returns the distance to an object, in millimeters, detected by the forward LIDAR
int lookForward(){
  sensor.read();
  return sensor.ranging_data.range_mm;
}

long lookRight() {
  long range = sonarSensor(trigPin, echoPin2);
  return range;
}

long lookLeft() {
  long range = sonarSensor(trigPin, echoPin1);
  return range;
}

long lookBack() {
  long range = sonarSensor(trigPin, echoPin3);
  return range;
}

//----------------------------------------------------------------------------------------------------------------------------
//System Controls
//----------------------------------------------------------------------------------------------------------------------------
//Setup and start, and anything else that's cool we want to add.

void escapeRight() {
  while (lookRight() > SideRange && lookForward() < FrontRange)
  {
    moveRight();
    Serial.println("RIGHT ESCAPING");
    delay(3);
  }
  if(lookLeft() < SideRange){
    moveStop();
    delay(300);
    escapeBack();
  }
  return;
}

void escapeLeft() {
  while (lookLeft() > SideRange && lookForward() < FrontRange)
  {   
    moveLeft();
    Serial.println("LEFT ESCAPING");
    delay(3);
  }
  if(lookLeft() < SideRange){
    moveStop();
    delay(300);
    escapeBack();
  }
  return;
}

void escapeBack() {
  int range = lookBack();
  while (range == 0 || (range > BackRange && lookForward() < FrontRange))
  {   
    range = lookBack();
    moveBackward();
    Serial.println("BACK ESCAPING");
    delay(3);
  }
  if (range != 0 && range < BackRange){
    moveStop();
    delay(300);
    escapeRight();
  }
  return;
}


void setup() {
  //UltraSensor 1 Setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin1, INPUT);
  //UltraSensor 2 Setup                           //<------ Interesting, we aren't setting up the LIDAR. Strange, and a potential problem......
  pinMode(echoPin2, INPUT);
  //UltraSensor 3 Setup                           //<------ Interesting, we aren't setting up the LIDAR. Strange, and a potential problem......
  pinMode(echoPin3, INPUT);
  //Motor Setup
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(motor3pin1, OUTPUT);
  pinMode(motor3pin2, OUTPUT);
  pinMode(motor4pin1, OUTPUT);
  pinMode(motor4pin2, OUTPUT);
  pinMode(motor1analog, OUTPUT);
  pinMode(motor2analog, OUTPUT);
  pinMode(motor3analog, OUTPUT);               
  pinMode(motor4analog, OUTPUT);

  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(50);
  motorSetSpeed(forwardSpeed);
  delay(2000);
}

void loop() {
  int forwardRange = lookForward();
  Serial.println(forwardRange);
  //If no object is in front range, move forward. If object is too close on sides, avoid
  if (forwardRange == 0 || forwardRange > FrontRange)
  {
       moveForward();
  }
  //If object is TOO close, reverse
  else if (forwardRange < FrontStopRange) {
      moveStop();
      delay(300);
      escapeBack();
  }

  //If object is in front range, turn
  else if (FrontStopRange < forwardRange && forwardRange < FrontRange){
      moveStop();
      rightDistance = lookRight();
      leftDistance = lookLeft();
      delay(300);
    if (rightDistance > leftDistance && rightDistance > SideRange){
      //motorSetSpeed(turnSpeed);
      escapeRight();
    }
    else if (leftDistance > rightDistance && leftDistance > SideRange){
      //motorSetSpeed(turnSpeed);
      escapeLeft();
    }
    else if (lookBack() > BackRange){
      escapeBack();
    }
  }

  //sensorTest();
  //motorTest();
  delay(5);
}


//----------------------------------------------------------------------------------------------------------------------------
//Debug Controls
//----------------------------------------------------------------------------------------------------------------------------
//Any functions that test a feature.
void motorTest() {
  //----------Motor Set Test--------------
  motorSetSpeed(forwardSpeed);
  moveStop();
  delay(800);
  Serial.println("--Forward--");
  moveForward();
  delay(2000);
  Serial.println("--Backward--");
  moveBackward();
  delay(2000);
  Serial.println("--Right--");
  moveRight();
  delay(3000);
  moveStop();
  delay(400);
  Serial.println("--Left--");
  moveLeft();
  delay(3000);
  //----------Individual Motor Test--------------
  moveStop();
  delay(800);

  Serial.print("--Motor 1--\n");
  motorForward(motor1pin1,motor1pin2);
  delay(800);
  motorBackward(motor1pin1,motor1pin2);
  delay(800);
  moveStop();
  delay(200);
  
  Serial.print("--Motor 2--\n");
  motorForward(motor2pin1,motor2pin2);
  delay(800);
  motorBackward(motor2pin1,motor2pin2);
  delay(800);
  moveStop();
  delay(200);

  Serial.print("--Motor 3--\n");
  motorForward(motor3pin1,motor3pin2);
  delay(800);
  motorBackward(motor3pin1,motor3pin2);
  delay(800);
  moveStop();
  delay(200);
  
  Serial.print("--Motor 4--\n");
  motorForward(motor4pin1,motor4pin2);
  delay(800);
  motorBackward(motor4pin1,motor4pin2);
  delay(800);
  moveStop();
  delay(200);
  //Super Speed Test
  motorSetSpeed(superSpeed);
  //----------Motor Set Test--------------
  moveStop();
  delay(800);
  Serial.println("--Forward--");
  moveForward();
  delay(2000);
  Serial.println("--Backward--");
  moveBackward();
  delay(2000);
  Serial.println("--Right--");
  moveRight();
  delay(3000);
  moveStop();
  delay(400);
  Serial.println("--Left--");
  moveLeft();
  delay(3000);
  //----------Individual Motor Test--------------
  moveStop();
  delay(800);

  Serial.print("--Motor 1--\n");
  motorForward(motor1pin1,motor1pin2);
  delay(800);
  motorBackward(motor1pin1,motor1pin2);
  delay(800);
  moveStop();
  delay(200);
  
  Serial.print("--Motor 2--\n");
  motorForward(motor2pin1,motor2pin2);
  delay(800);
  motorBackward(motor2pin1,motor2pin2);
  delay(800);
  moveStop();
  delay(200);

  Serial.print("--Motor 3--\n");
  motorForward(motor3pin1,motor3pin2);
  delay(800);
  motorBackward(motor3pin1,motor3pin2);
  delay(800);
  moveStop();
  delay(200);
  
  Serial.print("--Motor 4--\n");
  motorForward(motor4pin1,motor4pin2);
  delay(800);
  motorBackward(motor4pin1,motor4pin2);
  delay(800);
  moveStop();
  delay(200);
}

void sensorTest()
{
  Serial.print("Forward Range: ");
  Serial.print(lookForward());
  delay(5);
  Serial.print("      Right Range: ");
  Serial.print(lookRight());
  delay(5);
  Serial.print("      Left Range: ");
  Serial.print(lookLeft());
  delay(5);
  Serial.print("      Back Range: ");
  Serial.print(lookBack());
  delay(5);
  Serial.print("\n");
}

