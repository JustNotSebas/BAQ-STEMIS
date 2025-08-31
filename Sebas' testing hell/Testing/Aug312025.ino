// Dum vita est, spes est.

// Never thought I would actually use libraries here
#include <math.h>

// QTR8 Sensors
#define  s0        A0
#define  s1        A1
#define  s2        A2
#define  s3        A3
#define  s4        A4
#define  s5        A5
#define  s6        A6
#define  s7        A7

// Digital Pins
#define  BUTTON    2
#define  LED       3
#define  pwmb      5
#define  izq1      6
#define  izq2      7 
#define  STBY      8
#define  der1      9
#define  der2      10
#define  pwma      11
#define  PINBUZZER 12
#define  LED_ON    13

// PID
const float KP = 1;
const float KI = 0;
const float KD = 1;
const float KV = 1;
const bool useIntersect = true;
// Speeds
int vmin = 70;
int vmax = 100;
int fvmax = 180;
int fvmin = 120;
int vbase;

// Variables
float digital[8], sensors[8], readingsBG[8], readingsLN[8], middlePoint[8];
long int weightSum, sensorSum;
int position, lastPosition;
int leftTrim = 0;
int rightTrim = 20;
float integral = 0, lastProportional = 0;
float setPoint = 350, times = 250;


void setup() {
  //Serial.begin(9600);
  pinMode(der1, OUTPUT);
  pinMode(der2, OUTPUT);
  pinMode(izq1, OUTPUT);
  pinMode(izq2, OUTPUT);
  pinMode(s0, INPUT);
  pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);
  pinMode(s5, INPUT);
  pinMode(s6, INPUT);
  pinMode(s7, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(LED_ON, OUTPUT);
  pinMode(BUTTON, INPUT);
  lastPosition = setPoint;
  tone(PINBUZZER, 2500, 50);

  digitalWrite(LED_ON, HIGH);
  digitalWrite(LED, 1);

  while (digitalRead(BUTTON));
  tone(PINBUZZER, 2000, 100);
  for(int i = 0; i < 40; i++){
    calibrateBG();
    digitalWrite(LED, 0);
    delay(10);
    digitalWrite(LED, 1);
    delay(10);
  }
  tone(PINBUZZER, 2000, 100);

  while (digitalRead(BUTTON));
  tone(PINBUZZER, 2000, 100);
  for(int i = 0; i < 40; i++){
    calibrateLN();
    digitalWrite(LED, 0);
    delay(10);
    digitalWrite(LED, 1);
    delay(10);
  }
  getMidPoint();
  tone(PINBUZZER, 2000, 100);

  while (digitalRead(BUTTON));
  tone(PINBUZZER, 2000, 350);
  digitalWrite(LED, 0);
  delay(350);
  digitalWrite(STBY,HIGH);
}

void loop() {
  readSensors();
  if(useIntersect){
    if(intersectLines()){
      stopMotors();
      whereToGo();
    }
    else{
      // Remove holdBrakes() from here
      getPID();
    }
  } else{
    // If not using intersects, you can keep holdBrakes() here
    getPID();
    holdBrakes();
  }
}

void calibrateBG(){
  readingsBG[0] = analogRead(s0);
  readingsBG[1] = analogRead(s1);
  readingsBG[2] = analogRead(s2);
  readingsBG[3] = analogRead(s3);
  readingsBG[4] = analogRead(s4);
  readingsBG[5] = analogRead(s5);
  readingsBG[6] = analogRead(s6);
  readingsBG[7] = analogRead(s7);
}

void calibrateLN(){
  readingsLN[0] = analogRead(s0);
  readingsLN[1] = analogRead(s1);
  readingsLN[2] = analogRead(s2);
  readingsLN[3] = analogRead(s3);
  readingsLN[4] = analogRead(s4);
  readingsLN[5] = analogRead(s5);
  readingsLN[6] = analogRead(s6);
  readingsLN[7] = analogRead(s7);
}

void getMidPoint(){
  for(int i = 0; i < 8; i++){
    middlePoint[i]= (readingsBG[i]+readingsLN[i])/2;
  }
}

// Corrected readSensors() function
int readSensors(void){
  sensors[0] = analogRead(s0);
  sensors[1] = analogRead(s1);
  sensors[2] = analogRead(s2);
  sensors[3] = analogRead(s3);
  sensors[4] = analogRead(s4);
  sensors[5] = analogRead(s5);
  sensors[6] = analogRead(s6);
  sensors[7] = analogRead(s7);
  
  for(int i = 0; i < 8; i++){
    if (sensors[i]<= middlePoint[i]){
      digital[i] = 0;
    }
    else{digital[i] = 1;}
  }

  weightSum = (700*digital[0] + 600*digital[1] + 500*digital[2] + 400*digital[3] + 300*digital[4] + 200*digital[5] + 100*digital[6] + 0*digital[7]);  
  sensorSum = (digital[0] + digital[1] + digital[2] + digital[3] + digital[4] + digital[5] + digital[6] + digital[7]); 
  
  if (sensorSum == 0) {
    position = lastPosition;
  } else {
    position = (weightSum / sensorSum);
  }
  
  lastPosition = position;
  return position;
}

// Corrected getPID() function
void getPID() {
  // Normalize error from -350 to 350 to a -1.0 to 1.0 range.
  float error = (float)(position - setPoint) / 350.0; 
  
  float P = error;
  integral += error;
  integral = constrain(integral, -100, 100); // Tweak this value

  float D = error - lastProportional;
  lastProportional = error;
  
  vbase = (int)(vmin + (vmax - vmin) * exp(-KV * fabs(P)));
  vbase = constrain(vbase, vmin, vmax);

  // Calculate the correction value
  float pid_output = (P * KP) + (integral * KI) + (D * KD);
  int correction = (int)(pid_output * vmax); // Scale correction by vmax

  int left_speed = vbase + correction;
  int right_speed = vbase - correction;
  
  // Apply trim
  left_speed += leftTrim;
  right_speed += rightTrim;

  // Apply to motors
  motorSpeed(left_speed, right_speed);
}


void holdBrakes(){
  // This function should be a failsafe, not a part of normal operation
  if (sensorSum == 0) { // Check for lost line
    if (lastPosition <= 100) {
        motorSpeed(fvmax, -fvmin); // Spin left
    } else if (lastPosition >= 600) {
        motorSpeed(-fvmin, fvmax); // Spin right
    }
  }
}

bool intersectLines(){
  int linesOnSensor = 0;
  for(int i = 0; i < 8; i++){
    if (sensors[i] > middlePoint[i]){
      linesOnSensor++;
    }
  }

  // If more than 4 sensors are on, it's an intersection
  return linesOnSensor >= 4;
}

void whereToGo(){
  digitalWrite(LED, 1);
  if (digital[7] == 1) {
    moveRight();
  }
  else if (digital[0] == 1) {
    moveLeft();
  }
  else {
    moveForward();
  }
  digitalWrite(LED, 0);
}

void moveForward(){
  motorSpeed(60, 60);
}

void moveLeft(){
  motorSpeed(-fvmax, fvmax);
}

void moveRight(){
  motorSpeed(fvmax, -fvmax);
}

void stopMotors() {
  motorSpeed(0, 0);  // Detener ambos motores
}

void motorSpeed(int izq, int der){
  digitalWrite(STBY,HIGH);
  izq = constrain(izq, -255, 255);
  der = constrain(der, -255, 255);
  digitalWrite(izq2, izq<0);
  digitalWrite(izq1, izq>=0); 
  analogWrite(pwma, abs(izq));
  digitalWrite(der2, der<0);
  digitalWrite(der1, der>=0);
  analogWrite(pwmb, abs(der));
  
}