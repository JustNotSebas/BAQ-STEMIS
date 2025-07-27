#include <QTRSensors.h>

QTRSensors qtr;

// Definición de los pines para el driver TB6612 y los motores
int pwmb = 11;
int inb2 = 10;
int inb1 = 9;
int stndby = 8;
int ina1 = 7;
int ina2 = 6;
int pwma = 5;
int led = 3;

// Definición de la cantidad de sensores
#define SENSOR_COUNT 8
uint16_t sensorValues[SENSOR_COUNT];

// Parámetros del PID
const int vmin = 240;
const int vmax = 255;
const float kp = 0.030;   // Control proporcional
const float ki = 0.0015;  // Control integral
const float kd = 0.75;     // Control derivativo
const float kv = 0.9;    // Factor para la velocidad base
int p, d, u, vbase;
long i;
int p_old;

void setup() {
  Serial.begin(9600);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ 0, 1, 2, 3, 4, 5, 6, 7 }, SENSOR_COUNT);

  // Inicialización de pines para el driver TB6612
  pinMode(stndby, OUTPUT);
  pinMode(ina1, OUTPUT);
  pinMode(ina2, OUTPUT);
  pinMode(pwma, OUTPUT);
  pinMode(inb1, OUTPUT);
  pinMode(inb2, OUTPUT);
  pinMode(pwmb, OUTPUT);
  pinMode(led, OUTPUT);
  digitalWrite(stndby, HIGH);
  digitalWrite(led, HIGH);

  // Calibración de los sensores QTR-8RC
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(led, LOW);
}

void loop() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  p = -7 * sensorValues[0] - 5 * sensorValues[1] - 3 * sensorValues[2] - sensorValues[3] + sensorValues[4] + 3 * sensorValues[5] + 5 * sensorValues[6] + 7 * sensorValues[7];  
  //p =p = -7*sensorValues[7]-5*sensorValues[6]-3*sensorValues[5]-sensorValues[4]+sensorValues[3]+3*sensorValues[2]+5*sensorValues[1]+7*sensorValues[0];
  i = i + p;
  d = p - p - p_old;
  if ((p * i) < 0) i = 0;
  u = kp * p + ki * i + kd * d;
  vbase = vmin + (vmax - vmin) * exp(-kv * abs(kp * p));
  drive(vbase + u, vbase - u);
}
void drive(int L, int R) {
  // Constricción de valores para PWM
  L = constrain(L, -255, 255);
  R = constrain(R, -255, 255);

  // Control de motor izquierdo
  digitalWrite(ina1, L < 0);
  digitalWrite(ina2, L >= 0);
  analogWrite(pwma, abs(L));

  // Control de motor derecho
  digitalWrite(inb1, R >= 0);
  digitalWrite(inb2, R < 0);
  analogWrite(pwmb, abs(R));
}
