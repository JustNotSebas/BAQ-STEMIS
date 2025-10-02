#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>

// Pines
// Sensores
#define s0 A0
#define s1 A1
#define s2 A2
#define s3 A3
#define s4 A4
#define s5 A5
#define s6 A6
#define s7 A7

// Inputs/Outputs
#define LED_ON   13
#define LED      3
#define BOTON    2
#define PINBUZZER 12

// Motores
#define pwma  5
#define der1  6
#define der2  7
#define pwmb  11
#define izq1  10
#define izq2  9 
#define STBY  8

//Variables
//Sensores
extern float sensores[8];
extern float lectura_fondo[8];
extern float lectura_linea[8];
extern float umbral[8];
extern uint8_t digital[8];

// -PID
extern float KP, KI, KD, KV;
extern float PID_integral, PID_prev_error, PID_derivative;
extern float setpoint;

// PID defaults
const float KPDF = 0.1765;
const float KIDF = 0.001;
const float KDDF = 0.69;
const float KVDF = 5;

// Motores defaults
const int VMINDF = 80;
const int VMAXDF = 130;
const int FVUPDF = 180;
const int FVDOWNDF = 140;

// Motores
extern int vmin, vmax, fvup, fvdown, vbase;
extern float last_pos;

// Intersección
extern bool en_interseccion;
extern const int MIN_SENSORES_INTERSECCION;
extern unsigned long tiempo_interseccion;
extern const unsigned long INTERSECCION_DELAY;

// En el nombre del Padre y del Hijo y del Espiritu Santo, Amén
extern float times;
extern unsigned long lastMillis;

#endif
