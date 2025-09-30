#include "Config.h"

// --- Variables sensores ---
float sensores[8];
float lectura_fondo[8];
float lectura_linea[8];
float umbral[8];
uint8_t digital[8];

// --- Variables PID ---
float KP = KPDF;
float KI = KIDF;
float KD = KDDF;
float KV = KVDF;
float PID_integral = 0, PID_prev_error = 0, PID_derivative = 0;
float setpoint = 350.0;

// --- Motores ---
int vmin = VMINDF;
int vmax = VMAXDF;
int fvup = FVUPDF;
int fvdown = FVDOWNDF;
int vbase;
float last_pos = 350.0;

// --- Intersección ---
bool en_interseccion = false;
const int MIN_SENSORES_INTERSECCION = 4;
unsigned long tiempo_interseccion = 0;
const unsigned long INTERSECCION_DELAY = 120;

// --- Utilidades ---
float times = 180;
unsigned long lastMillis = 0;
