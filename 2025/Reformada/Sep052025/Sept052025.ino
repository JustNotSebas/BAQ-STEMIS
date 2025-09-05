// Per Aspera Ad Astra

// ------------- Pines analógicos (Sensores) -------------
#define s0 A0
#define s1 A1
#define s2 A2
#define s3 A3
#define s4 A4
#define s5 A5
#define s6 A6
#define s7 A7

// ------------- Pines digitales -------------
#define BOTON     2
#define LED       3
#define pwmb      5
#define izq1      6
#define izq2      7 
#define STBY      8
#define der1      9
#define der2      10
#define pwma      11 
#define PINBUZZER 4   // 2
#define LED_ON    13

// ------------- PID Constantes -------------
const float KP = 0.15;
const float KI = 0.001;
const float KD = 0.2;
const float KV = 5.5;

// ------------- Velocidades -------------
const int vmin = 50;
const int vmax = 80;
const int fvmax = 175;
const int fvmin = 135;

// ------------- Umbrales y sensores -------------
float sensores[8];
float digital[8];
float fondo_min[8], fondo_max[8];
float linea_min[8], linea_max[8];
float umbral[8];

// ------------- Variables PID -------------
int setpoint = 350;
int last_proporcional = 0;
int errores[6] = {0}; // para la integral
int pos = 0;
int pos_last = 0;

// ------------- Temporizador intersección -------------
const float delay_inter = 210;

// --------------------------------------------------
// -------------------- SETUP -----------------------
// --------------------------------------------------

void setup() {
  //Serial.begin(9600);
  inicializarPins();
  tonoInicio();

  esperarBoton();
  calibrarSensores(fondo_min, fondo_max, "Fondo");

  esperarBoton();
  calibrarSensores(linea_min, linea_max, "Línea");

  calcularUmbrales();

  esperarBoton();
  tonoConfirmacion(350);
  digitalWrite(STBY, HIGH);
}

// --------------------------------------------------
// --------------------- LOOP -----------------------
// --------------------------------------------------

void loop() {
  pos = leerLinea();

  if (esInterseccion()) {
    pararMotores();
    tomarDecision();
  } else if (pos == -1) {
    aplicarFrenos(); // línea perdida
  } else {
    ejecutarPID(pos);
  }
}

// --------------------------------------------------
// ---------------- FUNCIONES BASE ------------------
// --------------------------------------------------

void inicializarPins() {
  for (int i = 0; i < 8; i++) pinMode(A0 + i, INPUT);
  pinMode(BOTON, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(LED_ON, OUTPUT);
  pinMode(der1, OUTPUT);
  pinMode(der2, OUTPUT);
  pinMode(izq1, OUTPUT);
  pinMode(izq2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(pwma, OUTPUT);
  pinMode(pwmb, OUTPUT);
  pinMode(PINBUZZER, OUTPUT);

  digitalWrite(LED_ON, HIGH);
  digitalWrite(LED, HIGH);
}

void tonoInicio() {
  tone(PINBUZZER, 2500, 50);
}

void tonoConfirmacion(int duracion) {
  tone(PINBUZZER, 2000, duracion);
  delay(duracion);
}

void esperarBoton() {
  while (digitalRead(BOTON));
}

// --------------------------------------------------
// -------------- CALIBRACIÓN ------------------------
// --------------------------------------------------

// ✅ ahora guarda mínimo y máximo de cada sensor
void calibrarSensores(float lectura_min[], float lectura_max[], const char* tipo) {
  tonoConfirmacion(100);

  for (int j = 0; j < 8; j++) {
    lectura_min[j] = 1023;
    lectura_max[j] = 0;
  }

  for (int i = 0; i < 80; i++) { // más lecturas para mayor precisión
    for (int j = 0; j < 8; j++) {
      float valor = analogRead(A0 + j);
      if (valor < lectura_min[j]) lectura_min[j] = valor;
      if (valor > lectura_max[j]) lectura_max[j] = valor;
    }
    digitalWrite(LED, LOW);
    delay(10);
    digitalWrite(LED, HIGH);
    delay(10);
  }

  tonoConfirmacion(100);
}

void calcularUmbrales() {
  for (int i = 0; i < 8; i++) {
    umbral[i] = (fondo_min[i] + linea_max[i]) / 2.0; // ✅ umbral más realista
  }
}

// --------------------------------------------------
// ---------------- LECTURA SENSORES ----------------
// --------------------------------------------------

int leerLinea() {
  int suma = 0;
  long sumaPonderada = 0;

  for (int i = 0; i < 8; i++) {
    sensores[i] = analogRead(A0 + i);
    digital[i] = sensores[i] > umbral[i] ? 1 : 0;
    suma += digital[i];
    sumaPonderada += digital[i] * (700 - i * 100);
  }

  if (suma == 0) return -1; // Línea perdida

  pos_last = pos;
  return sumaPonderada / suma;
}

// --------------------------------------------------
// --------------------- PID ------------------------
// --------------------------------------------------

void ejecutarPID(int pos) {
  int proporcional = pos - setpoint;
  int derivativo = proporcional - last_proporcional;

  // Actualizar errores para la integral
  for (int i = 5; i > 0; i--) errores[i] = errores[i - 1];
  errores[0] = proporcional;

  int integral = 0;
  for (int i = 0; i < 6; i++) integral += errores[i];

  if (proporcional * integral < 0) integral = 0;

  // ✅ limitar la integral (anti-windup)
  integral = constrain(integral, -300, 300);

  float salidaPID = KP * proporcional + KD * derivativo + KI * integral;

  last_proporcional = proporcional;

  int vbase = vmin + (vmax - vmin) * exp(-KV * abs(KP * proporcional));
  motores(vbase + salidaPID, vbase - salidaPID);
}

// --------------------------------------------------
// --------------------- MOTORES --------------------
// --------------------------------------------------

void motores(int izq, int der) {
  izq = constrain(izq, -255, 255);
  der = constrain(der, -255, 255);

  digitalWrite(STBY, HIGH);

  digitalWrite(izq1, izq >= 0);
  digitalWrite(izq2, izq < 0);
  analogWrite(pwma, abs(izq));

  digitalWrite(der1, der >= 0);
  digitalWrite(der2, der < 0);
  analogWrite(pwmb, abs(der));
}

void pararMotores() {
  motores(0, 0);
  delay(delay_inter / 2);
}

void aplicarFrenos() {
  digitalWrite(STBY, HIGH);

  if (pos_last <= 100) {
    motores(fvmax, -fvmin);
  } else if (pos_last >= 600) {
    motores(-fvmin, fvmax);
  }
}

// --------------------------------------------------
// ------------- INTERSECCIONES ---------------------
// --------------------------------------------------

bool esInterseccion() {
  int activos = 0;
  for (int i = 0; i < 8; i++) {
    if (sensores[i] > umbral[i]) activos++;
  }
  return activos >= 5; // ✅ antes 4, ahora más seguro
}

void tomarDecision() {
  if (sensores[7] > umbral[7]) {
    girarDerecha();
  } else if (sensores[0] > umbral[0]) {
    girarIzquierda();
  } else {
    seguirRecto();
  }
}

void girarIzquierda() {
  motores(-fvmax, fvmin);
  delay(delay_inter / 1.5);
}

void girarDerecha() {
  motores(fvmin, -fvmax);
  delay(delay_inter / 1.5);
}

void seguirRecto() {
  motores(100, 100); // ✅ más rápido que antes (antes 60,60)
}
