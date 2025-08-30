// Definiciones 
// Pines de Sensores 
#define s0       A0 
#define s1       A1 
#define s2       A2 
#define s3       A3 
#define s4       A4 
#define s5       A5 
#define s6       A6 
#define s7       A7 

// Motores: Izquierda 
#define pwma  5 
#define izq1  9 
#define izq2  10  

// Motores: Derecha 
#define pwmb  11 
#define der1  7 
#define der2  6 

// Pines varios 
#define LED_ON   13 
#define LED       3 
#define BOTON     2 
#define PINBUZZER 12 
#define STBY  8 

// Variables
// Inicialización de variables de calibración / posición 
int digital[8], sensor[8], calc_fondo[8], calc_linea[8], umbral[8]; 
long int inf, act, pos, poslast, position; 
unsigned long startTime; 
unsigned long currentTime;

// PID: Inicialización de variables 
int error1 = 0, error2 = 0, error3 = 0, error4 = 0, error5 = 0, error6 = 0; 
int proporcional = 0, integral = 0, derivativo = 0, diferencial = 0, last_prop; 
int setpoint = 300;  // Ajuste el setpoint hacia la izquierda
int move;

// PID: Rendimiento del PID 
const float KP = 0.135; 
const float KD = 1; 
const float KI = 0.00185; 
const float KV = 1; 
 
// Velocidades 
// Incio de la carrera (2000ms) 
const unsigned long changeDelay = 2000; // Duración antes de cambiar los valores 
int vmin = 200; // Velocidad minima 
int vmax = 255; // Velocidad máxima 

// Resto de la carrera 
int vxmin = 150; 
int vxmax = 180; 
int fadelante = 190; // Frenos hacia delante 
int fatras = 170; // Frenos hacia atras 
int vbase; 

// Arranque del Arduino 
void setup() { 
  //Serial.begin(9600); // Activar UNICAMENTE en caso de depuración 
  startTime = millis();  
  // Inicialización de pines 
  // Motores 
  pinMode(der1, OUTPUT); 
  pinMode(der2, OUTPUT); 
  pinMode(izq1, OUTPUT); 
  pinMode(izq2, OUTPUT); 
  // Pines 
  pinMode(s0, INPUT); 
  pinMode(s1, INPUT); 
  pinMode(s2, INPUT); 
  pinMode(s3, INPUT); 
  pinMode(s4, INPUT); 
  pinMode(s5, INPUT); 
  pinMode(s6, INPUT); 
  pinMode(s7, INPUT); 
  // LEDs y botones 
  pinMode(LED, OUTPUT); 
  pinMode(LED_ON, OUTPUT); 
  pinMode(BOTON, INPUT); 
  digitalWrite(LED_ON, HIGH); 
  digitalWrite(LED, 1); 
  tone(PINBUZZER, 2000, 100); 
  delay(200);  
  // Arranque de los motores (Tercer click) 
  while (digitalRead(BOTON)); 
  tone(PINBUZZER, 2000, 100); 
  delay(200);  
  digitalWrite(LED, 0); 
  } 

// Bucle principal del programa 
void loop() {
  unsigned long currentTime = millis();
  if (currentTime - startTime >= changeDelay) {
  vmin = vxmin;
  vmax = vxmax;
  }
  digitalWrite(STBY,HIGH); 
  lectura();  
  PID();  
  frenos();
} 

// Calibración de fondo: 
// Guarda la lectura de los 8 sensores en las variables correspondientes 
// Esto permite promediarlo con las lecturas de las lineas 
void fondos(){ 
  calc_fondo[0] = analogRead(s0); 
  calc_fondo[1] = analogRead(s1); 
  calc_fondo[2] = analogRead(s2); 
  calc_fondo[3] = analogRead(s3); 
  calc_fondo[4] = analogRead(s4); 
  calc_fondo[5] = analogRead(s5); 
  calc_fondo[6] = analogRead(s6); 
  calc_fondo[7] = analogRead(s7); 
} 
// Calibración de lineas: 
// Guarda la lectura de los 8 sensores en las variables correspondientes 
// Esto se promedia con las lecturas del fondo 
void lineas(){ 
  calc_linea[0] = analogRead(s0); 
  calc_linea[1] = analogRead(s1); 
  calc_linea[2] = analogRead(s2); 
  calc_linea[3] = analogRead(s3); 
  calc_linea[4] = analogRead(s4); 
  calc_linea[5] = analogRead(s5); 
  calc_linea[6] = analogRead(s6); 
  calc_linea[7] = analogRead(s7); 
} 

// Promedio de los valores fondo/linea 
// Esto da un margen de referencia para determinar el color actual que esta recibiendo el sensor 
void promedio(){ 
  for(int i = 0; i < 8; i++){ 
    umbral[i]= (calc_fondo[i]+ calc_linea[i])/2; 
  } 
} 

// Lectura continua de los sensores 
int lectura(void){ 
  // Leer los 8 sensores 
  sensor[0] = analogRead(s0); 
  sensor[1] = analogRead(s1); 
  sensor[2] = analogRead(s2); 
  sensor[3] = analogRead(s3); 
  sensor[4] = analogRead(s4); 
  sensor[5] = analogRead(s5); 
  sensor[6] = analogRead(s6); 
  sensor[7] = analogRead(s7); 

  for(int i = 0; i < 8; i++){ 
    if (sensor[i] <= 350){ 
      digital[i] = 0; 
    } 
    else{
    digital[i] = 1;
  }
 // Serial.print(digital[i]), Serial.print("\t"); // Para depuración: Imprime los valores de los 8 sensores 0 = Blanco & 1 = Negro 
  } 
  //Serial.println(" ");  
    // Calcula la influencia que tiene cada sensor en el calculo final 
  inf = (700*digital[0] + 600*digital[1] + 500*digital[2] + 400*digital[3] + 300*digital[4] + 200*digital[5] + 100*digital[6] + 0*digital[7]); 
    // Calcula el promedio actual de blanco/negro tomado por los sensores 
  act = (digital[0] + digital[1] + digital[2] + digital[3] + digital[4] + digital[5] + digital[6] + digital[7]); 
    // Divide la influencia y el promedio, dando una posición promedio 
  pos = (inf / act); 
  
  // En caso de que se salga de ciertos limites, forzar al carro a utilizar los frenos 
  if(poslast <= 100 && pos == -1){ 
    pos = 0; 
  } 
  if(poslast >= 600 && pos == -1){ 
    pos = 700; 
  } 
  poslast = pos; 
  return pos; 
} 

int tiebreak(int move){
  if(move == 0){
    sensor[0] = analogRead(s0); 
    sensor[1] = analogRead(s1); 
    sensor[2] = analogRead(s2);
    if(sensor[0] + sensor[1] >= 850){
      return 0;
    }
  } else if(move == 1){
    sensor[5] = analogRead(s5); 
    sensor[6] = analogRead(s6); 
    sensor[7] = analogRead(s7);
    if(sensor[6] + sensor[7] >= 850){
      return 700;
    }
  }
  return pos; // En caso de no cumplir ninguna de las condiciones anteriores
}

// Calculo del PID usado en marcha 
void PID(void) {
  // Error
  error6 = error5; 
  error5 = error4; 
  error4 = error3; 
  error3 = error2; 
  error2 = error1; 
  error1 = proporcional; 
  proporcional = setpoint - pos; 
  last_prop = proporcional; 

  // Ajuste extra al término proporcional cuando el error es hacia la izquierda
  if (proporcional > 0) {
    proporcional += 10; // Ajuste hacia la derecha
  }

  integral = integral + proporcional; 
  derivativo = proporcional - diferencial; 
  diferencial = proporcional; 
  
  // Calculo del PID
  move = ((KP * proporcional) + (KD * derivativo) + (KI * integral)) * KV; 
  if (move < -255) move = -255; 
  if (move > 255) move = 255; 
  
  position = move;
} 

// Asigna los valores del PID a los motores 
void frenos(void){ 
  if(pos == 700){ 
    analogWrite(pwma, fadelante); 
    digitalWrite(izq1, HIGH); 
    digitalWrite(izq2, LOW); 
    analogWrite(pwmb, fatras); 
    digitalWrite(der1, LOW); 
    digitalWrite(der2, HIGH); 
  } 
  else if(pos == 0){ 
    analogWrite(pwma, fatras); 
    digitalWrite(izq1, LOW); 
    digitalWrite(izq2, HIGH); 
    analogWrite(pwmb, fadelante); 
    digitalWrite(der1, HIGH); 
    digitalWrite(der2, LOW); 
  } 
  else{ 
    vbase = vmax - abs(position); 
    if(vbase <= vmin){ 
      vbase = vmin; 
    } 
    if (position < 0){ 
      analogWrite(pwma, vmax); 
      digitalWrite(izq1, HIGH); 
      digitalWrite(izq2, LOW); 
      analogWrite(pwmb, vbase); 
      digitalWrite(der1, HIGH); 
      digitalWrite(der2, LOW); 
    } 
    else if(position > 0){ 
      analogWrite(pwma, vbase); 
      digitalWrite(izq1, HIGH); 
      digitalWrite(izq2, LOW); 
      analogWrite(pwmb, vmax); 
      digitalWrite(der1, HIGH); 
      digitalWrite(der2, LOW); 
    } 
  } 
}
