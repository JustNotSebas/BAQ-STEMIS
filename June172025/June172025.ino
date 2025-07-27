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
// Calibración y posicion
int digital[8], sensor[8], calc_fondo[8], calc_linea[8], umbral[8]; 
long int inf, act, pos, poslast, position; 
int setpoint = 350; // Punto de guia
unsigned long startTime = 0;
float times = 235;
// Derivado al PID
int error1 = 0, error2 = 0, error3 = 0, error4 = 0, error5 = 0, error6 = 0; 
int proporcional = 0, integral = 0, derivativo = 0, diferencial = 0, last_prop; 
int move;
// Constantes
// PID
const float KP = 0.16;
const float KD = 1;
const float KI = 0.0017;
const float KV = 6;
// Velocidad
int vbase; 
// Inicio
int vmin = 80; // Velocidad minima
int vmax = 90; // Velocidad maxima
int frmax = 140; // Frenos (delante)
int frmin = 120; // Frenos (atras)
// Turbo
const unsigned long changeDelay = 5000; // Tiempo de espera antes de cambiar los valores (1000ms = 1s)
int vxmin = 100; 
int vxmax = 110; 
int fxmax = 150; 
int fxmin = 170; 

void setup(){
  //Serial.begin(9600); // Depuración
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
  // Calibración de fondos (Primer click) 
  while (digitalRead(BOTON)); 
  tone(PINBUZZER, 2000, 100); 
  delay(200);  
  for(int i = 0; i < 50; i++){ 
    fondos(); 
    digitalWrite(LED, 0); 
    delay(20); 
    digitalWrite(LED, 1); 
    delay(20); 
  } 
  tone(PINBUZZER, 2000, 100); 
  delay(200); 
  // Calibración de lineas (Segundo click) 
  while (digitalRead(BOTON)); 
  tone(PINBUZZER, 2000, 100); 
  delay(200);  
  for(int i = 0; i < 50; i++){ 
    lineas(); 
    digitalWrite(LED, 0); 
    delay(20); 
    digitalWrite(LED, 1); 
    delay(20); 
  } 
  promedio(); // Promediación de valores 
  tone(PINBUZZER, 2000, 100); 
  delay(200);  
  // Arranque de los motores (Tercer click) 
  while (digitalRead(BOTON)); 
  tone(PINBUZZER, 2000, 100); 
  delay(200);  
  digitalWrite(LED, 0); 
  startTime = millis(); // Inicio del tiempo
} 

void loop(){
  lectura();
  if(interseccion()){
    motorMove("stop");
    tomarDecision();
  } 
  else{
    PID(); 
    frenos();
  }
  
  unsigned long currentTime = millis();  // Tiempo actual
  if (currentTime - startTime >= changeDelay){ // Despues del delay, aumentar la velocidad y los frenos para contrarrestar
    vmin = vxmin;
    vmax = vxmax;
    frmax = fxmax;
    frmin = fxmin;
  }
}

// Calibración de fondo
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

// Calibración de lineas
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
    if (sensor[i]>= umbral[i]){ 
      digital[i] = 1;
    } 
    else{
    digital[i] = 0;
  }

  inf = (700*digital[0] + 600*digital[1] + 500*digital[2] + 400*digital[3] + 300*digital[4] + 200*digital[5] + 100*digital[6] + 0*digital[7]);  
  act = (digital[0] + digital[1] + digital[2] + digital[3] + digital[4] + digital[5] + digital[6] + digital[7]);  
  pos = (inf / act);

  if(poslast <= 100 && pos == -1){
    pos = 0;
  }
  if(poslast >= 700 && pos == -1){
    pos = 750;
  }

  poslast = pos;
  return pos;
}

void PID(){
  proporcional = pos - setpoint;
  derivativo = proporcional - last_prop;
  integral = error1 + error2 + error3 + error4 + error5 + error6;
  last_prop = proporcional;
  error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2;
  error2 = error1;
  error1 = proporcional;

  if((proporcional*integral)<0) integral = 0;
  diferencial = (proporcional * KP) + (derivativo * KD) + (integral * KI);
  
  vbase = vmin+(vmax-vmin)*exp(-KV*abs(KP*proporcional));
  motores(vbase+diferencial, vbase-diferencial);
}

void frenos(){
  if (pos <= 100){
    digitalWrite(STBY,HIGH);
    motores(frmin, -frmax);
  }
  if (pos >= 600){
    digitalWrite(STBY,HIGH);
    motores(-frmin, frmax);
  }
}

bool interseccion(){
  int lineasDetectadas = 0;
  for(int i = 0; i < 8; i++){
    if (sensor[i] > umbral[i]){
      lineasDetectadas++;
    }
  }

  //dar que es una interseccion si se detectan 4 sensores
  return lineasDetectadas >= 4;
}

//funcion para actuar en intersecciones
void tomarDecision(){
  if (sensor[7] > umbral[7]) { //Linea derecha
    motorMove("izq");
  }
  else if (sensor[0] > umbral[0]){ //Linea izquierda
    motorMove("der");
  }
  else { //No hay giro
    motorMove("front");
  }
}

void motorMove(String action){
  if(action == "izq"){
    motores(170, -100);
    delay(times);
  }
  if(action == "der"){
    motores(-120, 170);
    delay(times);
  }
  if(action == "front"){
    motores(vmin, vmin);
  }
  if(action == "stop"){
    motores(0, 0);  // Detener ambos motores
  }
}

void motores(int izq, int der){
  izq = constrain(izq, -255, 255);
  der = constrain(der, -255, 255);
  digitalWrite(izq2, izq<0);
  digitalWrite(izq1, izq>=0); 
  analogWrite(pwma, abs(izq));
  digitalWrite(der2, der<0);
  digitalWrite(der1, der>=0);
  analogWrite(pwmb, abs(der));
}
