// Per Aspera Ad Astra

// Sensors
#define s0       A0
#define s1       A1
#define s2       A2
#define s3       A3
#define s4       A4
#define s5       A5
#define s6       A6
#define s7       A7
// Motors
#define  pwma  11
#define  der1  9
#define  der2  10
#define  pwmb  5
#define  izq1  6
#define  izq2  7 
#define  STBY  8
// Others :p
#define LED_ON   13
#define LED       3
#define BOTON     2
#define PINBUZZER 12

// PID
const float KP = 0.15; // 0.15
const float KI = 0.0015; // 0.0015
const float KD = 0.9; // 0.9
const float KV = 6.5; // 6.5

// Speeds
int vmin = 60;
int vmax = 80;
int fvmax = 170;
int fvmin = 140;
int vbase;

// Variables
float digital[8];
float sensores[8];
float lectura_fondo[8];
float lectura_linea[8];
float umbral[8];
long int sumap, suma, pos, poslast, position;
int last_prop;
int error1 = 0, error2 = 0, error3 = 0, error4 = 0, error5 = 0, error6 = 0;
int proporcional = 0, integral = 0, derivativo = 0, diferencial = 0;
int setpoint = 350;
float times = 200;

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
  pinMode(BOTON, INPUT);
  tone(PINBUZZER, 2500, 50);
  digitalWrite(LED_ON, HIGH);
  digitalWrite(LED, 1);
  while (digitalRead(BOTON));
  tone(PINBUZZER, 2000, 100);
  for(int i = 0; i < 30; i++){
    fondos();
    digitalWrite(LED, 0);
    delay(20);
    digitalWrite(LED, 1);
    delay(20);
  }
  tone(PINBUZZER, 2000, 100);
  while (digitalRead(BOTON));
  tone(PINBUZZER, 2000, 100);
  for(int i = 0; i < 30; i++){
    lineas();
    digitalWrite(LED, 0);
    delay(20);
    digitalWrite(LED, 1);
    delay(20);
  }
  promedio();
  tone(PINBUZZER, 2000, 100);
  while (digitalRead(BOTON));
  tone(PINBUZZER, 2000, 350);
  digitalWrite(LED, 0);
  delay(250);
  digitalWrite(STBY,HIGH);
}

void loop() {
  //position = lectura();
  //Serial.println(position);
  lectura();
  if(interseccion()){
    pararMotores();
    tomarDecision();
  }
  else{
    PID(); 
    frenos();
  }
}

void fondos(){
  lectura_fondo[0] = analogRead(s0);
  lectura_fondo[1] = analogRead(s1);
  lectura_fondo[2] = analogRead(s2);
  lectura_fondo[3] = analogRead(s3);
  lectura_fondo[4] = analogRead(s4);
  lectura_fondo[5] = analogRead(s5);
  lectura_fondo[6] = analogRead(s6);
  lectura_fondo[7] = analogRead(s7);
}

void lineas(){
  lectura_linea[0] = analogRead(s0);
  lectura_linea[1] = analogRead(s1);
  lectura_linea[2] = analogRead(s2);
  lectura_linea[3] = analogRead(s3);
  lectura_linea[4] = analogRead(s4);
  lectura_linea[5] = analogRead(s5);
  lectura_linea[6] = analogRead(s6);
  lectura_linea[7] = analogRead(s7);
}

void promedio(){
  for(int i = 0; i < 8; i++){
    umbral[i]= (lectura_fondo[i]+ lectura_linea[i])/2;
  }
}

int lectura(void){
  sensores[0] = analogRead(s0);
  sensores[1] = analogRead(s1);
  sensores[2] = analogRead(s2);
  sensores[3] = analogRead(s3);
  sensores[4] = analogRead(s4);
  sensores[5] = analogRead(s5);
  sensores[6] = analogRead(s6);
  sensores[7] = analogRead(s7);
  
  for(int i = 0; i < 8; i++){
    if (sensores[i]<= umbral[i]){
      digital[i] = 0;
    }
    else{digital[i] =1;}
    //Serial.print(digital[i]);
    //Serial.print("\t");
  }
  //Serial.println(" ");

  sumap = (700*digital[0] + 600*digital[1] + 500*digital[2] + 400*digital[3] + 300*digital[4] + 200*digital[5] + 100*digital[6] + 0*digital[7]);  
  suma = (digital[0] + digital[1] + digital[2] + digital[3] + digital[4] + digital[5] + digital[6] + digital[7]);  
  pos = (sumap / suma);

  if(poslast <= 100 && pos == -1){
    pos = 0;
  }
  if(poslast >= 600 && pos == -1){
    pos = 700;
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
    motores(fvmax, -fvmin);
  }
  if (pos >= 600){
    digitalWrite(STBY,HIGH);
    motores(-fvmin, fvmax);
  }
}

bool interseccion(){
  int lineasDetectadas = 0;
  for(int i = 0; i < 8; i++){
    if (sensores[i] > umbral[i]){
      lineasDetectadas++;
    }
  }

  //dar que es una interseccion si se detectan 4 sensores
  return lineasDetectadas >= 5;
}

//funcion para actuar en intersecciones
void tomarDecision(){
  if (sensores[7] > umbral[7]) { //Linea derecha
    girarDerecha();
  }
  else if (sensores[0] > umbral[0]) {
    girarIzquierda();
  }
  else { //No hay giro
    seguirRecto();
  }
}

void seguirRecto(){
  motores(50, 50);
}

void girarIzquierda(){
  motores(-fvmax, fvmin);
  delay(times);
}

void girarDerecha(){
  motores(fvmin, -fvmax);
  delay(times);
}

void pararMotores() {
  motores(0, 0);  // Detener ambos motores
  delay(times/2);
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
