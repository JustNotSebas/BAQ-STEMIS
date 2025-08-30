// Per Aspera Ad Astra

// Sensors [Analog]
#define  s0        A0
#define  s1        A1
#define  s2        A2
#define  s3        A3
#define  s4        A4
#define  s5        A5
#define  s6        A6
#define  s7        A7

// Pins [Digital]
#define  BOTON     2
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
const float KP = 0.131; // 0.15 || 0.139
const float KI = 0.00148; // 0.0015 || 0.00135
const float KD = 0.862; // 0.9 || 0.86
const float KV = 6; // 6.5 || ''

// Speeds
int vmin = 80;
int vmax = 110;
int fvmax = 220;
int fvmin = 80;
int vbase;

// Variables
float digital[8], sensores[8], lectura_fondo[8], lectura_linea[8], umbral[8];
long int sumap, suma, pos, poslast, position;
int error1 = 0, error2 = 0, error3 = 0, error4 = 0, error5 = 0, error6 = 0;
int proporcional = 0, integral = 0, derivativo = 0, diferencial = 0;
int setpoint = 370, times = 100;
int last_prop;

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
  for(int i = 0; i < 40; i++){
    fondos();
    digitalWrite(LED, 0);
    delay(10);
    digitalWrite(LED, 1);
    delay(10);
  }
  tone(PINBUZZER, 2000, 100);

  while (digitalRead(BOTON));
  tone(PINBUZZER, 2000, 100);
  for(int i = 0; i < 40; i++){
    lineas();
    digitalWrite(LED, 0);
    delay(10);
    digitalWrite(LED, 1);
    delay(10);
  }
  promedio();
  tone(PINBUZZER, 2000, 100);

  while (digitalRead(BOTON));
  tone(PINBUZZER, 2000, 350);
  digitalWrite(LED, 0);
  delay(350);
  digitalWrite(STBY,HIGH);
}

void loop() {
  lectura();
  PID();
  frenos();
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
    else{digital[i] = 1;}
  }

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
  motores(vbase+diferencial, vbase-diferencial+25);
}

void frenos(){
  if (pos <= 100){
    digitalWrite(STBY,HIGH);
    motores(fvmin+20, -fvmax-20);
  }
  if (pos >= 600){
    digitalWrite(STBY,HIGH);
    motores(-fvmax, fvmin);
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