#define s0       A0
#define s1       A1
#define s2       A2
#define s3       A3
#define s4       A4
#define s5       A5
#define s6       A6
#define s7       A7
#define LED_ON   13
#define LED       3
#define BOTON     2
#define PINBUZZER 12

//pines motores
#define  pwma  5
#define  der1  7
#define  der2  6
#define  pwmb  11
#define  izq1  9
#define  izq2  10 
#define  STBY  8

//int umbral = 120;
int digital[8];
int sensores[8];
int lectura_fondo[8];
int lectura_linea[8];
int umbral[8];

long int sumap, suma, pos, poslast, position;


//////////DATOS PARA LA INTEGRAL///////////////
int error1 = 0;
int error2 = 0;
int error3 = 0;
int error4 = 0;
int error5 = 0;
int error6 = 0;

/////////////////VARIABLES PID///////////////////
int proporcional = 0;
int integral = 0;
int derivativo = 0;
int diferencial = 0;
int last_prop;
int setpoint = 350;

//////////////////////PID////////////////////////
const float KP = 0.11;
const float KD = 1;
const float KI = 0.006;
const float KV = 7;

/////////////////VELOCIDADES///////////////////
int vmin = 30;
int vmax = 90;

int vmini = 120;
int vmaxi = 190;

int veladelante = 190;
int velatras = 110;

int vbase;
unsigned long startTime;
const unsigned long changeDelay = 2000;

void setup() {
  startTime = millis();
  pinMode(der1, OUTPUT);
  pinMode(der2, OUTPUT);
  pinMode(izq1, OUTPUT);
  pinMode(izq2, OUTPUT);

 //Serial.begin(9600);
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

  digitalWrite(LED_ON, HIGH);
  digitalWrite(LED, 1);

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
  promedio();
  tone(PINBUZZER, 2000, 100);
  delay(200); 
  while (digitalRead(BOTON));
  tone(PINBUZZER, 2000, 100);
  delay(200); 
  digitalWrite(LED, 0);

}

void loop() {
    unsigned long currentTime = millis();
  if (currentTime - startTime >= changeDelay) {
    vmin = vmini;
    vmax = vmaxi;
  }
  digitalWrite(STBY,HIGH);
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
    motores(veladelante, -velatras);
  }
  if (pos >= 600){
    digitalWrite(STBY,HIGH);
    motores(-velatras, veladelante);
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