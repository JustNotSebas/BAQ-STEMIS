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
// Motores: Izquierda #1
#define pwma  5
#define izq1  9
#define izq2  10 
// Motores: Derecha #2
#define pwmb  11
#define der1  7
#define der2  6
// Motores: Izquierda #3
//#define pwmc  x
//#define izq3
//#define izq4
// Motores: Derecha #4
//#define pwmd  z
//#define der3
//#define der4
// Pines varios
#define LED_ON   13
#define LED       3
#define BOTON     2
#define STBY  8

// Variables
// Inicialización de variables de calibración / posición
int digital[8], sensor[8], calc_fondo[8], calc_linea[8], umbral[8];
long int inf, act, post, lastpost, position;
unsigned long startTime;

// PID: Inicialización de variables
int ant1 = 0, ant2 = 0, ant3 = 0, ant4 = 0, ant5 = 0, ant6 = 0;
int proporcional = 0, integral = 0, derivativo = 0, diferencial = 0, last_prop;
int setpoint = 350;

// PID: Rendimiento del PID
const float KP = 0.145;
const float KD = 1.01;
const float KI = 0.0085;
const float KV = 10;

// Velocidades
// Incio de la carrera (2000ms)
const unsigned long changeDelay = 2000; // Duración antes de cambiar los valores
int vmin = 80; // Velocidad minima
int vmax = 255; // Velocidad máxima
// Resto de la carrera
int vxmin = 170;
int vxmax = 255;
int fadelante = 140; // Frenos hacia delante
int fatras = 100; // Frenos hacia atras
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

  // Calibración inicial (Primer click)
  while (digitalRead(BOTON));
  delay(200); 
  for(int i = 0; i < 50; i++){
    fondos();
    digitalWrite(LED, 0);
    delay(20);
    digitalWrite(LED, 1);
    delay(20);
  }
  delay(200);

  // Calibración secundaria (Segundo click)
  while (digitalRead(BOTON));
  delay(200); 
  for(int i = 0; i < 50; i++){
    lineas();
    digitalWrite(LED, 0);
    delay(20);
    digitalWrite(LED, 1);
    delay(20);
  }

  promedio(); // Promediación de valores
  delay(200); 

  // Arranque de los motores (Tercer click)
  while (digitalRead(BOTON));
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
    if (sensor[i]<= umbral[i]){
      digital[i] = 0;
    }
    else{digital[i] =1;}
 // Serial.print(digital[i]), Serial.print("\t"); // Para depuración: Imprime los valores de los 8 sensores 0 = Blanco & 1 = Negro
  }
  //Serial.println(" "); 

  // Calcula la influencia que tiene cada sensor en el calculo final

  inf = (700*digital[0] + 600*digital[1] + 500*digital[2] + 400*digital[3] + 300*digital[4] + 200*digital[5] + 100*digital[6] + 0*digital[7]);
  // Calcula el promedio actual de blanco/negro tomado por los sensores
  act = (digital[0] + digital[1] + digital[2] + digital[3] + digital[4] + digital[5] + digital[6] + digital[7]);
  // Divide la influencia y el promedio, dando una posición promedio
  post = (inf / act);

  // En caso de que se salga de ciertos limites, forzar al carro a utilizar los frenos
  if(lastpost <= 100 && post == -1){
    post = 0;
  }
  if(lastpost >= 600 && post == -1){
    post = 700;
  }
  
  lastpost = post;
  return post;
}

// Calculo del PID usado en marcha
void PID(){
  // La proporcional se calcula con la posición actual y la deseada
  proporcional = post - setpoint;
  // La derivativa se calcula con la proporcional actual y la anterior
  derivativo = proporcional - last_prop;
  // La integral se calcula con los errores actuales detectados por el PID
  integral = ant1 + ant2 + ant3 + ant4 + ant5 + ant6;
  // Recalcular los errores, tomando como "error" la proporcional actual
  last_prop = proporcional;
  ant6 = ant5;
  ant5 = ant4;
  ant4 = ant3;
  ant3 = ant2;
  ant2 = ant1;
  ant1 = proporcional;

  // Calcula el termino diferencial para manejar gradualmente la velocidad de los motores
  if((proporcional*integral)<0) integral = 0;
  diferencial = (proporcional*KP) + (derivativo*KD) + (integral*KI);
  // Calcula la velocidad base del vehiculo usando como referencia la proporcional
  vbase = vmin + (vmax-vmin) * exp(-KV * abs(KP*proporcional));
  // Mueve los motores
  motores1(vbase+diferencial, vbase-diferencial);
  //motores2(vbase+diferencial, vbase-diferencial);
}

// Frenos del vehiculo
// En caso de salirse de un valor especifico, el vehiculo se mueve al lado contrario
void frenos(){
  if (post <= 100){
    digitalWrite(STBY,HIGH);
    motores1(fadelante, -fatras);
  }
  if (post >= 600){
    digitalWrite(STBY,HIGH);
    motores1(-fatras, fadelante);
  }
}

// Motores
// Mueve los motores dependiendo de lo transferido de la función PID
void motores1(int izq, int der){
  izq = constrain(izq, -255, 255);
  der = constrain(der, -255, 255);

  digitalWrite(izq2, izq<0);
  digitalWrite(izq1, izq>=0); 
  analogWrite(pwma, abs(izq));

  digitalWrite(der2, der<0);
  digitalWrite(der1, der>=0);
  analogWrite(pwmb, abs(der));
  
}

// void motores2(int izq, int der){
//  izq = constrain(izq, -255, 255);
//  der = constrain(der, -255, 255);

//  digitalWrite(izq4, izq<0);
//  digitalWrite(izq3, izq>=0); 
//  analogWrite(pwmc, abs(izq));

//  digitalWrite(der4, der<0);
//  digitalWrite(der3, der>=0);
//  analogWrite(pwmd, abs(der));
  
//}
// Se acabo