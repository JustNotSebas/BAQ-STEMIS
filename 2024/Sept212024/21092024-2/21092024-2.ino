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

// PID: Inicialización de variables
int error1 = 0, error2 = 0, error3 = 0, error4 = 0, error5 = 0, error6 = 0;
int proporcional = 0, integral = 0, derivativo = 0, diferencial = 0, last_prop;
int setpoint = 350;

// PID: Rendimiento del PID
const float KP = 0.11;
const float KD = 1;
const float KI = 0.006;
const float KV = 7;

// Velocidades
// Incio de la carrera (2000ms)
const unsigned long changeDelay = 2000; // Duración antes de cambiar los valores
int vmin = 30; // Velocidad minima
int vmax = 90; // Velocidad máxima
// Resto de la carrera
int vxmin = 120;
int vxmax = 190;
int fadelante = 190; // Frenos hacia delante
int fatras = 110; // Frenos hacia atras
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

  // Calibración secundaria (Segundo click)
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
  inf = (700*digital[0] + 600*digital[1] + 500*digital[2] + 400*digital[3] + 400*digital[4] + 500*digital[5] + 600*digital[6] + 700*digital[7]);
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

// Calculo del PID usado en marcha
void PID(){
  // La proporcional se calcula con la posición actual y la deseada
  proporcional = pos - setpoint;
  // La derivativa se calcula con la proporcional actual y la anterior
  derivativo = proporcional - last_prop;
  // La integral se calcula con los errores actuales detectados por el PID
  integral = error1 + error2 + error3 + error4 + error5 + error6;
  // Recalcular los errores, tomando como "error" la proporcional actual
  last_prop = proporcional;
  error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2;
  error2 = error1;
  error1 = proporcional;

  // Calcula el termino diferencial para manejar gradualmente la velocidad de los motores
  if((proporcional*integral)<0) integral = 0;
  diferencial = (proporcional*KP) + (derivativo*KD) + (integral*KI);
  // Calcula la velocidad base del vehiculo usando como referencia la proporcional
  vbase = vmin + (vmax-vmin) * exp(-KV * abs(KP*proporcional));
  // Mueve los motores
  motores(vbase+diferencial, vbase-diferencial);
}

// Frenos del vehiculo
// En caso de salirse de un valor especifico, el vehiculo se mueve al lado contrario
void frenos(){
  if (pos <= 100){
    digitalWrite(STBY,HIGH);
    motores(fadelante, -fatras);
  }
  if (pos >= 600){
    digitalWrite(STBY,HIGH);
    motores(-fatras, fadelante);
  }
}

// Motores
// Mueve los motores dependiendo de lo transferido de la función PID
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

// Se acabo