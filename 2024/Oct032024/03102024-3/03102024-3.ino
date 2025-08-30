

  

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







  if(black >= 4){
    motores(-20, -20);
    delay(1000);
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
  black = 0;
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
    motores(fxmax, -fxmin); 
  } 
  if (pos >= 600){ 
    digitalWrite(STBY,HIGH); 
    motores(-fxmin, fxmax); 
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