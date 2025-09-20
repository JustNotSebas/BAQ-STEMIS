#include "Sensors.h"

float leerSensores() {
  float sumaPesos = 0, sumaDigital = 0;
  for (int i = 0; i < 8; i++) {
    sensores[i] = analogRead(A0 + i);
    digital[i]  = sensores[i] > umbral[i];
    sumaPesos   += (700 - 100 * i) * digital[i];
    sumaDigital += digital[i];
    //Serial.print(digital[i]); Serial.print(" ");
  }
  float pos = sumaDigital > 0 ? sumaPesos / sumaDigital : last_pos;
  last_pos  = pos;
  //Serial.print("Posicion: "); Serial.println(pos);
  return pos;
}
