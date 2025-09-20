#include "Intersections.h"

bool detectarInterseccion() {
  int activos = 0;
  for (int i = 0; i < 8; i++) {
    if (digital[i]) activos++;
  }
  return activos >= MIN_SENSORES_INTERSECCION;
}

void tomarDecision() {
  Serial.print("Intersección detectada. Sensores: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(digital[i]);
    Serial.print(" ");
  }
  Serial.println();

  // Giro derecha si los sensores 6 o 7 ven línea
  if (digital[6] || digital[7]) {
    Serial.println("Girando a la derecha...");
    girar(1);
  }
  // Giro izquierda si los sensores 0 o 1 ven línea
  else if (digital[0] || digital[1]) {
    Serial.println("Girando a la izquierda...");
    girar(-1);
  }
  // Si no, sigue recto por la intersección
  else {
    Serial.println("Sigue recto por la intersección.");
    seguirRecto();
  }
}
