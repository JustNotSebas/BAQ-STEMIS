#include "Utils.h"

void esperarBoton() {
  while (digitalRead(BOTON));
  delay(200);
}

void parpadeoLED() {
  digitalWrite(LED, LOW);
  delay(15);
  digitalWrite(LED, HIGH);
  delay(15);
}

void leerFondos() {
  for (int i = 0; i < 8; i++) {
    lectura_fondo[i] += analogRead(A0 + i) / 50.0;
  }
}

void leerLineas() {
  for (int i = 0; i < 8; i++) {
    lectura_linea[i] += analogRead(A0 + i) / 50.0;
  }
}

void calcularUmbral() {
  for (int i = 0; i < 8; i++) {
    umbral[i] = (lectura_fondo[i] + lectura_linea[i]) / 2.0;
  }
}
