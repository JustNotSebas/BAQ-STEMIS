#include "Motors.h"

void motores(int izq, int der) {
  izq = constrain(izq, -255, 255);
  der = constrain(der, -255, 255);


  digitalWrite(izq1, izq >= 0); digitalWrite(izq2, izq < 0);
  analogWrite(pwma, abs(izq));

  digitalWrite(der1, der >= 0); digitalWrite(der2, der < 0);
  analogWrite(pwmb, abs(der));
}

void seguirRecto() {
  motores(vmin-20, vmin);
  delay(INTERSECCION_DELAY);
}

void girar(int dir) {
  // dir = 1 (derecha), dir = -1 (izquierda)
  if (dir == 1) motores(110, -110);
  else motores(-70, 150);
  delay(times);
  pararMotores();
}

void pararMotores() { 
  motores(0, 0); 
  delay(80); 
}

void frenos(float pos) {
  if (pos <= 120) motores(fvup, -fvdown);
  else if (pos >= 580) motores(-fvdown, fvup);
}
