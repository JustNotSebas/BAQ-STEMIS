#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>   // Necesario para digitalRead, analogRead, delay
#include "Config.h"    // Para usar BOTON, LED y las variables globales de sensores

void esperarBoton();
void parpadeoLED();
void leerFondos();
void leerLineas();
void calcularUmbral();

#endif
