#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "Config.h"

void motores(int izq, int der);
void seguirRecto();
void girar(int dir);
void pararMotores();
void frenos(float pos);

#endif
