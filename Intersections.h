#ifndef INTERSECTIONS_H
#define INTERSECTIONS_H

#include <Arduino.h>
#include "Config.h"
#include "Motors.h"   // Para llamar a girar() y seguirRecto()

bool detectarInterseccion();
void tomarDecision();

#endif
