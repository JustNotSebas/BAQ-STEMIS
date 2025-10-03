#ifndef PIDCONTROL_H
#define PIDCONTROL_H

#include <Arduino.h>
#include "Config.h"
#include "Motors.h"  // Para llamar a motores() desde aquí

void calcularPID(float pos);
void resetPID();  // Reset PID state (integral, derivative, etc.)

#endif
