#include "PIDControl.h"
#include <math.h> // Para exp()

void calcularPID(float pos) {
  float error = pos - setpoint;
  PID_integral += error;

  // Anti-windup: limitar la integral
  if (PID_integral > 255) PID_integral = 255;
  else if (PID_integral < -255) PID_integral = -255;

  // Derivativo filtrado
  PID_derivative = 0.7 * (error - PID_prev_error) + 0.3 * PID_derivative;
  PID_prev_error = error;

  float salida = KP * error + KI * PID_integral + KD * PID_derivative;

  vbase = vmin + (vmax - vmin) * exp(-KV * abs(KP * error));

  // Llamada al módulo de motores
  motores(vbase + salida, vbase - salida);
}
