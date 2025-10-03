#include "PIDControl.h"
#include <math.h> // Para exp()

void calcularPID(float pos) {
  // Calculate time delta
  unsigned long current_time = millis();
  float dt;
  
  // Initialize timing on first run
  if (PID_last_time == 0) {
    PID_last_time = current_time;
    dt = 0.020; // Assume 20ms as default dt on first run
  } else {
    dt = (current_time - PID_last_time) / 1000.0; // Convert to seconds
    
    // Sanity check: if dt is unreasonable, use default
    if (dt <= 0 || dt > 0.5) {
      dt = 0.020;
    }
  }
  PID_last_time = current_time;
  
  // Calculate error
  float error = pos - setpoint;
  
  // Proportional term
  float P_term = KP * error;
  
  // Integral term with time-based accumulation
  PID_integral += error * dt;
  
  // Calculate preliminary output to determine integral limits
  float max_integral_contribution = PID_output_limit / 2.0;  // Reserve half output range for integral
  float integral_limit = (KI != 0) ? max_integral_contribution / KI : 1000.0;
  
  // Anti-windup: limit integral based on output range
  if (PID_integral > integral_limit) {
    PID_integral = integral_limit;
  } else if (PID_integral < -integral_limit) {
    PID_integral = -integral_limit;
  }
  
  float I_term = KI * PID_integral;
  
  // Derivative term with time-based calculation and filtering
  float derivative_raw = (dt > 0) ? (error - PID_prev_error) / dt : 0;
  
  // Low-pass filter on derivative to reduce noise
  PID_derivative = DERIVATIVE_FILTER_COEF * derivative_raw + 
                   (1.0 - DERIVATIVE_FILTER_COEF) * PID_derivative;
  
  float D_term = KD * PID_derivative;
  
  // Update previous error for next iteration
  PID_prev_error = error;
  
  // Calculate total PID output
  float salida = P_term + I_term + D_term;
  
  // Limit PID output to prevent saturation
  if (salida > PID_output_limit) {
    salida = PID_output_limit;
  } else if (salida < -PID_output_limit) {
    salida = -PID_output_limit;
  }
  
  // Calculate base velocity with exponential decay on curves
  vbase = vmin + (vmax - vmin) * exp(-KV * abs(error) / 100.0);
  
  // Calculate motor speeds
  int motor_left = vbase + salida - MOTOR_OFFSET;
  int motor_right = vbase - salida;
  
  // Send commands to motors (they will constrain values internally)
  motores(motor_left, motor_right);
}

void resetPID() {
  // Reset all PID state variables
  PID_integral = 0;
  PID_prev_error = 0;
  PID_derivative = 0;
  PID_last_time = 0;  // Force re-initialization on next call
}
