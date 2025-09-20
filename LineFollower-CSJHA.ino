#include "Config.h"
#include "Utils.h"
#include "Sensors.h"
#include "PIDControl.h"
#include "Motors.h"
#include "Intersections.h"

void setup() {
  // --- Pines ---
  pinMode(der1, OUTPUT); pinMode(der2, OUTPUT);
  pinMode(izq1, OUTPUT); pinMode(izq2, OUTPUT);
  pinMode(pwma, OUTPUT); pinMode(pwmb, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(LED, OUTPUT); pinMode(LED_ON, OUTPUT);
  pinMode(BOTON, INPUT_PULLUP);
  pinMode(PINBUZZER, OUTPUT);

  for (int i = 0; i < 8; i++) pinMode(A0 + i, INPUT);

  Serial.begin(115200);
  digitalWrite(LED_ON, HIGH);

  // --- Calibración ---
  digitalWrite(LED, HIGH);
  Serial.println("Coloca los sensores sobre fondo y presiona el botón...");
  esperarBoton();
  for(int i = 0; i < 50; i++) { leerFondos(); parpadeoLED(); }
  tone(PINBUZZER, 2000, 80);

  Serial.println("Coloca los sensores sobre línea y presiona el botón...");
  esperarBoton();
  for(int i = 0; i < 50; i++) { leerLineas(); parpadeoLED(); }
  tone(PINBUZZER, 2000, 80);

  calcularUmbral();
  digitalWrite(LED, LOW);
  digitalWrite(STBY, HIGH);
  Serial.println("¡Listo para iniciar! Presiona el botón para comenzar.");
  esperarBoton();
}

void loop() {
  float pos = leerSensores();

  if (detectarInterseccion()) {
    if (!en_interseccion) {
      en_interseccion = true;
      tiempo_interseccion = millis();
      pararMotores();
      tomarDecision();
    }
  } else {
    en_interseccion = false;
    calcularPID(pos);
    frenos(pos);
  }
}
