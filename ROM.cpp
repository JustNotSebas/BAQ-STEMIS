#include "ROM.h"
#include "Config.h"
#include <EEPROM.h>

// --- Definiciones reales de direcciones ---
const int addrMagic  = 0;
const int addrKP     = addrMagic + sizeof(uint16_t);
const int addrKI     = addrKP + sizeof(float);
const int addrKD     = addrKI + sizeof(float);
const int addrKV     = addrKD + sizeof(float);
const int addrVmin   = addrKV + sizeof(float);
const int addrVmax   = addrVmin + sizeof(int);
const int addrFVup   = addrVmax + sizeof(int);
const int addrFVdown = addrFVup + sizeof(int);

// --- Guardar configuración ---
void saveEEPROM() {
  EEPROM.put(addrMagic, (uint16_t)EEPROM_MAGIC);

  EEPROM.put(addrKP,    KP);
  EEPROM.put(addrKI,    KI);
  EEPROM.put(addrKD,    KD);
  EEPROM.put(addrKV,    KV);
  EEPROM.put(addrVmin,  vmin);
  EEPROM.put(addrVmax,  vmax);
  EEPROM.put(addrFVup,  fvup);
  EEPROM.put(addrFVdown,fvdown);

  Serial.println("Configuración guardada en EEPROM.");
}

// --- Cargar configuración ---
void loadEEPROM() {
  uint16_t magic;
  EEPROM.get(addrMagic, magic);

  if (magic != EEPROM_MAGIC) {
    Serial.println("EEPROM vacía, usando valores por defecto...");
    resetConfig();
    saveEEPROM();
    return;
  }

  EEPROM.get(addrKP,    KP);
  EEPROM.get(addrKI,    KI);
  EEPROM.get(addrKD,    KD);
  EEPROM.get(addrKV,    KV);
  EEPROM.get(addrVmin,  vmin);
  EEPROM.get(addrVmax,  vmax);
  EEPROM.get(addrFVup,  fvup);
  EEPROM.get(addrFVdown,fvdown);

  Serial.println("Configuración cargada desde EEPROM:");
  Serial.print("KP=");    Serial.println(KP, 6);
  Serial.print("KI=");    Serial.println(KI, 6);
  Serial.print("KD=");    Serial.println(KD, 6);
  Serial.print("KV=");    Serial.println(KV, 6);
  Serial.print("vmin=");  Serial.println(vmin);
  Serial.print("vmax=");  Serial.println(vmax);
  Serial.print("fvup=");  Serial.println(fvup);
  Serial.print("fvdown=");Serial.println(fvdown);
}

// --- Resetear configuración a valores por defecto ---
void resetConfig() {
  KP = KPDF;
  KI = KIDF;
  KD = KDDF;
  KV = KVDF;
  vmin = VMINDF;
  vmax = VMAXDF;
  fvup = FVUPDF;
  fvdown = FVDOWNDF;

  Serial.println("Configuración reseteada a valores por defecto.");
}

// --- Procesamiento de comandos ---
void processCommand(String cmd) {
  if (cmd.startsWith("set ")) {
    int eq = cmd.indexOf('=');
    if (eq > 0) {
      String param = cmd.substring(4, eq);
      String valStr = cmd.substring(eq + 1);
      float val = valStr.toFloat();

      bool updated = true;

      if (param == "KP") KP = val;
      else if (param == "KI") KI = val;
      else if (param == "KD") KD = val;
      else if (param == "KV") KV = val;
      else if (param == "vmin") vmin = (int)val;
      else if (param == "vmax") vmax = (int)val;
      else if (param == "fvup") fvup = (int)val;
      else if (param == "fvdown") fvdown = (int)val;
      else updated = false;

      if (updated) {
        Serial.print("Parámetro actualizado: ");
        Serial.print(param);
        Serial.print(" = ");
        Serial.println(val, 6);
      } else {
        Serial.print("Parámetro desconocido: ");
        Serial.println(param);
      }
    }
  } else if (cmd == "save") {
    saveEEPROM();
  } else if (cmd == "load") {
    loadEEPROM();
  } else if (cmd == "reset") {
    resetConfig();
    saveEEPROM();
  } else {
    Serial.println("Comando no reconocido.");
  }
}

// --- Lectura no bloqueante de Serial ---
void readSerial() {
  static String buffer = "";

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      buffer.trim();
      if (buffer.length() > 0) {
        processCommand(buffer);
      }
      buffer = "";
    } else {
      buffer += c;
    }
  }
}
