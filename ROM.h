#ifndef ROM_H
#define ROM_H

#include <Arduino.h>
#include <EEPROM.h>

// Dirección base mágica
#define EEPROM_MAGIC 0x42

// --- Direcciones en memoria EEPROM ---
extern const int addrMagic;
extern const int addrKP;
extern const int addrKI;
extern const int addrKD;
extern const int addrKV;
extern const int addrVmin;
extern const int addrVmax;
extern const int addrFVup;
extern const int addrFVdown;

// --- Funciones públicas ---
void saveEEPROM();
void loadEEPROM();
void resetConfig();
void readSerial();

#endif
