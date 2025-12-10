#pragma once
#include <Arduino.h>
#include <Wire.h>

// Mega2 DataReady-Pin & I2C-Adresse kommen aus config.h
#include "config.h"

// Initialisiert den I2C-Slave
void megaI2C_begin();

// Führt DataReady-Handling aus
void megaI2C_update();
