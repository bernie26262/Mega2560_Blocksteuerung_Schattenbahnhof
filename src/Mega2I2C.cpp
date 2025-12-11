#include "Mega2I2C.h"
#include "mega2_pins.h"
#include "proto_mega2.h"

extern Mega2Payload g_payload;

void megaI2C_begin()
{
    Wire.begin(0x12); // I2C-Adresse Mega2
    Wire.onRequest([]() {
        Wire.write((uint8_t*)&g_payload, sizeof(g_payload));
    });

    pinMode(PIN_DATA_READY_M2, OUTPUT);
    digitalWrite(PIN_DATA_READY_M2, LOW);
}

void megaI2C_update()
{
    digitalWrite(PIN_DATA_READY_M2, HIGH);
    delayMicroseconds(200);
    digitalWrite(PIN_DATA_READY_M2, LOW);
}
