#include "Mega2I2C.h"

#include <Wire.h>
#include "config.h"

#include "core/proto_common.h"
#include "core/proto_mega2.h"

extern Mega2StatusPayload g_payload;
extern volatile bool g_payloadDirty;

static void onI2CRequest();

void megaI2C_begin() {
    pinMode(PIN_I2C_INT, OUTPUT);
    digitalWrite(PIN_I2C_INT, LOW);

    Wire.begin(I2C_SLAVE_ADDR);
    Wire.onRequest(onI2CRequest);
}

void megaI2C_update() {
    digitalWrite(PIN_I2C_INT, g_payloadDirty ? HIGH : LOW);
}

static void onI2CRequest() {
    uint8_t buffer[PROTO_MAX_FRAME_SIZE];
    size_t len = buildMega2StatusFrame(buffer, g_payload);

    Wire.write(buffer, len);

    g_payloadDirty = false;
    digitalWrite(PIN_I2C_INT, LOW);
}
