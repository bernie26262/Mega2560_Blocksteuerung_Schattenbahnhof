#pragma once
#include <Arduino.h>

// ======================================================================================
// Gemeinsame Protokollbasis für ESP und alle Mega-Slaves
// ======================================================================================

// Magic + Version -------------------------------------------------------------
static const uint8_t PROTO_MAGIC   = 0xA5;
static const uint8_t PROTO_VERSION = 0x01;

// Maximale Frame-Länge
static const uint8_t PROTO_MAX_FRAME_SIZE = 160;

// Slave-IDs -------------------------------------------------------------------
enum class SlaveId : uint8_t {
    MEGA1 = 1,
    MEGA2 = 2,
    // später z.B. BOOSTER = 3, LICHT = 4
};

// Nachrichtentypen ------------------------------------------------------------
enum class MsgType : uint8_t {
    STATUS  = 1,    // regulärer Status-Snapshot
    EVENT   = 2,    // optional
    COMMAND = 3     // ESP → Slave
};

// Statusflags im Payload
enum StatusFlags : uint8_t {
    STATUS_FLAG_NONE              = 0,
    STATUS_FLAG_READY             = 1 << 0,   // Slave vollständig initialisiert
    STATUS_FLAG_SNAPSHOT_COMPLETE = 1 << 1,   // Vollständiges Bild
    STATUS_FLAG_ERROR             = 1 << 2,   // interner Fehler
};

// globale Error- / Warnflags (Megaspezifische Fehler kommen im Payload)
enum ErrorFlags : uint8_t {
    ERR_NONE          = 0,
    ERR_OVERCURRENT   = 1 << 0,
    ERR_SENSOR_FAULT  = 1 << 1,
    ERR_INTERNAL      = 1 << 2,
};

enum WarnFlags : uint8_t {
    WARN_NONE         = 0,
    WARN_NEAR_LIMIT   = 1 << 0,
};

// Struktur des Headers --------------------------------------------------------
struct ProtoHeader {
    uint8_t magic     = PROTO_MAGIC;
    uint8_t version   = PROTO_VERSION;
    SlaveId slaveId   = SlaveId::MEGA1;
    MsgType msgType   = MsgType::STATUS;
    uint8_t payloadLen= 0;
};

// Header in Buffer schreiben
inline void proto_writeHeader(uint8_t* buf, const ProtoHeader& h)
{
    buf[0] = h.magic;
    buf[1] = h.version;
    buf[2] = static_cast<uint8_t>(h.slaveId);
    buf[3] = static_cast<uint8_t>(h.msgType);
    buf[4] = h.payloadLen;
}

// Prüfsumme (Summe modulo 256)
inline uint8_t proto_calcChecksum(const uint8_t* data, size_t lenWithoutChecksum)
{
    uint16_t sum = 0;
    for (size_t i = 0; i < lenWithoutChecksum; ++i)
        sum += data[i];
    return (uint8_t)(sum & 0xFF);
}
