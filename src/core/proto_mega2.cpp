#include "proto_mega2.h"

size_t buildMega2StatusFrame(uint8_t* outBuf, const Mega2StatusPayload& p)
{
    ProtoHeader h;
    h.slaveId    = SlaveId::MEGA2;
    h.msgType    = MsgType::STATUS;
    h.payloadLen = sizeof(Mega2StatusPayload);

    // Header
    proto_writeHeader(outBuf, h);

    // Payload
    const uint8_t* payloadBytes = reinterpret_cast<const uint8_t*>(&p);

    const size_t headerSize = 5;
    for (size_t i = 0; i < sizeof(Mega2StatusPayload); ++i)
        outBuf[headerSize + i] = payloadBytes[i];

    // Checksumme
    size_t lenWithoutChecksum = headerSize + sizeof(Mega2StatusPayload);
    outBuf[lenWithoutChecksum] = proto_calcChecksum(outBuf, lenWithoutChecksum);

    return lenWithoutChecksum + 1;
}
