#pragma once
#include <Arduino.h>

class PowerControl {
public:
    virtual ~PowerControl() {}

    virtual void setBlock5ToSBhf(bool on) = 0;
    virtual bool isBlock5ToSBhfActive() const = 0;
    virtual void setSbhfGleis(uint8_t gleis, bool on) = 0;
    virtual void setNothalt(bool on) = 0;
};
