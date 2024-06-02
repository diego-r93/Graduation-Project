#ifndef L298PMOTORCONTROLLER_H
#define L298PMOTORCONTROLLER_H

#include <Arduino.h>
#include "ShiftRegister74HC595.h"

class L298PMotorController : public ShiftRegister74HC595 {
public:
    L298PMotorController(int latchPin, int clockPin, int dataPin, int enablePinA, int enablePinB, int pwmChannelA, int pwmChannelB);
    void begin();
    void setMotor1(bool isOn);
    void setMotor2(bool isOn);
    void setSpeed(int motor, int speed);

private:
    int enablePinA;
    int enablePinB;
    int pwmChannelA;
    int pwmChannelB;
    byte motorStates;
};

#endif
