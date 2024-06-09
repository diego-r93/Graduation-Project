#ifndef SHIFTREGISTER74HC595_H
#define SHIFTREGISTER74HC595_H

#include <Arduino.h>

class ShiftRegister74HC595 {
  public:
   ShiftRegister74HC595(int latchPin, int clockPin, int dataPin);
   void begin();
   void updateShiftRegister(byte data);

   // Funções para controlar os pinos específicos
   std::function<void(bool)> getControlFunctionForMotor1();
   std::function<void(bool)> getControlFunctionForMotor2();
   std::function<void(bool)> getControlFunctionForMotor3();
   std::function<void(bool)> getControlFunctionForMotor4();

  private:
   int latchPin;
   int clockPin;
   int dataPin;
   byte shiftData;
};

#endif
