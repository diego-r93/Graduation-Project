#ifndef SHIFTREGISTER74HC595_H
#define SHIFTREGISTER74HC595_H

#include <Arduino.h>

class ShiftRegister74HC595 {
  public:
   ShiftRegister74HC595(int latchPin, int clockPin, int dataPin);
   void begin();
   void updateShiftRegister(byte data);

  private:
   int latchPin;
   int clockPin;
   int dataPin;
   byte shiftData;
};

#endif
