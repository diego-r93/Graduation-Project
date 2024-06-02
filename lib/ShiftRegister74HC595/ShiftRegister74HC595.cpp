#include "ShiftRegister74HC595.h"

ShiftRegister74HC595::ShiftRegister74HC595(int latchPin, int clockPin, int dataPin)
    : latchPin(latchPin), clockPin(clockPin), dataPin(dataPin), shiftData(0) {}

void ShiftRegister74HC595::begin() {
   pinMode(latchPin, OUTPUT);
   pinMode(clockPin, OUTPUT);
   pinMode(dataPin, OUTPUT);
}

void ShiftRegister74HC595::updateShiftRegister(byte data) {
   shiftData = data;
   digitalWrite(latchPin, LOW);
   shiftOut(dataPin, clockPin, MSBFIRST, shiftData);
   digitalWrite(latchPin, HIGH);
}