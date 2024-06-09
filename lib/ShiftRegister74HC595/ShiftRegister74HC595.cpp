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

std::function<void(bool)> ShiftRegister74HC595::getControlFunctionForMotor1() {
   return [this](bool isOn) {
      if (isOn) {
         shiftData |= 0b00000100;  // Liga os pinos IN1 e IN2 (Q3 e Q2)
      } else {
         shiftData &= 0b11110011;  // Desliga os pinos IN1 e IN2
      }
      updateShiftRegister(shiftData);
   };
}

std::function<void(bool)> ShiftRegister74HC595::getControlFunctionForMotor2() {
   return [this](bool isOn) {
      if (isOn) {
         shiftData |= 0b00000001;  // Liga os pinos IN3 e IN4
      } else {
         shiftData &= 0b11111100;  // Desliga os pinos IN3 e IN4
      }
      updateShiftRegister(shiftData);
   };
}

std::function<void(bool)> ShiftRegister74HC595::getControlFunctionForMotor3() {
   return [this](bool isOn) {
      if (isOn) {
         shiftData |= 0b01000000;  // Liga os pinos IN5 e IN6
      } else {
         shiftData &= 0b00111111;  // Desliga os pinos IN5 e IN6
      }
      updateShiftRegister(shiftData);
   };
}

std::function<void(bool)> ShiftRegister74HC595::getControlFunctionForMotor4() {
   return [this](bool isOn) {
      if (isOn) {
         shiftData |= 0b00010000;  // Liga os pinos IN7 e IN8
      } else {
         shiftData &= 0b11001111;  // Desliga os pinos IN7 e IN8
      }
      updateShiftRegister(shiftData);
   };
}