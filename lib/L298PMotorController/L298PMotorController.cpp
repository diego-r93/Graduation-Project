#include "L298PMotorController.h"

L298PMotorController::L298PMotorController(int latchPin, int clockPin, int dataPin, int enablePinA, int enablePinB, int pwmChannelA, int pwmChannelB)
    : ShiftRegister74HC595(latchPin, clockPin, dataPin), enablePinA(enablePinA), enablePinB(enablePinB), pwmChannelA(pwmChannelA), pwmChannelB(pwmChannelB), motorStates(0) {}

void L298PMotorController::begin() {
   ShiftRegister74HC595::begin();
   pinMode(enablePinA, OUTPUT);
   pinMode(enablePinB, OUTPUT);

   // Configurando PWM para os pinos de ENABLE
   ledcSetup(pwmChannelA, 15000, 8);
   ledcSetup(pwmChannelB, 15000, 8);
   ledcAttachPin(enablePinA, pwmChannelA);
   ledcAttachPin(enablePinB, pwmChannelB);
}

void L298PMotorController::setMotor1(bool isOn) {
   if (isOn) {
      motorStates |= 0b00000100;  // Liga os pinos IN1 e IN2 (Q3 e Q2)
   } else {
      motorStates &= 0b11110011;  // Desliga os pinos IN1 e IN2
   }
   updateShiftRegister(motorStates);
}

void L298PMotorController::setMotor2(bool isOn) {
   if (isOn) {
      motorStates |= 0b00000001;  // Liga os pinos IN3 e IN4
   } else {
      motorStates &= 0b11111100;  // Desliga os pinos IN3 e IN4
   }
   updateShiftRegister(motorStates);
}

void L298PMotorController::setSpeed(int motor, int speed) {
   if (motor == 1) {
      ledcWrite(pwmChannelA, speed);
   } else if (motor == 2) {
      ledcWrite(pwmChannelB, speed);
   }
}
