#include "L298PMotorController.h"

L298PMotorController::L298PMotorController(int enablePinA, int enablePinB, int pwmChannelA, int pwmChannelB)
    : enablePinA(enablePinA), enablePinB(enablePinB), pwmChannelA(pwmChannelA), pwmChannelB(pwmChannelB), useFunctions(false) {}

L298PMotorController::L298PMotorController(int enablePinA, int enablePinB, int pwmChannelA, int pwmChannelB,
                                           std::function<void(bool)> motor1Func, std::function<void(bool)> motor2Func)
    : enablePinA(enablePinA), enablePinB(enablePinB), pwmChannelA(pwmChannelA), pwmChannelB(pwmChannelB), motor1ControlFunc(motor1Func), motor2ControlFunc(motor2Func), useFunctions(true) {}

void L298PMotorController::begin() {
   pinMode(enablePinA, OUTPUT);
   pinMode(enablePinB, OUTPUT);
   ledcSetup(pwmChannelA, 20000, 8);
   ledcSetup(pwmChannelB, 20000, 8);
   ledcAttachPin(enablePinA, pwmChannelA);
   ledcAttachPin(enablePinB, pwmChannelB);
}

void L298PMotorController::setMotor1(bool isOn) {
   if (useFunctions && motor1ControlFunc) {
      motor1ControlFunc(isOn);
   } else {
      digitalWrite(enablePinA, isOn ? HIGH : LOW);
   }
}

void L298PMotorController::setMotor2(bool isOn) {
   if (useFunctions && motor2ControlFunc) {
      motor2ControlFunc(isOn);
   } else {
      digitalWrite(enablePinB, isOn ? HIGH : LOW);
   }
}

void L298PMotorController::setSpeed(int motor, int speed) {
   if (motor == 1) {
      ledcWrite(pwmChannelA, speed);
   } else if (motor == 2) {
      ledcWrite(pwmChannelB, speed);
   }
}
