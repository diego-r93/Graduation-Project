#ifndef L298PMOTORCONTROLLER_H
#define L298PMOTORCONTROLLER_H

#include <Arduino.h>

class L298PMotorController {
  public:
   L298PMotorController(int enablePinA, int enablePinB, int pwmChannelA, int pwmChannelB);
   L298PMotorController(int enablePinA, int enablePinB, int pwmChannelA, int pwmChannelB,
                        std::function<void(bool)> motor1Func,
                        std::function<void(bool)> motor2Func);

   void begin();
   void setMotor1(bool isOn);
   void setMotor2(bool isOn);
   void setSpeed(int motor, int speed);

  private:
   int enablePinA;
   int enablePinB;
   int pwmChannelA;
   int pwmChannelB;
   std::function<void(bool)> motor1ControlFunc;
   std::function<void(bool)> motor2ControlFunc;
   bool useFunctions;
};

#endif
