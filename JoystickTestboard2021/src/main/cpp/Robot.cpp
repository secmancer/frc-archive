#include "Robot.h"
#include <iostream>
#include <units/math.h>

void Robot::RobotInit() {
    encoder.SetVelocityConversionFactor(wpi::math::pi * 2.0 / 60.0);
}

void Robot::RobotPeriodic() {
    std::cout << "Motor Velocity: " << encoder.GetVelocity() << std::endl;
    std::cout << "Motor Temperature: " << motor.GetMotorTemperature() << std::endl;
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

   if (joystick.GetRawButtonPressed(3)) {
       motor.Set(0);
   } else if (joystick.GetRawButtonPressed(4)) {
       motor.Set(0.2);
   } else if (joystick.GetRawButtonPressed(5)) {
       motor.Set(0.3);
   } else if (joystick.GetRawButtonPressed(6)) {
       motor.Set(0.4);
   } else if (joystick.GetRawButtonPressed(7)) {
       motor.Set(0.5);
   } else if (joystick.GetRawButtonPressed(8)) {
       motor.Set(0.6);
   } else if (joystick.GetRawButtonPressed(9)) {
       motor.Set(0.7);
   } else if (joystick.GetRawButtonPressed(10)) {
       motor.Set(0.8);
   } else if (joystick.GetRawButtonPressed(11)) {
       motor.Set(0.9);
   } else if (joystick.GetRawButtonPressed(12)) {
       motor.Set(1.0);
   }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
