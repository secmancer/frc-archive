// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

// Implementation code by Adan Silva

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/units.h>
#include <wpi/math>

Robot::Robot(){
    encoder0.SetVelocityConversionFactor(wpi::math::pi * 2.0 / 60.0);
    encoder1.SetVelocityConversionFactor(wpi::math::pi * 2.0 / 60.0);

    controller0.SetP(0.5);
    controller1.SetP(0.5);
    controller0.SetI(0);
    controller1.SetI(0);
    controller0.SetD(0);
    controller1.SetD(0);
}
void Robot::RobotInit() {}

void Robot::RobotPeriodic() {
    std::cout << "Left Vel:" << encoder0.GetVelocity() << std::endl;
    std::cout << "Right Vel:" << encoder1.GetVelocity() << std::endl;
    std::cout << "Left Temp: " << spark0.GetMotorTemperature() << std::endl;
    std::cout << "Left Temp: " << spark1.GetMotorTemperature() << std::endl;
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
    constexpr units::radians_per_second_t kMaxSpeed = 558.904_rad_per_s;

    // Spark Max # 0
    /*if (joystick0.GetRawButtonPressed(5)) {
        reference = kMaxSpeed * 0.1;
    } else if (joystick0.GetRawButtonPressed(6)) {
        reference = kMaxSpeed * 0.2;
    } else if (joystick0.GetRawButtonPressed(3)) {
        reference = kMaxSpeed * 0.3;
    } else if (joystick0.GetRawButtonPressed(4)) {
        reference = kMaxSpeed * 0.4;
    } else if (joystick0.GetRawButtonPressed(7)) {
        reference = kMaxSpeed * 0.5;
    } else if (joystick0.GetRawButtonPressed(8)) {
        reference = kMaxSpeed * 0.6;
    } else if (joystick0.GetRawButtonPressed(9)) {
        reference = kMaxSpeed * 0.7;
    } else if (joystick0.GetRawButtonPressed(10)) {
        reference = kMaxSpeed * 0.8;
    } else if (joystick0.GetRawButtonPressed(11)) {
        reference = kMaxSpeed * 0.9;
    } else if (joystick0.GetRawButtonPressed(12)) {
        reference = kMaxSpeed;
    } else if (joystick0.GetRawButtonPressed(2)) {
        reference = 0_rpm;
    } else if (joystick0.GetRawButtonPressed(1)) {
        reference = kMaxSpeed * 0.65;
    } 

    controller0.SetReference(
        units::radians_per_second_t(reference).to<double>(),
        rev::ControlType::kVelocity, reference / kMaxSpeed * 12.0);

    controller1.SetReference(
        units::radians_per_second_t(-reference).to<double>(),
        rev::ControlType::kVelocity, -reference / kMaxSpeed * 12.0); */
    
     if (joystick0.GetRawButtonPressed(5)){ 
      spark0.Set(0.1);
      spark1.Set(-.1);
     }
     else if (joystick0.GetRawButtonPressed(6)) {
       spark0.Set(0.2);
       spark1.Set(-.2);
     }
     else if (joystick0.GetRawButtonPressed(3)) {
              spark0.Set(0.3);
       spark1.Set(-.3);
     }
     else if (joystick0.GetRawButtonPressed(4)) {
              spark0.Set(0.4);

       spark1.Set(-.4);
     }
     else if (joystick0.GetRawButtonPressed(7)) {
              spark0.Set(0.5);

       spark1.Set(-.5);
     }
     else if (joystick0.GetRawButtonPressed(8)) {
              spark0.Set(0.6);

       spark1.Set(-.6);
     }
     else if (joystick0.GetRawButtonPressed(9)) {
              spark0.Set(0.7);

       spark1.Set(-.7);
     }
     else if (joystick0.GetRawButtonPressed(10)) {
              spark0.Set(0.8);

       spark1.Set(-.8);
     }
     else if (joystick0.GetRawButtonPressed(11)) {
              spark0.Set(0.55);

       spark1.Set(-0.55);
     } else if (joystick0.GetRawButtonPressed(12)) {
              spark0.Set(0.65);
       spark1.Set(-0.65);
     } else if (joystick0.GetRawButtonPressed(2)) {
       spark0.Set(0);
       spark1.Set(0);
     } 
    /*//Spark Max #1
     if (joystick0.GetRawButtonPressed(5))
       spark1.Set(-.1);
     else if (joystick0.GetRawButtonPressed(6))
       spark1.Set(-.2);
     else if (joystick0.GetRawButtonPressed(3))
       spark1.Set(-.3);
     else if (joystick0.GetRawButtonPressed(4))
       spark1.Set(-.4);
     else if (joystick0.GetRawButtonPressed(7))
       spark1.Set(-.5);
     else if (joystick0.GetRawButtonPressed(8))
       spark1.Set(-.6);
     else if (joystick0.GetRawButtonPressed(9))
       spark1.Set(-.7);
     else if (joystick0.GetRawButtonPressed(10))
       spark1.Set(-.8);
     else if (joystick0.GetRawButtonPressed(11))
       spark1.Set(-1);
     if (joystick0.GetRawButtonPressed(12))
       spark1.Set(0);
    */
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
