// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>

#include "Constants.hpp"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
    frc::Joystick joystick {joystickPort};
    rev::CANSparkMax motor {motorID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANEncoder encoder = motor.GetEncoder();
    frc::Solenoid solenoid {solenoidPort};
};
