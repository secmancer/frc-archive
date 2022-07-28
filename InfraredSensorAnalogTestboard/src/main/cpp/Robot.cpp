// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>
#include <fmt/core.h>
#include <fmt/printf.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutBoolean("No Sensors Tripped", kSeeNone);
  frc::SmartDashboard::PutBoolean("Left Sensor Tripped", kSeeLeft);
  frc::SmartDashboard::PutBoolean("Right Sensor Tripped", kSeeRight);
  frc::SmartDashboard::PutBoolean("Two Sensors Tripped", kSeeTwo);
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  
  if (m_leftInfraredSensor.GetValue() > 210 && m_rightInfraredSensor.GetValue() > 210) {
    kSeeTwo = true;
    kSeeLeft = false;
    kSeeRight = false;
    kSeeNone = false;
  }  else if (m_leftInfraredSensor.GetValue() < 10 && m_rightInfraredSensor.GetValue() < 10) {
    kSeeNone = true;
    kSeeLeft = false;
    kSeeRight = false;
    kSeeTwo = false;
  } else if (m_leftInfraredSensor.GetValue() > m_rightInfraredSensor.GetValue()) {
    kSeeLeft = true;
    kSeeRight = false;
    kSeeTwo = false;
    kSeeNone = false;
  } else if (m_leftInfraredSensor.GetValue() < m_rightInfraredSensor.GetValue()) {
    kSeeRight = true;
    kSeeLeft = false;
    kSeeTwo = false;
    kSeeNone = false;
  }

  std::cout << "Left Sensor: " << m_leftInfraredSensor.GetValue() << std::endl;
  std::cout << "Right Sensor: " << m_rightInfraredSensor.GetValue() << std::endl;
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
