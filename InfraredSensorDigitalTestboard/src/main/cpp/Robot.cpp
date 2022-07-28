// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

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
  if (m_leftInfraredSensor.Get() && m_rightInfraredSensor.Get()) {
    kSeeTwo = true;
    kSeeLeft = false;
    kSeeRight = false;
    kSeeNone = false;
  } else if ((m_leftInfraredSensor.Get() && !m_rightInfraredSensor.Get())) {
    kSeeLeft = true;
    kSeeRight = false;
    kSeeTwo = false;
    kSeeNone = false;
  } else if ((!m_leftInfraredSensor.Get() && m_rightInfraredSensor.Get())) {
    kSeeRight = true;
    kSeeLeft = false;
    kSeeTwo = false;
    kSeeNone = false;
  } else {
    kSeeNone = true;
    kSeeLeft = false;
    kSeeRight = false;
    kSeeTwo = false;
  }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

bool Robot::IsLeftSensorBlock() { return !m_leftInfraredSensor.Get();  }
bool Robot::IsRightSensorBlock() { return !m_rightInfraredSensor.Get(); }

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
