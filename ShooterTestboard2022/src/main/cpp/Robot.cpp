// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <wpi/numbers>

void Robot::RobotInit() { 
  frc::SmartDashboard::PutNumber("Flywheel Speed", m_speedFLEntry);
  frc::SmartDashboard::PutNumber("Top Roller Speed", m_speedTREntry);
  frc::SmartDashboard::PutNumber("Conveyor Left Speed", m_speedC1Entry);
  frc::SmartDashboard::PutNumber("Conveyor Right Speed", m_speedC2Entry);
  frc::SmartDashboard::PutNumber("Intake Speed", m_speedIEntry);
  frc::SmartDashboard::PutNumber("LED Light", light);
}

void Robot::RobotPeriodic() {
  m_speedFLEntry = frc::SmartDashboard::GetNumber("Flywheel Speed", 0.0);
  m_speedTREntry = frc::SmartDashboard::GetNumber("Top Roller Speed", 0.0);
  m_speedC1Entry = frc::SmartDashboard::GetNumber("Conveyor Left Speed", 0.0);
  m_speedC2Entry = frc::SmartDashboard::GetNumber("Conveyor Right Speed", 0.0);
  m_speedIEntry = frc::SmartDashboard::GetNumber("Intake Speed", 0.0);
  light = frc::SmartDashboard::GetNumber("LED Light", 0.0);
  m_led.Set(light);
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {
  m_flywheel.Set(0.0);
  m_topRoller.Set(0.0);
  m_conveyor1.Set(0.0);
  m_conveyor2.Set(0.0);
  m_intake.Set(0.0);
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
  m_flywheel.Set(0.0);
  m_topRoller.Set(0.0);
  m_conveyor1.Set(0.0);
  m_conveyor2.Set(0.0);
  m_intake.Set(0.0);
}

void Robot::TestPeriodic() {
  m_flywheel.Set(m_speedFLEntry);
  m_topRoller.Set(m_speedTREntry);
  m_conveyor1.Set(m_speedC1Entry);
  m_conveyor2.Set(m_speedC2Entry);
  m_intake.Set(m_speedIEntry);
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
