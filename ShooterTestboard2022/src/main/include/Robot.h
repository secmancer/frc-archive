// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AddressableLED.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include <frc/motorcontrol/MotorControllerGroup.h>

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

    // Flywheel
    rev::CANSparkMax m_flywheelLeader {1, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_flywheelFollower {2, rev::CANSparkMax::MotorType::kBrushless};
    frc::MotorControllerGroup m_flywheel {m_flywheelLeader, m_flywheelFollower};
    rev::CANSparkMax m_topRoller{3, rev::CANSparkMax::MotorType::kBrushless};

    // Conveyor
    rev::CANSparkMax m_conveyor1 {4, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_conveyor2 {5, rev::CANSparkMax::MotorType::kBrushless};
    // Intake
    rev::CANSparkMax m_intake {6, rev::CANSparkMax::MotorType::kBrushless};

    // Speed Values
    double m_speedFLEntry = 0.0;
    double m_speedTREntry = 0.0;
    double m_speedC1Entry = 0.0;
    double m_speedC2Entry = 0.0;
    double m_speedIEntry = 0.0;

    // LED Lights
    static constexpr int kLength = 60;

    // LED Strip
    frc::Spark m_led{9};

    // LED lights
    double light = 0.0;
};
