// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /** Robot mode enum. */
  public static enum RobotMode {
    ROBOT_2022_REAL,
    ROBOT_2022_SIM
  }

  /** Running mode enum. */
  public static enum RunningMode {
    REAL,
    REPLAY,
    SIM
  }

  /** The mode you want to run the robot in. Change it to reflect what you want to run it in. */
  private static final RobotMode robot = RobotMode.ROBOT_2022_REAL;

  /** Get the robot your code is running. */
  public static RobotMode getRobot() {
    if (RobotBase.isReal()) {
      if (robot == RobotMode.ROBOT_2022_SIM) {
        return RobotMode.ROBOT_2022_REAL;
      } else {
        return robot;
      }
    } else {
      return robot;
    }
  }

  /** Get the mode of what you're running the robot in. */
  public static RunningMode getMode() {
    switch (getRobot()) {
      case ROBOT_2022_REAL:
        return RobotBase.isReal() ? RunningMode.REAL : RunningMode.REPLAY;
      case ROBOT_2022_SIM:
        return RunningMode.SIM;
      default:
        return RunningMode.REAL;
    }
  }

  public static final int xboxControllerPort = 0;

  public static class Elevator {
    public static final int elevatorMotorID = 9;
    public static final int elevatorEncoderA = 4;
    public static final int elevatorEncoderB = 5;

    public static final double distancePerPulseFactor =
        (2.0 * Math.PI * (0.0363728 / 2.0)) * 2.0 / 2048.0;

    public static final double carriageMass = 9.78;
    public static final double drumRadius = 0.0363728 / 2.0;
    public static final double elevatorGearing = 1.0;

    public static final TrapezoidProfile.Constraints m_constraints =
        new TrapezoidProfile.Constraints(8.6, 4.3);
  }
}
