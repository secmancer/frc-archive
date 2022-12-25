// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc3512.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int xboxControllerPort = 0;

  public static class Elevator {
    public static final int elevatorMotorID = 9;

    public static final double carriageMass = 8.381376;
    public static final double drumRadius = 0.0363728 / 2.0;
    public static final double elevatorGearing = 12.5;

    public static final TrapezoidProfile.Constraints m_constraints =
        new TrapezoidProfile.Constraints(8.6, 4.3);
  }
}
