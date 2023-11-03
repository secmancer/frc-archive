package frc3512.lib.controllers;

public class FlywheelConstants {

  /** Flywheel physical robot placement. */
  static enum FlywheelPose {
    kFront,
    kBack
  }

  public static class FrontFlywheel {

    /// Static friction system ID gain.
    public static final double kS = 0.18296;

    /// Angular velocity system ID gain.
    public static final double kV = 0.020245;

    /// Angular acceleration system ID gain.
    public static final double kA = 0.003333;

    /// Maximum front flywheel angular velocity.
    public static final double kMaxAngularVelocity = 12 / kV;

    /// High goal speed for front shooter (fender, high goal shot)
    public static final double kShootHighFender = 359;

    /// High goal speed for front shooter (tarmac shot)
    public static final double kShootHighTarmac = 359;

    /// Low goal speed for front shooter (fender, low goal shot)
    public static final double kShootLow = 240;
  }

  public static class BackFlywheel {

    /// Static friction system ID gain
    public static final double kS = 0.070473;

    /// Angular velocity system ID gain.
    public static final double kV = 0.021752;

    /// Angular acceleration system ID gain.
    public static final double kA = 0.0095716;

    /// Maximum back flywheel angular velocity.
    public static final double kMaxAngularVelocity = 12 / kV;

    /// High goal speed for back shooter (high, fender shot)
    public static final double kShootHighFender = 100;

    /// High goal speed for back shooter (high, tarmac shot)
    public static final double kShootHighTarmac = 240;

    /// Low goal speed for back shooter.
    public static final double kShootLow = 290;
  }
}
