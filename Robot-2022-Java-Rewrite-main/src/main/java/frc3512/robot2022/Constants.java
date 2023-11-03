package frc3512.robot2022;

/** Constants for the robot project * */
public final class Constants {

  /** Constants revolving around joysticks * */
  public static final class Joysticks {
    // Xbox Controller port
    public static final int kXboxControllerPort = 0;

    // Appendage joystick 1 port
    public static final int kAppendageStick1Port = 2;

    // Appendage joystick 2 port
    public static final int kAppendageStick2Port = 3;
  }

  /** Constants for the drivetrain subsystem * */
  public static final class Drivetrain {
    /** The left-to-right distance between the drivetrain wheels */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.27305 * 2.0;
    /** The front-to-back distance between the drivetrain wheels. */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.27305 * 2.0;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 10;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 20;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 1;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(283.00 + 180.00);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 11;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 21;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(140.97);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 12;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 22;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(246.47);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 13;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 23;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 4;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(238.89);
  }

  /** Constants for the intake subsystem * */
  public static final class Intake {
    /// Arm motor CAN ID
    public static final int kArmMotorID = 50;

    /// Mini arm motor CAN ID
    public static final int kMiniArmMotorID = 51;

    /// Arm solenoid channel
    public static final int kArmChannel = 4;
  }

  /** Constants for the conveyor subsystem * */
  public static final class Conveyor {
    /// Conveyor motor CAN ID
    public static final int kConveyorMotorID = 52;

    /// Lower proximity sensor digial input channel
    public static final int kLowerSensorChannel = 3;

    /// Upper proximity sensor digital input channel
    public static final int kUpperSensorChannel = 2;
  }

  /** Constants for the climber subsystem * */
  public static final class Climber {
    public static final int kLeftClimberID = 40;
    public static final int kRightClimberID = 41;

    public static final int kLeftMagneticSwitch = 0;
    public static final int kRightMagenticSwitch = 2;

    public static final int kClimberSolenoidChannel = 5;
  }

  /** Constants for the front flywheel subsystem * */
  public static final class FrontFlywheel {
    /// Front motor CAN ID
    public static final int kFrontMotorID = 30;

    /// Front encoder channel A
    public static final int kFrontEncoderA = 4;

    /// Front encoder channel B
    public static final int kFrontEncoderB = 5;
  }

  /** Constants for the back flywheel subsystem * */
  public static final class BackFlywheel {
    /// Back motor CAN ID
    public static final int kBackMotorID = 31;

    /// Back encoder channel A
    public static final int kBackEncoderA = 0;

    /// Back encoder channel B
    public static final int kBackEncoderB = 1;

    /// Angle solenoid channel
    public static final int kShooterSolenoidChannel = 6;
  }
}
