package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends CommandBase {

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;

  private Swerve s_Swerve;
  private Joystick controller;
  private int translationAxis;
  private int strafeAxis;
  private int rotationAxis;

  /** Driver control */
  public TeleopSwerve(
      Swerve s_Swerve,
      Joystick controller,
      int translationAxis,
      int strafeAxis,
      int rotationAxis,
      boolean fieldRelative,
      boolean openLoop) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.controller = controller;
    this.translationAxis = translationAxis;
    this.strafeAxis = strafeAxis;
    this.rotationAxis = rotationAxis;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
  }

  @Override
  public void execute() {
    double xAxis =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(controller.getRawAxis(translationAxis), 0.1));
    double yAxis =
        -m_yspeedLimiter.calculate(
            MathUtil.applyDeadband(controller.getRawAxis(strafeAxis), 0.1));
    double rAxis =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(controller.getRawAxis(rotationAxis), 0.1));

    translation = new Translation2d(xAxis, yAxis).times(Constants.Swerve.maxSpeed).times(0.5);
    rotation = rAxis * Constants.Swerve.maxAngularVelocity * 0.5;
    s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
  }
}
