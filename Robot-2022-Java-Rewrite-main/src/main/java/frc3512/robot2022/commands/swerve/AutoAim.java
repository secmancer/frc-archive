package frc3512.robot2022.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot2022.subsystems.DrivetrainSubsystem;
import frc3512.robot2022.subsystems.Vision;

public class AutoAim extends CommandBase {

  private final DrivetrainSubsystem m_swerve;
  private PIDController m_pid;

  public AutoAim(DrivetrainSubsystem swerve) {
    this.m_swerve = swerve;
    m_pid = new PIDController(1.0, 0.0, 0.0);
    addRequirements(m_swerve);
  }

  @Override
  public void initialize() {
    m_pid.reset();
  }

  @Override
  public void execute() {
    if (!isFinished()) {
      double speed = m_pid.calculate(Vision.getLatestYaw(), 0);
      m_swerve.drive(0, 0, -speed, false);
    } else {
      m_swerve.drive(0.0, 0.0, 0.0, false);
    }
  }

  @Override
  public boolean isFinished() {
    return Math.abs(Vision.getLatestYaw()) < 0.1;
  }
}
