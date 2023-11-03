package frc3512.robot2022.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot2022.subsystems.DrivetrainSubsystem;

public class ResetModules extends CommandBase {

  private DrivetrainSubsystem m_subsystem;

  public ResetModules(DrivetrainSubsystem subsystem) {
    this.m_subsystem = subsystem;
    addRequirements(m_subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.resetModules();
  }
}
