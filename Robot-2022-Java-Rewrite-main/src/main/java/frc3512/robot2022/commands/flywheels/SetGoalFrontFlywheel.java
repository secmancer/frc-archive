package frc3512.robot2022.commands.flywheels;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot2022.subsystems.FrontFlywheel;

public class SetGoalFrontFlywheel extends CommandBase {

  private FrontFlywheel m_flywheel;
  private final double m_rpm;

  public SetGoalFrontFlywheel(FrontFlywheel flywheel, double rpm) {
    this.m_flywheel = flywheel;
    this.m_rpm = rpm;
    addRequirements(m_flywheel);
  }

  @Override
  public void initialize() {
    m_flywheel.setGoal(m_rpm);
  }

  @Override
  public boolean isFinished() {
    return m_flywheel.atGoal();
  }
}
