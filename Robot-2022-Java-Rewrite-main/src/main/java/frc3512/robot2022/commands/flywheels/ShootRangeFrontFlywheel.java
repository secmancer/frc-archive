package frc3512.robot2022.commands.flywheels;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot2022.subsystems.FrontFlywheel;

public class ShootRangeFrontFlywheel extends CommandBase {

  private FrontFlywheel m_flywheel;
  private boolean m_useRange;

  public ShootRangeFrontFlywheel(FrontFlywheel flywheel, boolean useRange) {
    this.m_flywheel = flywheel;
    this.m_useRange = useRange;
    addRequirements(m_flywheel);
  }

  @Override
  public void initialize() {
    m_flywheel.setGoalFromRange(m_useRange);
  }

  @Override
  public boolean isFinished() {
    return m_flywheel.atGoal();
  }
}
