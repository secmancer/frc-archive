package frc3512.robot2022.commands.climbers;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot2022.subsystems.Climber;

public class DeployClimbers extends CommandBase {

  Climber m_climber;

  /** Deploys the climber */
  public DeployClimbers(Climber climber) {
    m_climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    if (m_climber.areDeployed()) {
      m_climber.stowClimbers();
    } else {
      m_climber.deployClimbers();
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
