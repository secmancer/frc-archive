package frc3512.robot2022.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot2022.subsystems.Conveyor;
import frc3512.robot2022.subsystems.Intake;
import frc3512.robot2022.subsystems.Intake.IntakeDirection;

public class RunConveyor extends CommandBase {

  Conveyor m_conveyor;
  Intake m_intake;
  boolean m_ignoreSensors;
  Timer timer;

  /** Runs the conveyor */
  public RunConveyor(Conveyor conveyor, Intake intake, boolean ignoreSensors) {
    m_conveyor = conveyor;
    m_intake = intake;
    m_ignoreSensors = ignoreSensors;
    addRequirements(m_conveyor);
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (m_intake.m_currState == IntakeDirection.kOuttake) {
      m_conveyor.setConveyorToOuttake();
    } else if (m_conveyor.isTimeToShoot()) {
      m_conveyor.setConveyorTimeToShoot(true);
    } else if (!m_conveyor.isTimeToShoot()
        && !m_conveyor.isUpperSensorBlocked()
        && !m_conveyor.isLowerSensorBlocked()) {
      m_conveyor.setConveyorTimeToShoot(false);
    } else {
      m_conveyor.setConveyorToIdle();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1.0);
  }
}
