package frc3512.robot2022.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot2022.subsystems.Intake;
import frc3512.robot2022.subsystems.Intake.IntakeDirection;

public class RunIntake extends CommandBase {

  IntakeDirection m_direction = IntakeDirection.kIdle;
  Intake m_intake;

  /** Runs the intake */
  public RunIntake(Intake intake, IntakeDirection direction) {
    m_intake = intake;
    m_direction = direction;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_direction == IntakeDirection.kIntake) {
      m_intake.runIntake();
    } else if (m_direction == IntakeDirection.kOuttake) {
      m_intake.runOuttake();
    } else {
      m_intake.runIdle();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
