package frc3512.robot2022.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc3512.robot2022.commands.flywheels.SetGoalBackFlywheel;
import frc3512.robot2022.commands.flywheels.SetGoalFrontFlywheel;
import frc3512.robot2022.commands.intake.RunConveyor;
import frc3512.robot2022.subsystems.BackFlywheel;
import frc3512.robot2022.subsystems.Conveyor;
import frc3512.robot2022.subsystems.FrontFlywheel;
import frc3512.robot2022.subsystems.Intake;

public class ManualShot extends SequentialCommandGroup {

  private FrontFlywheel m_frontFlywheel;
  private BackFlywheel m_backFlywheel;
  private Conveyor m_conveyor;
  private Intake m_intake;
  public double m_rpm;

  public ManualShot(
      FrontFlywheel frontFlywheel,
      BackFlywheel backFlywheel,
      Conveyor conveyor,
      Intake intake,
      double rpm) {
    this.m_frontFlywheel = frontFlywheel;
    this.m_backFlywheel = backFlywheel;
    this.m_conveyor = conveyor;
    this.m_intake = intake;
    this.m_rpm = rpm;
    addCommands(
        new ParallelCommandGroup(
            new SetGoalFrontFlywheel(m_frontFlywheel, rpm),
            new SetGoalBackFlywheel(m_backFlywheel, rpm)),
        new RunConveyor(m_conveyor, m_intake, false));
  }
}
