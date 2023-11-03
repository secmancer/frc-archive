package frc3512.robot2022.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc3512.robot2022.commands.flywheels.ShootRangeBackFlywheel;
import frc3512.robot2022.commands.flywheels.ShootRangeFrontFlywheel;
import frc3512.robot2022.commands.intake.RunConveyor;
import frc3512.robot2022.subsystems.BackFlywheel;
import frc3512.robot2022.subsystems.Conveyor;
import frc3512.robot2022.subsystems.FrontFlywheel;
import frc3512.robot2022.subsystems.Intake;

public class RangeShot extends SequentialCommandGroup {

  private FrontFlywheel m_frontFlywheel;
  private BackFlywheel m_backFlywheel;
  private Conveyor m_conveyor;
  private Intake m_intake;

  public RangeShot(
      FrontFlywheel frontFlywheel, BackFlywheel backFlywheel, Conveyor conveyor, Intake intake) {
    this.m_frontFlywheel = frontFlywheel;
    this.m_backFlywheel = backFlywheel;
    this.m_conveyor = conveyor;
    this.m_intake = intake;
    addCommands(
        new ParallelCommandGroup(
            new ShootRangeFrontFlywheel(m_frontFlywheel, true),
            new ShootRangeBackFlywheel(m_backFlywheel, true)),
        new RunConveyor(m_conveyor, m_intake, false));
  }
}
