package frc3512.robot2022.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc3512.robot2022.commands.swerve.AutoAim;
import frc3512.robot2022.subsystems.BackFlywheel;
import frc3512.robot2022.subsystems.Conveyor;
import frc3512.robot2022.subsystems.DrivetrainSubsystem;
import frc3512.robot2022.subsystems.FrontFlywheel;
import frc3512.robot2022.subsystems.Intake;

public class VisionShot extends SequentialCommandGroup {

  private DrivetrainSubsystem m_swerve;
  private FrontFlywheel m_frontFlywheel;
  private BackFlywheel m_backFlywheel;
  private Conveyor m_conveyor;
  private Intake m_intake;

  public VisionShot(
      DrivetrainSubsystem swerve,
      FrontFlywheel frontFlywheel,
      BackFlywheel backFlywheel,
      Conveyor conveyor,
      Intake intake) {
    this.m_frontFlywheel = frontFlywheel;
    this.m_backFlywheel = backFlywheel;
    this.m_conveyor = conveyor;
    this.m_intake = intake;
    addCommands(
        new AutoAim(m_swerve),
        new RangeShot(m_frontFlywheel, m_backFlywheel, m_conveyor, m_intake));
  }
}
