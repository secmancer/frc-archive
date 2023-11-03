package frc3512.robot2022.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutonomousChooser {
  static SendableChooser<Command> chooser = new SendableChooser<Command>();

  public AutonomousChooser() {
    chooser.setDefaultOption("No-op", new InstantCommand());

    SmartDashboard.putData(chooser);
  }

  public void addAuton(String name, Command command) {
    chooser.addOption(name, command);
  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}
