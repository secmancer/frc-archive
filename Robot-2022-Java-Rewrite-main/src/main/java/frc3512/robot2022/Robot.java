package frc3512.robot2022;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimesliceRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public Robot() {
    super(0.005, 0.01);

    DriverStation.silenceJoystickConnectionWarning(true);
    LiveWindow.disableAllTelemetry();

    // Runs for 2 ms after first controller function
    schedule(() -> m_robotContainer.m_frontFlywheel.controllerPeriodic(), 0.0007);

    // Runs for 2 ms after robot periodic functions
    schedule(() -> m_robotContainer.m_backFlywheel.controllerPeriodic(), 0.0007);
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
