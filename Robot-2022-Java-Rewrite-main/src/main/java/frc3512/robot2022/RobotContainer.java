package frc3512.robot2022;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc3512.robot2022.auto.AutonomousChooser;
import frc3512.robot2022.commands.climbers.DeployClimbers;
import frc3512.robot2022.commands.intake.DeployIntake;
import frc3512.robot2022.commands.intake.RunIntake;
import frc3512.robot2022.commands.swerve.DriveSwerveCommand;
import frc3512.robot2022.commands.swerve.ResetModules;
import frc3512.robot2022.commands.swerve.ZeroIMU;
import frc3512.robot2022.subsystems.BackFlywheel;
import frc3512.robot2022.subsystems.Climber;
import frc3512.robot2022.subsystems.DrivetrainSubsystem;
import frc3512.robot2022.subsystems.FrontFlywheel;
import frc3512.robot2022.subsystems.Intake;
import frc3512.robot2022.subsystems.Intake.IntakeDirection;
import frc3512.robot2022.subsystems.Vision;

public class RobotContainer {

  // Autonomous chooser
  private AutonomousChooser m_chooser = new AutonomousChooser();

  // Joysticks + XboxController
  private final XboxController m_controller =
      new XboxController(Constants.Joysticks.kXboxControllerPort);
  private final Joystick m_appendageStick1 = new Joystick(Constants.Joysticks.kAppendageStick1Port);
  private final Joystick m_appendageStick2 = new Joystick(Constants.Joysticks.kAppendageStick2Port);

  // Buttons
  private final JoystickButton m_deployIntakeButton = new JoystickButton(m_appendageStick2, 1);
  private final JoystickButton m_intakeButton = new JoystickButton(m_appendageStick2, 3);
  private final JoystickButton m_outtakeButton = new JoystickButton(m_appendageStick2, 4);
  private final JoystickButton m_deployClimbersButton = new JoystickButton(m_appendageStick1, 1);
  private final JoystickButton m_overrideLimitsButton = new JoystickButton(m_appendageStick2, 11);

  // Subsystems
  public final Vision m_vision = new Vision();
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  private final Intake m_intake = new Intake();
  private final Climber m_climber = new Climber();
  public final FrontFlywheel m_frontFlywheel = new FrontFlywheel();
  public final BackFlywheel m_backFlywheel = new BackFlywheel();

  public RobotContainer() {
    configureButtonBindings();
    configureJoystickControls();
    addAutons();
  }

  private void configureButtonBindings() {
    // Intake buttons
    m_deployIntakeButton.whenPressed(new DeployIntake(m_intake));
    m_intakeButton.whenHeld(new RunIntake(m_intake, IntakeDirection.kIntake));
    m_outtakeButton.whenHeld(new RunIntake(m_intake, IntakeDirection.kOuttake));

    // Climber buttons
    m_deployClimbersButton.whenPressed(new DeployClimbers(m_climber));
    m_overrideLimitsButton.whenPressed(new InstantCommand(m_climber::overrideLimits, m_climber));

    new JoystickButton(m_controller, XboxController.Button.kB.value)
        .whenPressed(new ZeroIMU(m_drivetrain));
    new JoystickButton(m_controller, XboxController.Button.kA.value)
        .whenHeld(new ResetModules(m_drivetrain));
  }

  private void configureJoystickControls() {
    m_drivetrain.setDefaultCommand(
        new DriveSwerveCommand(
            m_drivetrain,
            () -> m_controller.getLeftY(),
            () -> m_controller.getLeftX(),
            () -> m_controller.getRightX()));

    m_climber.setDefaultCommand(
        new RunCommand(
            () ->
                m_climber.setClimbers(
                    MathUtil.applyDeadband(m_appendageStick1.getRawAxis(1), 0.76),
                    MathUtil.applyDeadband(m_appendageStick2.getRawAxis(1), 0.8)),
            m_climber));
  }

  private void addAutons() {}

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
