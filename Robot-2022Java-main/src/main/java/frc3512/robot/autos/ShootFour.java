package frc3512.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc3512.robot.Constants;
import frc3512.robot.subsystems.drive.Swerve;
import frc3512.robot.subsystems.flywheels.BackFlywheel;
import frc3512.robot.subsystems.flywheels.FrontFlywheel;
import frc3512.robot.subsystems.intake.Intake;
import frc3512.robot.subsystems.vision.Vision;

public class ShootFour extends SequentialCommandGroup {
  public ShootFour(
      Swerve swerve,
      FrontFlywheel frontFlywheel,
      BackFlywheel backFlywheel,
      Intake intake,
      Vision vision) {
    PathConstraints constraints = new PathConstraints(1.0, 4.0);

    PathPlannerTrajectory driveBack =
        PathPlanner.loadPath("New Path", constraints);

    PPSwerveControllerCommand driveBackCommand =
        new PPSwerveControllerCommand(
            driveBack,
            swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(1.0, 0, 0),
            new PIDController(1.0, 0, 0),
            new PIDController(1.0, 0, 0),
            swerve::setModuleStates,
            swerve);

    addCommands(
        new InstantCommand(() -> swerve.resetOdometry(driveBack.getInitialHolonomicPose())),
        driveBackCommand);
  }
}
