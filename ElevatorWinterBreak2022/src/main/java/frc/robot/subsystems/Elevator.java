package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorIO.ElevatorInputs;

public class Elevator extends SubsystemBase {

  private final ElevatorIO elevatorIO;
  private final ElevatorInputs elevatorInputs = new ElevatorInputs();

  private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal;

  private final LinearSystem<N2, N1, N1> m_elevatorPlant =
      LinearSystemId.createElevatorSystem(
          DCMotor.getNEO(1),
          Constants.Elevator.carriageMass,
          Constants.Elevator.drumRadius,
          Constants.Elevator.elevatorGearing);

  private final KalmanFilter<N2, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N2(),
          Nat.N1(),
          m_elevatorPlant,
          VecBuilder.fill(Units.inchesToMeters(2), Units.inchesToMeters(40)),
          VecBuilder.fill(0.001),
          0.020);

  private final LinearQuadraticRegulator<N2, N1, N1> m_controller =
      new LinearQuadraticRegulator<>(
          m_elevatorPlant,
          VecBuilder.fill(Units.inchesToMeters(1.0), Units.inchesToMeters(10.0)),
          VecBuilder.fill(12.0),
          0.020);

  private final LinearSystemLoop<N2, N1, N1> m_loop =
      new LinearSystemLoop<>(m_elevatorPlant, m_controller, m_observer, 12.0, 0.020);

  public Elevator(ElevatorIO io) {
    this.elevatorIO = io;

    reset();
    setGoal(0.0);
  }

  public void reset() {
    m_loop.reset(VecBuilder.fill(elevatorInputs.positionRad, elevatorInputs.velocityRadPerSec));
    m_lastProfiledReference =
        new TrapezoidProfile.State(elevatorInputs.positionRad, elevatorInputs.velocityRadPerSec);
  }

  public void setGoal(double positionMeters) {
    goal = new TrapezoidProfile.State(positionMeters, 0.0);
  }

  public void controllerPeriodic() {
    m_lastProfiledReference =
        (new TrapezoidProfile(Constants.Elevator.m_constraints, goal, m_lastProfiledReference)).calculate(0.020);
    m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);

    m_loop.correct(VecBuilder.fill(elevatorInputs.positionRad));

    m_loop.predict(0.020);

    double nextVoltage = m_loop.getU(0);
    elevatorIO.setVoltage(nextVoltage);
  }

  @Override
  public void periodic() {
    controllerPeriodic();

    elevatorIO.updateInputs(elevatorInputs);
    Logger.getInstance().processInputs("Elevator", elevatorInputs);
  }
}
