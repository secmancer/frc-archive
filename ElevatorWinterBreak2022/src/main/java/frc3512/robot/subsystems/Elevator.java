package frc3512.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.logging.NetworkTableUtil;
import frc3512.robot.Constants;

public class Elevator extends SubsystemBase {

  private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State(0.0, 0.0);
  private boolean closedLoop = true;

  private final CANSparkMax m_elevatorGrbx;
  private final RelativeEncoder m_elevatorEncoder;

  private final LinearSystem<N2, N1, N1> m_elevatorPlant =
      LinearSystemId.createElevatorSystem(
          DCMotor.getNEO(1),
          Constants.Elevator.carriageMass,
          0.0181864,
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

  private final NetworkTableEntry positionEntry =
      NetworkTableUtil.makeDoubleEntry("/Diagnostics/Elevator/Position");
  private final NetworkTableEntry velocityEntry =
      NetworkTableUtil.makeDoubleEntry("/Diagnostics/Elevator/Velocity");

  public Elevator() {
    m_elevatorGrbx = new CANSparkMax(Constants.Elevator.elevatorMotorID, MotorType.kBrushless);
    m_elevatorEncoder = m_elevatorGrbx.getEncoder();

    m_elevatorGrbx.setSmartCurrentLimit(60);
    m_elevatorGrbx.setInverted(true);
    m_elevatorGrbx.set(0.0);
  }

  public void reset() {
    m_loop.reset(VecBuilder.fill(getPosition(), getRate()));
    m_lastProfiledReference = new TrapezoidProfile.State(getPosition(), getRate());
  }

  public void setSpeed(double percent) {
    closedLoop = false;
    m_elevatorGrbx.set(percent);
  }

  public void setGoal(double positionMeters) {
    if (!closedLoop) {
      closedLoop = true;
      reset();
    }

    if (positionMeters > 0.0) {
      goal = new TrapezoidProfile.State(positionMeters, 0.0);
    } else {
      goal = new TrapezoidProfile.State(0.0, 0.0);
    }
  }

  public double getPosition() {
    return Units.rotationsToRadians(
        m_elevatorEncoder.getPosition() * Constants.Elevator.drumRadius);
  }

  public double getRate() {
    return Units.rotationsPerMinuteToRadiansPerSecond(
        m_elevatorEncoder.getVelocity() * Constants.Elevator.drumRadius);
  }

  public void controllerPeriodic() {
    if (closedLoop) {
      m_lastProfiledReference =
          (new TrapezoidProfile(Constants.Elevator.m_constraints, goal, m_lastProfiledReference))
              .calculate(0.020);
      m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);

      m_loop.correct(VecBuilder.fill(getPosition()));

      m_loop.predict(0.020);

      double nextVoltage = m_loop.getU(0);
      m_elevatorGrbx.setVoltage(nextVoltage);
    }
  }

  @Override
  public void periodic() {
    controllerPeriodic();
    positionEntry.setDouble(getPosition());
    velocityEntry.setDouble(getRate());
  }
}
