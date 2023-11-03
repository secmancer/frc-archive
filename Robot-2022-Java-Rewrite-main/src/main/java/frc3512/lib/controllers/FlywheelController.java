package frc3512.lib.controllers;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc3512.lib.controllers.FlywheelConstants.FlywheelPose;
import frc3512.robot2022.GeneralConfig;

public class FlywheelController extends ControllerBase<N1, N1, N1> {

  /// Gear ratio from encoder to both flywheels
  private static final double kGearRatio = 1.0 / 1.0;

  /// Angle per encoder pulse.
  public static final double kDpP = (Math.PI * 2.0) * kGearRatio / 2048.0;

  private static final int kAngularVelocityShotThreshold = 25;
  private static final int kAngularVelocityRecoveryThreshold = 25;

  FlywheelPose m_pose;

  LinearSystem<N1, N1, N1> m_frontPlant =
      LinearSystemId.identifyVelocitySystem(
          FlywheelConstants.FrontFlywheel.kV, FlywheelConstants.FrontFlywheel.kA);
  LinearQuadraticRegulator<N1, N1, N1> m_frontLQR =
      new LinearQuadraticRegulator<N1, N1, N1>(
          m_frontPlant,
          VecBuilder.fill(50.0),
          VecBuilder.fill(12.0),
          GeneralConfig.kControllerPeriod);
  LinearPlantInversionFeedforward<N1, N1, N1> m_frontFF =
      new LinearPlantInversionFeedforward<N1, N1, N1>(
          m_frontPlant, GeneralConfig.kControllerPeriod);

  LinearSystem<N1, N1, N1> m_backPlant = getBackPlant();
  LinearQuadraticRegulator<N1, N1, N1> m_backLQR =
      new LinearQuadraticRegulator<N1, N1, N1>(
          m_backPlant,
          VecBuilder.fill(50.0),
          VecBuilder.fill(12.0),
          GeneralConfig.kControllerPeriod);
  LinearPlantInversionFeedforward<N1, N1, N1> m_backFF =
      new LinearPlantInversionFeedforward<N1, N1, N1>(m_backPlant, GeneralConfig.kControllerPeriod);

  boolean m_atGoal = false;

  public class State {
    public static final int kAngularVelocity = 0;
  }

  public class Input {
    public static final int kVoltage = 0;
  }

  public class Output {
    public static final int kAngularVelocity = 0;
  }

  FlywheelController(FlywheelPose pose) {
    reset();
    this.m_pose = pose;
  }

  public void setGoal(double angularVelocity) {
    if (m_nextR.get(0, 0) == angularVelocity) {
      return;
    }

    Matrix<N1, N1> u = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(angularVelocity);
    m_nextR = u;
    m_atGoal = false;
  }

  public double getGoal() {
    return m_nextR.get(0, 0);
  }

  public boolean atGoal() {
    return m_atGoal;
  }

  public void reset() {
    Matrix<N1, N1> blank = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.0);
    m_r = blank;
    m_nextR = blank;
  }

  @Override
  public Matrix<N1, N1> calculate(Matrix<N1, N1> x) {
    if (m_nextR.get(0, 0) == 0.0) {
      m_u = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.0);
    } else {
      if (m_pose == FlywheelPose.kFront) {
        m_u =
            m_frontLQR
                .calculate(x, m_r)
                .plus(m_frontFF.calculate(m_nextR))
                .plus(
                    new MatBuilder<>(Nat.N1(), Nat.N1()).fill(FlywheelConstants.FrontFlywheel.kS));
      } else if (m_pose == FlywheelPose.kBack) {
        m_u =
            m_backLQR
                .calculate(x, m_r)
                .plus(m_frontFF.calculate(m_nextR))
                .plus(new MatBuilder<>(Nat.N1(), Nat.N1()).fill(FlywheelConstants.BackFlywheel.kS));
      }
    }
    m_u = new Matrix<N1, N1>(StateSpaceUtil.desaturateInputVector(m_u, 12.0));
    updateAtGoal(m_nextR.get(0, 0) - x.get(0, 0));
    m_r = m_nextR;
    return m_u;
  }

  public static LinearSystem<N1, N1, N1> getBackPlant() {
    return LinearSystemId.identifyVelocitySystem(
        FlywheelConstants.BackFlywheel.kV, FlywheelConstants.BackFlywheel.kA);
  }

  public void updateAtGoal(double error) {
    if (m_atGoal && error > kAngularVelocityShotThreshold) {
      m_atGoal = false;
    } else if (!m_atGoal && error < kAngularVelocityRecoveryThreshold) {
      m_atGoal = true;
    }
  }
}
