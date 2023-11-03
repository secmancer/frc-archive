package frc3512.robot2022.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import frc3512.lib.controllers.FlywheelConstants;
import frc3512.lib.controllers.FlywheelController;
import frc3512.lib.controllers.FlywheelController.Input;
import frc3512.lib.subsystems.ControlledSubsystem;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.lib.util.InterpolatingTreeMap;
import frc3512.lib.util.NetworkTableUtil;
import frc3512.robot2022.Constants;
import frc3512.robot2022.GeneralConfig;

public class FrontFlywheel extends ControlledSubsystem<N1, N1, N1> {

  private boolean setFromRange;

  CANSparkMax m_frontGrbx =
      new CANSparkMax(Constants.FrontFlywheel.kFrontMotorID, MotorType.kBrushless);

  Encoder m_frontEncoder =
      new Encoder(Constants.FrontFlywheel.kFrontEncoderA, Constants.FrontFlywheel.kFrontEncoderB);

  LinearSystem<N1, N1, N1> m_plant =
      LinearSystemId.identifyVelocitySystem(
          FlywheelConstants.FrontFlywheel.kV, FlywheelConstants.FrontFlywheel.kA);

  KalmanFilter<N1, N1, N1> m_observer =
      new KalmanFilter<N1, N1, N1>(
          Nat.N1(),
          Nat.N1(),
          m_plant,
          new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.25),
          new MatBuilder<>(Nat.N1(), Nat.N1()).fill(2.5),
          GeneralConfig.kControllerPeriod);

  FlywheelController m_controller;

  InterpolatingTreeMap<Double, Double> m_table = new InterpolatingTreeMap<>();

  Matrix<N1, N1> m_u;

  double m_range;
  double m_angle;
  double m_lastAngle;
  double m_time = Timer.getFPGATimestamp();
  double m_lastTime = m_time - GeneralConfig.kControllerPeriod;

  double m_angularVelocity;
  LinearFilter m_velocityFilter = LinearFilter.movingAverage(4);

  double m_testThrottle = 0.0;

  NetworkTableEntry m_percentageEntry =
      NetworkTableUtil.MakeDoubleEntry("/Diagnostics/Front Flywheel/Percent");

  public FrontFlywheel() {
    m_frontGrbx.setIdleMode(IdleMode.kCoast);
    m_frontGrbx.setSmartCurrentLimit(40);

    m_frontGrbx.set(0.0);

    m_frontGrbx.setInverted(true);

    m_frontEncoder.setDistancePerPulse(FlywheelController.kDpP);
    m_frontEncoder.setSamplesToAverage(5);
    m_frontEncoder.setReverseDirection(true);

    CANSparkMaxUtil.SetCANSparkMaxBusUsage(m_frontGrbx, Usage.kMinimal);

    m_table.put(12.0, 359.0);
    m_table.put(24.0, 359.0);
    m_table.put(48.0, 359.0);
    m_table.put(72.0, 359.0);
    m_table.put(96.0, 359.0);
    m_table.put(108.0, 414.89);

    reset();
    setGoal(0.0);
  }

  public double getAngle() {
    return m_frontEncoder.getDistance();
  }

  public double getAngularVelocity() {
    return m_angularVelocity;
  }

  public void setGoal(double velocity) {
    m_controller.setGoal(velocity);
  }

  public void setGoalFromRange(boolean setGoal) {
    setFromRange = setGoal;
  }

  public double getGoal() {
    return m_controller.getGoal();
  }

  public double getGoalFromRange() {
    return m_table.get(m_range);
  }

  public void stopFlywheel() {
    setGoal(0.0);
  }

  public boolean atGoal() {
    return m_controller.atGoal();
  }

  public boolean isOn() {
    return getGoal() > 0.0;
  }

  public void reset() {
    m_observer.reset();
    m_controller.reset();
    m_u = new Matrix<>(Nat.N1(), Nat.N1());
    m_angle = getAngle();
    m_lastAngle = m_angle;
  }

  public void setVoltage(double voltage) {
    m_frontGrbx.setVoltage(voltage);
  }

  public double throttleToReference(double throttle) {
    // Remaps inputs from [1.. -1] to [0..1]
    var remap = (1.0 - throttle) / 2.0;
    // Rescales the result to [400.0...800.0]
    var kLow = 100.0;
    var kHigh = 1500.0;
    var rescale = kLow + (kHigh - kLow) * remap;
    return Math.round(rescale);
  }

  @Override
  public void controllerPeriodic() {
    updateDT();

    m_observer.predict(m_u, getDT());

    m_angle = getAngle();
    m_time = Timer.getFPGATimestamp();

    m_angularVelocity = m_velocityFilter.calculate((m_angle - m_lastAngle) / (m_time - m_lastTime));

    Matrix<N1, N1> y = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(getAngularVelocity());
    m_observer.correct(m_controller.getInputs(), y);
    m_u = m_controller.calculate(m_observer.getXhat());
    setVoltage(m_u.get(0, Input.kVoltage));

    if (Vision.hasTargets()) {
      m_range = Vision.getLatestRange();
    }

    if (setFromRange) {
      setGoal(m_table.get(m_range));
    }

    m_lastAngle = m_angle;
    m_lastTime = m_time;
  }
}
