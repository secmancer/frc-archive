package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class ElevatorIOSparkMax implements ElevatorIO {

  private CANSparkMax m_elevatorGrbx;
  private Encoder m_elevatorEncoder;

  public ElevatorIOSparkMax() {
    m_elevatorGrbx = new CANSparkMax(Constants.Elevator.elevatorMotorID, MotorType.kBrushless);
    m_elevatorEncoder =
        new Encoder(Constants.Elevator.elevatorEncoderA, Constants.Elevator.elevatorEncoderB);

    m_elevatorGrbx.setSmartCurrentLimit(60);
    m_elevatorGrbx.setInverted(true);
    m_elevatorGrbx.set(0.0);
    m_elevatorEncoder.setDistancePerPulse(2.0 * Math.PI * Constants.Elevator.drumRadius / 2048.0);
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.positionRad = m_elevatorEncoder.getDistance();
    inputs.velocityRadPerSec = m_elevatorEncoder.getRate();
    inputs.appliedVolts = m_elevatorGrbx.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = new double[] {m_elevatorGrbx.getOutputCurrent()};
    inputs.tempCelcius = new double[] {m_elevatorGrbx.getMotorTemperature()};
  }

  @Override
  public void setVoltage(double volts) {
    m_elevatorGrbx.setVoltage(volts);
  }

  @Override
  public void setSpeed(double percent) {
    m_elevatorGrbx.set(percent);
  }
}
