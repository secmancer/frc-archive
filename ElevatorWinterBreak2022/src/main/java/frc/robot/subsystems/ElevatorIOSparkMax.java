package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class ElevatorIOSparkMax implements ElevatorIO {

  private CANSparkMax m_elevatorGrbx;

  public ElevatorIOSparkMax() {
    m_elevatorGrbx = new CANSparkMax(Constants.Elevator.elevatorMotorID, MotorType.kBrushless);

    m_elevatorGrbx.setSmartCurrentLimit(60);
    m_elevatorGrbx.set(0.0);
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.positionRad = 0.0;
    inputs.velocityRadPerSec = 0.0;
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
