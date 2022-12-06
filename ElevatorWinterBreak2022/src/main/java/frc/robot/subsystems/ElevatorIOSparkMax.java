package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;

public class ElevatorIOSparkMax implements ElevatorIO {
    
    private CANSparkMax m_elevatorGrbx;
    private Encoder m_elevatorEncoder;

    public ElevatorIOSparkMax() {
        m_elevatorGrbx = new CANSparkMax(9, MotorType.kBrushless);
        m_elevatorEncoder = new Encoder(4, 5);

        m_elevatorGrbx.setSmartCurrentLimit(60);
        m_elevatorGrbx.setInverted(true);
        m_elevatorGrbx.set(0.0);
        m_elevatorEncoder.setDistancePerPulse((2.0 * Math.PI * (0.0363728 / 2.0)) * 2.0 / 2048.0);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.positionRad = m_elevatorEncoder.getDistance();
        inputs.velocityRadPerSec = m_elevatorEncoder.getRate();
        inputs.appliedVolts = m_elevatorGrbx.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.currentAmps = new double[] { m_elevatorGrbx.getOutputCurrent() };
        inputs.tempCelcius = new double[] { m_elevatorGrbx.getMotorTemperature() };
    }

    @Override
    public void setVoltage(double volts) {
        m_elevatorGrbx.setVoltage(volts);
    }

    @Override
    public void reset() {
        m_elevatorEncoder.reset();
    }

}
