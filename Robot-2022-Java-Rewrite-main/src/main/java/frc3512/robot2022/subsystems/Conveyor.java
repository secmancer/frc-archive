package frc3512.robot2022.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.robot2022.Constants;

public class Conveyor extends SubsystemBase {

  boolean m_timeToShoot = false;

  Timer m_conveyorTimer;

  CANSparkMax m_conveyorMotor =
      new CANSparkMax(
          Constants.Conveyor.kConveyorMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

  DigitalInput m_upperSensor = new DigitalInput(Constants.Conveyor.kUpperSensorChannel);
  DigitalInput m_lowerSensor = new DigitalInput(Constants.Conveyor.kLowerSensorChannel);

  /** Subsystem class for the conveyor */
  public Conveyor() {
    CANSparkMaxUtil.SetCANSparkMaxBusUsage(m_conveyorMotor, Usage.kMinimal);
    m_conveyorMotor.setSmartCurrentLimit(80);
  }

  public void setConveyorToOuttake() {
    setConveyor(0.8);
  }

  public void setConveyorTimeToShoot(boolean ignoreSensors) {
    if (ignoreSensors) {
      setConveyor(-0.45);
    } else {
      setConveyor(-0.8);
    }
  }

  public void setConveyorToIdle() {
    setConveyor(0.0);
  }

  public void setConveyor(double speed) {
    m_conveyorMotor.set(-speed);
  }

  public void setTimeToShoot(boolean timeToShoot) {
    m_timeToShoot = timeToShoot;
  }

  public boolean isConveyorRunning() {
    return m_conveyorMotor.get() > 0.0;
  }

  public boolean isUpperSensorBlocked() {
    return !m_upperSensor.get();
  }

  public boolean isLowerSensorBlocked() {
    return !m_lowerSensor.get();
  }

  public boolean isTimeToShoot() {
    return m_timeToShoot;
  }

  @Override
  public void periodic() {}
}
